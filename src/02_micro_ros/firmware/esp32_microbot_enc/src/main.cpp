// AIR26 Workshop 02 — ESP32 micro-ROS firmware, ENCODER (closed-loop) variant.
//
// Same robot, same ROS 2 interface as firmware/esp32_microbot, BUT the motors are the
// JGB37-520 DC gear motors with magnetic quadrature encoders, and this firmware CLOSES THE
// SPEED LOOP: it reads the encoders, works out how fast each side is *actually* turning
// (wheel diameter x rev/s), and trims the PWM so the measured speed matches the /cmd_vel
// command. Result: the distance/speed driven actually matches what was asked — no more
// open-loop guesswork through MAX_LIN.
//
//   IMPORTANT: this firmware still publishes NO odometry. The encoders are used ONLY to
//   regulate wheel speed on-board. Nothing new goes on the wire; the ROS interface is
//   byte-for-byte identical to the open-loop firmware:
//     publishes:  /ultrasonic/front|left|right   (std_msgs/UInt8, cm)   <- 3x HC-SR04
//     subscribes: /cmd_vel                        (geometry_msgs/Twist) -> L298N motors
//
// LATENCY BRANCH (10-fix): ultrasonics are std_msgs/UInt8 cm, best-effort QoS, round-robin
// polling, WiFi power-save off. A host-side bridge re-inflates UInt8 -> Range for RViz.
// See LATENCY.md.
//
// Transport: WiFi/UDP to the micro-ROS Agent. Flash over USB, then it runs untethered.
//
// >>> EDIT THE CONFIG BLOCK BELOW for your WiFi, your Agent's IP, and your wiring. <<<
// Motor: JGB37-520 (6-pin: M1, GND, C2, C1, VCC, M2). M1/M2 -> L298N motor output;
//        VCC/GND power the encoder (3.3 V); C1/C2 are the two quadrature channels (A/B).

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int8.h>
#include <geometry_msgs/msg/twist.h>

// ============================ CONFIG — EDIT ME ============================
// WiFi + Agent
static char        WIFI_SSID[]  = "LSPrmn60x";
static char        WIFI_PASS[]  = "pi=3.14159";
static uint8_t     AGENT_IP[4]  = {10, 185, 122, 251};  // the PC running micro_ros_agent
static uint16_t    AGENT_PORT   = 8888;

// HC-SR04 ultrasonics: {trig, echo} pins  (ESP32-S3 safe GPIOs)
// NOTE: HC-SR04 echo is 5V - use a level shifter / voltage divider to the S3 (3.3V).
static const int US_FRONT[2] = {10, 11};
static const int US_LEFT[2]  = {12, 13};
static const int US_RIGHT[2] = {14, 21};

// L298N motor driver — left channel (both left wheels) / right channel (both right wheels)
static const int L_EN = 6,  L_IN1 = 4, L_IN2 = 5;      // ENA, IN1, IN2
static const int R_EN = 16, R_IN1 = 7, R_IN2 = 15;     // ENB, IN3, IN4

// JGB37-520 quadrature encoders: {A=C1, B=C2} pins, one encoder per side (free S3 GPIOs).
// C1/C2 to these pins; encoder VCC->3V3, GND->GND. If a side reads speed with the WRONG
// SIGN (integrator runs away / motor fights the command), swap that side's A/B pins.
static const int ENC_L[2] = {17, 18};   // left  {A, B}
static const int ENC_R[2] = {8, 9};     // right {A, B}

// Kinematics / tuning
static const float WHEEL_SEP   = 0.28f;   // m, left-right track width
static const float MAX_LIN     = 0.25f;   // m/s at full PWM — here only a feedforward guess;
                                          // the PI loop corrects whatever it gets wrong.
static const int   PWM_FREQ    = 1000;    // Hz
static const int   US_MAX_M    = 2;       // clamp ultrasonic range (m)

// ---- Encoder -> real wheel speed (THIS is where wheel diameter finally matters) ----
static const float WHEEL_DIA   = 0.065f;  // m (65 mm) — MEASURE your actual wheel!
static const int   ENC_PPR     = 11;      // encoder lines per MOTOR-shaft rev, per channel
static const float GEAR_RATIO  = 30.0f;   // JGB37-520 motor:output gearbox (CHECK your unit)
// We interrupt on ONE channel's rising edge (1x decoding), so we accumulate PPR*gear ticks
// per OUTPUT-shaft (wheel) revolution. Distance per tick = pi*WHEEL_DIA / ticks_per_rev.
static const float ENC_TICKS_PER_REV = ENC_PPR * GEAR_RATIO;      // e.g. 11*30 = 330

// Per-side speed PI controller (starting values — students tune these).
static const float CONTROL_HZ = 50.0f;    // control-loop rate
static const float KP = 2.0f;             // proportional gain (duty per m/s error)
static const float KI = 6.0f;             // integral gain
// ========================================================================

// ---- micro-ROS handles ----
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t pub_front, pub_left, pub_right;
rcl_subscription_t sub_cmd;
rcl_timer_t timer;          // ultrasonic round-robin
rcl_timer_t ctrl_timer;     // encoder speed loop
std_msgs__msg__UInt8 range_front, range_left, range_right;   // centimetres (0..US_MAX_M*100)
geometry_msgs__msg__Twist cmd_msg;

// ---- one skid-steer side: encoder count (ISR) + speed target + PI integrator ----
struct SideCtl {
  int en, in1, in2;             // L298N pins
  const int* enc;               // {A, B} pins
  volatile long ticks;          // signed encoder count (updated in ISR)
  long  last_ticks;             // snapshot at previous control tick
  float target_v;               // commanded wheel speed (m/s)
  float integ;                  // PI integrator state
};
SideCtl L = {L_EN, L_IN1, L_IN2, ENC_L, 0, 0, 0.0f, 0.0f};
SideCtl R = {R_EN, R_IN1, R_IN2, ENC_R, 0, 0, 0.0f, 0.0f};

// ---- agent connection state machine (standard micro-ROS reconnect pattern) ----
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;

#define RCCHECK(fn)    { if ((fn) != RCL_RET_OK) return false; }
#define EXEC_EVERY(MS, X)  do { static volatile int64_t t=-1; \
  if (t==-1) t=uxr_millis(); if ((int32_t)(uxr_millis()-t) > (MS)) { X; t=uxr_millis(); } } while (0)

// === CHECKPOINT: encoder_isr ===
// Quadrature: interrupt on channel A rising, direction from channel B level. Forward -> ++.
// (If a wheel counts the wrong way, swap that side's ENC_* {A,B} pins in the config block.)
void IRAM_ATTR isr_left()  { if (digitalRead(ENC_L[1])) L.ticks--; else L.ticks++; }
void IRAM_ATTR isr_right() { if (digitalRead(ENC_R[1])) R.ticks--; else R.ticks++; }
// === END CHECKPOINT: encoder_isr ===

// ---- HC-SR04: one blocking ping -> metres ----
float read_ultrasonic(const int pins[2]) {
  digitalWrite(pins[0], LOW);  delayMicroseconds(2);
  digitalWrite(pins[0], HIGH); delayMicroseconds(10);
  digitalWrite(pins[0], LOW);
  long us = pulseIn(pins[1], HIGH, US_MAX_M * 6000);   // timeout ~ round trip for US_MAX_M
  if (us == 0) return (float)US_MAX_M;                 // no echo -> max range
  float m = (us * 0.000343f) / 2.0f;
  return m > US_MAX_M ? (float)US_MAX_M : m;
}

// ---- L298N: signed [-1,1] command per side ----
void drive_side(int en, int in1, int in2, float cmd) {
  cmd = constrain(cmd, -1.0f, 1.0f);
  digitalWrite(in1, cmd >= 0);
  digitalWrite(in2, cmd < 0);
  analogWrite(en, (int)(fabs(cmd) * 255));
}

// ---- /cmd_vel -> per-side SPEED TARGETS (m/s). The control loop does the driving. ----
void on_cmd(const void* msgin) {
  const geometry_msgs__msg__Twist* m = (const geometry_msgs__msg__Twist*)msgin;
  float v = m->linear.x, w = m->angular.z;
  L.target_v = v - w * WHEEL_SEP / 2.0f;
  R.target_v = v + w * WHEEL_SEP / 2.0f;
}

// === CHECKPOINT: closed_loop_velocity ===
// One PI step for a side: measured speed -> duty. Feedforward (target/MAX_LIN) gives a fast
// first guess; P+I correct whatever MAX_LIN / battery / load got wrong. This is exactly the
// step the open-loop firmware CAN'T do, because without encoders it has no measured speed.
float pi_step(SideCtl& s, float v_meas, float dt) {
  float err = s.target_v - v_meas;
  s.integ += err * dt;
  // anti-windup: keep the integral term within the actuator range [-1, 1].
  if (KI * s.integ >  1.0f) s.integ =  1.0f / KI;
  if (KI * s.integ < -1.0f) s.integ = -1.0f / KI;
  float duty = s.target_v / MAX_LIN + KP * err + KI * s.integ;   // feedforward + PI
  return constrain(duty, -1.0f, 1.0f);
}

// ---- control timer: turn encoder ticks into real m/s and regulate both sides ----
void on_control(rcl_timer_t*, int64_t) {
  static uint32_t last_us = 0;
  uint32_t now = micros();
  float dt = last_us ? (now - last_us) * 1e-6f : (1.0f / CONTROL_HZ);
  last_us = now;
  if (dt < 1e-4f) return;                          // guard against a zero/tiny dt

  long lt = L.ticks, rt = R.ticks;                 // 32-bit reads are atomic on ESP32
  // ticks -> wheel revs -> metres travelled -> m/s. WHEEL_DIA enters right here.
  float mpt = 3.14159265f * WHEEL_DIA / ENC_TICKS_PER_REV;   // metres per tick
  float vL = (lt - L.last_ticks) * mpt / dt;
  float vR = (rt - R.last_ticks) * mpt / dt;
  L.last_ticks = lt;  R.last_ticks = rt;

  drive_side(L.en, L.in1, L.in2, pi_step(L, vL, dt));
  drive_side(R.en, R.in1, R.in2, pi_step(R, vR, dt));
}
// === END CHECKPOINT: closed_loop_velocity ===

// ---- read one sensor -> clamp to centimetres -> publish its UInt8 ----
void poll_and_publish(const int pins[2], std_msgs__msg__UInt8* msg, rcl_publisher_t* pub) {
  int cm = (int)(read_ultrasonic(pins) * 100.0f + 0.5f);   // metres -> cm, rounded
  msg->data = (uint8_t)(cm > 255 ? 255 : cm);              // US_MAX_M*100 <= 200, safe
  rcl_publish(pub, msg, NULL);
}

// ---- timer: weighted round-robin so ONE blocking pulseIn runs per tick, not three ----
// cycle [F, L, F, R]: at a 50 ms tick, front updates every 100 ms (~10 Hz), each side
// every 200 ms (~5 Hz). Front is the obstacle-critical sensor, so it is polled 2x the sides.
void on_timer(rcl_timer_t*, int64_t) {
  static uint8_t phase = 0;
  switch (phase) {
    case 0: case 2: poll_and_publish(US_FRONT, &range_front, &pub_front); break;
    case 1:         poll_and_publish(US_LEFT,  &range_left,  &pub_left);  break;
    case 3:         poll_and_publish(US_RIGHT, &range_right, &pub_right); break;
  }
  phase = (phase + 1) & 3;
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "microbot_esp32", "", &support));

  // Best-effort + volatile QoS: sensor data is fire-and-forget. Over lossy WiFi/UDP the
  // default RELIABLE QoS (ACKs + retransmits + history queue) is the main tail-latency
  // source; drop it. A stale reading is worthless — next sample is <=200 ms away.
  RCCHECK(rclc_publisher_init_best_effort(&pub_front, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/ultrasonic/front"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_left, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/ultrasonic/left"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_right, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "/ultrasonic/right"));
  // cmd_vel best-effort too: we want the freshest command, not a retransmitted stale one.
  RCCHECK(rclc_subscription_init_best_effort(&sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), on_timer));
  RCCHECK(rclc_timer_init_default(&ctrl_timer, &support,
    RCL_MS_TO_NS((int)(1000.0f / CONTROL_HZ)), on_control));

  // fresh start: don't let counts accumulated before connect spike the first control tick.
  L.last_ticks = L.ticks;  R.last_ticks = R.ticks;
  L.integ = R.integ = 0.0f;  L.target_v = R.target_v = 0.0f;

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));   // 2 timers + sub
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &ctrl_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, &on_cmd, ON_NEW_DATA));
  return true;
}

void destroy_entities() {
  rmw_context_t* rmw = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw, 0);
  rcl_publisher_fini(&pub_front, &node);
  rcl_publisher_fini(&pub_left, &node);
  rcl_publisher_fini(&pub_right, &node);
  rcl_subscription_fini(&sub_cmd, &node);
  rcl_timer_fini(&timer);
  rcl_timer_fini(&ctrl_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  // motor + sensor GPIO
  for (int p : {L_EN, L_IN1, L_IN2, R_EN, R_IN1, R_IN2,
                US_FRONT[0], US_LEFT[0], US_RIGHT[0]}) pinMode(p, OUTPUT);
  for (int p : {US_FRONT[1], US_LEFT[1], US_RIGHT[1]}) pinMode(p, INPUT);
  drive_side(L_EN, L_IN1, L_IN2, 0);
  drive_side(R_EN, R_IN1, R_IN2, 0);

  // === CHECKPOINT: encoder_isr ===
  // encoder channels as inputs + attach the quadrature interrupts.
  for (int p : {ENC_L[0], ENC_L[1], ENC_R[0], ENC_R[1]}) pinMode(p, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L[0]), isr_left,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R[0]), isr_right, RISING);
  // === END CHECKPOINT: encoder_isr ===

  // === CHECKPOINT: serial_diag ===
  // Boot/WiFi/Agent diagnostics on the USB serial console (115200). Comment out this
  // block to run fully headless. `pio device monitor` shows the IP + connection state.
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.printf("[microbot] boot. connecting to WiFi SSID='%s' ...\n", WIFI_SSID);

  //old esp32-dev//   Manual, non-blocking WiFi join with status logging (don't hang forever like the
  //old esp32-dev//   default set_microros_wifi_transports loop). status codes: 0=IDLE 1=NO_SSID_AVAIL
  //old esp32-dev//   3=CONNECTED 4=CONNECT_FAILED 6=DISCONNECTED. NO_SSID on a classic ESP32 usually
  //old esp32-dev//   means the SSID is 5 GHz only (ESP32 is 2.4 GHz). Scan results are printed too.
  // Manual, non-blocking WiFi join with status logging. status codes: 0=IDLE
  // 1=NO_SSID_AVAIL 3=CONNECTED 4=CONNECT_FAILED 6=DISCONNECTED. Scan results printed too.
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);   // disable modem power-save: default sleep adds 100-200 ms of RX
                          // latency + jitter to incoming /cmd_vel. Costs battery, worth it.
  int n = WiFi.scanNetworks();
  Serial.printf("[microbot] scan found %d networks:\n", n);
  for (int i = 0; i < n; i++)
    Serial.printf("    '%s'  rssi=%d  ch=%d  enc=%d\n",
                  WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i), WiFi.encryptionType(i));
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
    delay(1000);
    Serial.printf("[microbot] ...wifi status=%d\n", (int)WiFi.status());
  }
  Serial.printf("[microbot] WiFi status=%d  IP=%s  RSSI=%d dBm\n",
                (int)WiFi.status(), WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  Serial.printf("[microbot] agent target = %d.%d.%d.%d:%u\n",
                AGENT_IP[0], AGENT_IP[1], AGENT_IP[2], AGENT_IP[3], AGENT_PORT);
  // === END CHECKPOINT: serial_diag ===

  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
}

void loop() {
  // === CHECKPOINT: serial_diag ===
  // Print agent-connection state whenever it changes (heartbeat every 2 s while WAITING).
  static AgentState last_state = AGENT_DISCONNECTED;
  static const char* NM[] = {"WAITING_AGENT", "AGENT_AVAILABLE", "AGENT_CONNECTED", "AGENT_DISCONNECTED"};
  if (state != last_state) { Serial.printf("[microbot] state -> %s\n", NM[state]); last_state = state; }
  if (state == WAITING_AGENT) EXEC_EVERY(2000, Serial.println("[microbot] waiting for agent ping..."));
  // === END CHECKPOINT: serial_diag ===

  // standard micro-ROS reconnect lifecycle: survive the Agent restarting
  switch (state) {
    case WAITING_AGENT:
      EXEC_EVERY(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                              ? AGENT_AVAILABLE : WAITING_AGENT);
      break;
    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroy_entities();
      break;
    case AGENT_CONNECTED:
      // 1 attempt, not 3: a failed 3-try ping blocks up to 300 ms right next to the
      // executor spin. 1 try caps the stall at 100 ms; a real drop is still caught.
      EXEC_EVERY(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                              ? AGENT_CONNECTED : AGENT_DISCONNECTED);
      if (state == AGENT_CONNECTED)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
      break;
    case AGENT_DISCONNECTED:
      L.target_v = R.target_v = 0.0f;        // safety: stop on link loss
      drive_side(L_EN, L_IN1, L_IN2, 0);
      drive_side(R_EN, R_IN1, R_IN2, 0);
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}
