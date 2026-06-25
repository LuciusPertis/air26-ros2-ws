// AIR26 Workshop 02 — ESP32 micro-ROS firmware for the obstacle-avoider rover.
//
// This is the REAL robot's brain-stem. It exposes the SAME ROS 2 interface as the
// MuJoCo sim, so the microbot_behaviors nodes drive it unchanged:
//     publishes: /ultrasonic/front|left|right   (sensor_msgs/Range)   <- 3x HC-SR04
//     subscribes: /cmd_vel                       (geometry_msgs/Twist) -> L298N motors
//
// Transport: WiFi/UDP to the micro-ROS Agent. Flash over USB, then it runs untethered.
//
// >>> EDIT THE CONFIG BLOCK BELOW for your WiFi, your Agent's IP, and your wiring. <<<
// Reference hardware: ESP32 DevKit + L298N dual H-bridge (skid-steer: left pair / right
// pair) + 3x HC-SR04. If you use a different motor driver, only set_motor()/setup change.

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>

// ============================ CONFIG — EDIT ME ============================
// WiFi + Agent
static char        WIFI_SSID[]  = "LSPrmn60x";
static char        WIFI_PASS[]  = "pi=3.14159";
static uint8_t     AGENT_IP[4]  = {10, 185, 122, 251};  // the PC running micro_ros_agent
static uint16_t    AGENT_PORT   = 8888;

// HC-SR04 ultrasonics: {trig, echo} pins
static const int US_FRONT[2] = {26, 25};
static const int US_LEFT[2]  = {33, 32};
static const int US_RIGHT[2] = {18, 19};

// L298N motor driver — left channel (both left wheels) / right channel (both right wheels)
static const int L_EN = 13, L_IN1 = 12, L_IN2 = 14;     // ENA, IN1, IN2
static const int R_EN = 27, R_IN1 = 16, R_IN2 = 17;     // ENB, IN3, IN4

// Kinematics / tuning
static const float WHEEL_SEP   = 0.28f;   // m, left-right track width
static const float MAX_LIN     = 0.25f;   // m/s mapped to full PWM
static const int   PWM_FREQ    = 1000;    // Hz
static const int   US_MAX_M    = 2;       // clamp ultrasonic range (m)
// ========================================================================

// ---- micro-ROS handles ----
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t pub_front, pub_left, pub_right;
rcl_subscription_t sub_cmd;
rcl_timer_t timer;
sensor_msgs__msg__Range range_front, range_left, range_right;
geometry_msgs__msg__Twist cmd_msg;

// ---- agent connection state machine (standard micro-ROS reconnect pattern) ----
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;

#define RCCHECK(fn)    { if ((fn) != RCL_RET_OK) return false; }
#define EXEC_EVERY(MS, X)  do { static volatile int64_t t=-1; \
  if (t==-1) t=uxr_millis(); if ((int32_t)(uxr_millis()-t) > (MS)) { X; t=uxr_millis(); } } while (0)

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

// ---- /cmd_vel -> differential mixing ----
void on_cmd(const void* msgin) {
  const geometry_msgs__msg__Twist* m = (const geometry_msgs__msg__Twist*)msgin;
  float v = m->linear.x, w = m->angular.z;
  float left  = (v - w * WHEEL_SEP / 2.0f) / MAX_LIN;
  float right = (v + w * WHEEL_SEP / 2.0f) / MAX_LIN;
  drive_side(L_EN, L_IN1, L_IN2, left);
  drive_side(R_EN, R_IN1, R_IN2, right);
}

void fill_range(sensor_msgs__msg__Range* r, const char* frame) {
  r->radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
  r->field_of_view = 0.26f;
  r->min_range = 0.04f;
  r->max_range = (float)US_MAX_M;
  r->header.frame_id = micro_ros_string_utilities_set(r->header.frame_id, frame);
}

// ---- timer: read the 3 ultrasonics and publish ----
void on_timer(rcl_timer_t*, int64_t) {
  range_front.range = read_ultrasonic(US_FRONT);
  range_left.range  = read_ultrasonic(US_LEFT);
  range_right.range = read_ultrasonic(US_RIGHT);
  rcl_publish(&pub_front, &range_front, NULL);
  rcl_publish(&pub_left,  &range_left,  NULL);
  rcl_publish(&pub_right, &range_right, NULL);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "microbot_esp32", "", &support));

  RCCHECK(rclc_publisher_init_default(&pub_front, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "/ultrasonic/front"));
  RCCHECK(rclc_publisher_init_default(&pub_left, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "/ultrasonic/left"));
  RCCHECK(rclc_publisher_init_default(&pub_right, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "/ultrasonic/right"));
  RCCHECK(rclc_subscription_init_default(&sub_cmd, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), on_timer));
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &cmd_msg, &on_cmd, ON_NEW_DATA));

  fill_range(&range_front, "us_front");
  fill_range(&range_left,  "us_left");
  fill_range(&range_right, "us_right");
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

  // === CHECKPOINT: serial_diag ===
  // Boot/WiFi/Agent diagnostics on the USB serial console (115200). Comment out this
  // block to run fully headless. `pio device monitor` shows the IP + connection state.
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.printf("[microbot] boot. connecting to WiFi SSID='%s' ...\n", WIFI_SSID);

  // Manual, non-blocking WiFi join with status logging (don't hang forever like the
  // default set_microros_wifi_transports loop). status codes: 0=IDLE 1=NO_SSID_AVAIL
  // 3=CONNECTED 4=CONNECT_FAILED 6=DISCONNECTED. NO_SSID on a classic ESP32 usually
  // means the SSID is 5 GHz only (ESP32 is 2.4 GHz). Scan results are printed too.
  WiFi.mode(WIFI_STA);
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
      EXEC_EVERY(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 3))
                              ? AGENT_CONNECTED : AGENT_DISCONNECTED);
      if (state == AGENT_CONNECTED)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
      break;
    case AGENT_DISCONNECTED:
      drive_side(L_EN, L_IN1, L_IN2, 0);     // safety: stop on link loss
      drive_side(R_EN, R_IN1, R_IN2, 0);
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}
