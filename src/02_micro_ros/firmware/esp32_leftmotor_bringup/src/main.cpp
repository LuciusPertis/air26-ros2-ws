// Left-motor + encoder test node, exposed over micro-ROS (WiFi/UDP).
//   publishes: /left_encoder/ticks   (std_msgs/Int32)
// Drives the left motor slowly and streams the encoder tick count to the Agent.
// WiFi/Agent transport + publishing lifecycle follow the workshop firmware pattern.

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// ============================ CONFIG — EDIT ME ============================
// WiFi + Agent
static char        WIFI_SSID[]  = "";
static char        WIFI_PASS[]  = "";
static uint8_t     AGENT_IP[4]  = {0, 0, 0, 0};   // the PC running micro_ros_agent
static uint16_t    AGENT_PORT   = 8888;
// ========================================================================

// ===== LEFT MOTOR - L298N (IN1/IN2 + ENA) =====
#define L_IN1   4    // direction
#define L_IN2   5    // direction
#define L_EN    6    // ENA - PWM speed

// ===== LEFT ENCODER =====
#define L_ENC_A  17   // Channel A (interrupt)
#define L_ENC_B  18   // Channel B (direction)

// ===== SLOW SPEED (0-255 PWM duty) =====
#define SLOW_SPEED  100

// ---- micro-ROS handles ----
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t pub_ticks;
rcl_timer_t timer;
std_msgs__msg__Int32 ticks_msg;

// ---- agent connection state machine (standard micro-ROS reconnect pattern) ----
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;

#define RCCHECK(fn)    { if ((fn) != RCL_RET_OK) return false; }
#define EXEC_EVERY(MS, X)  do { static volatile int64_t t=-1; \
if (t==-1) t=uxr_millis(); if ((int32_t)(uxr_millis()-t) > (MS)) { X; t=uxr_millis(); } } while (0)

// ===== ENCODER TICK COUNTER (updated inside ISR) =====
volatile long leftEncoderTicks = 0;

void IRAM_ATTR readLeftEncoder() {
  if (digitalRead(L_ENC_B) == digitalRead(L_ENC_A))
    leftEncoderTicks++;
  else
    leftEncoderTicks--;
}

// speed > 0 : forward, speed < 0 : reverse, speed = 0 : coast
void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_EN, speed);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    analogWrite(L_EN, -speed);
  }
}

// ---- timer: publish the current encoder tick count ----
void on_timer(rcl_timer_t*, int64_t) {
  noInterrupts();
  long ticks = leftEncoderTicks;
  interrupts();

  ticks_msg.data = (int32_t)ticks;
  rcl_publish(&pub_ticks, &ticks_msg, NULL);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "left_motor_esp32", "", &support));

  RCCHECK(rclc_publisher_init_default(&pub_ticks, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/left_encoder/ticks"));

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), on_timer));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  return true;
}

void destroy_entities() {
  rmw_context_t* rmw = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw, 0);
  rcl_publisher_fini(&pub_ticks, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  // motor + encoder GPIO
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_EN,  OUTPUT);
  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), readLeftEncoder, RISING);

  // start the left motor turning forward, slowly
  setLeftMotor(SLOW_SPEED);

  // === CHECKPOINT: serial_diag ===
  // Boot/WiFi/Agent diagnostics on the USB serial console (115200).
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.printf("[leftmotor] boot. connecting to WiFi SSID='%s' ...\n", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks();
  Serial.printf("[leftmotor] scan found %d networks:\n", n);
  for (int i = 0; i < n; i++)
    Serial.printf("    '%s'  rssi=%d  ch=%d  enc=%d\n",
      WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i), WiFi.encryptionType(i));

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
    delay(1000);
    Serial.printf("[leftmotor] ...wifi status=%d\n", (int)WiFi.status());
  }
  Serial.printf("[leftmotor] WiFi status=%d  IP=%s  RSSI=%d dBm\n",
    (int)WiFi.status(), WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  Serial.printf("[leftmotor] agent target = %d.%d.%d.%d:%u\n",
    AGENT_IP[0], AGENT_IP[1], AGENT_IP[2], AGENT_IP[3], AGENT_PORT);
  // === END CHECKPOINT: serial_diag ===

  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
}

void loop() {
  // === CHECKPOINT: serial_diag ===
  static AgentState last_state = AGENT_DISCONNECTED;
  static const char* NM[] = {"WAITING_AGENT", "AGENT_AVAILABLE", "AGENT_CONNECTED", "AGENT_DISCONNECTED"};
  if (state != last_state) { Serial.printf("[leftmotor] state -> %s\n", NM[state]); last_state = state; }
  if (state == WAITING_AGENT) EXEC_EVERY(2000, Serial.println("[leftmotor] waiting for agent ping..."));
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
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}
