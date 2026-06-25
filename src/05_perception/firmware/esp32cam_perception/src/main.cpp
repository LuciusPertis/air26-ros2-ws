// AIR26 Workshop 05 — ESP32-CAM perception firmware.
//
// The real-hardware twin of the Webots camera. It publishes the SAME two "cheap" topics the
// sim's camera_processor does, so the perceptbot behaviours (light-seek B4, colour-seek B5)
// run against the real board unchanged:
//
//   micro-ROS (WiFi/UDP):  /camera/mean_intensity  (std_msgs/Float32)   0..1
//                          /camera/mean_color      (std_msgs/ColorRGBA) r,g,b 0..1, a=1
//   WiFi MJPEG HTTP:       http://<board-ip>/stream   (the full image — far too big to push
//                          through micro-ROS/UDP, so it goes over plain HTTP, like every
//                          real ESP32-CAM project. ArUco (B6) runs on a PC subscribing to
//                          this stream; the board stays cheap.)
//
// Flash over USB serial; runs over WiFi. Default pinout = AI-Thinker ESP32-CAM (OV2640).
// >>> EDIT THE CONFIG BLOCK for your WiFi, the Agent's IP, and (if needed) the colour byte
// order. <<<

#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/color_rgba.h>

// ============================ CONFIG — EDIT ME ============================
static char     WIFI_SSID[] = "LSPrmn60x";
static char     WIFI_PASS[] = "pi=3.14159";
static uint8_t  AGENT_IP[4] = {10, 65, 205, 251};    // PC running micro_ros_agent
static uint16_t AGENT_PORT  = 8888;

static const float PUBLISH_HZ = 5.0f;     // rate for the two micro-ROS topics
// RGB565 byte order varies by board; if mean_color has red/blue swapped, flip this.
static const bool  SWAP_RGB565 = false;
// ========================================================================

// --- AI-Thinker ESP32-CAM pin map (OV2640) ---
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ---- micro-ROS handles ----
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t pub_intensity, pub_color;
rcl_timer_t timer;
std_msgs__msg__Float32 msg_intensity;
std_msgs__msg__ColorRGBA msg_color;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;

#define RCCHECK(fn) { if ((fn) != RCL_RET_OK) return false; }
#define EXEC_EVERY(MS, X) do { static volatile int64_t t=-1; \
  if (t==-1) t=uxr_millis(); if ((int32_t)(uxr_millis()-t) > (MS)) { X; t=uxr_millis(); } } while (0)

httpd_handle_t http_server = NULL;

// ---- camera ----
bool init_camera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM; c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.frame_size = FRAMESIZE_QVGA;       // 320x240
  c.pixel_format = PIXFORMAT_RGB565;   // RGB so we can average colour; jpeg made on demand
  c.fb_count = 2;                      // 2 buffers: stream + stats can both grab
  c.grab_mode = CAMERA_GRAB_LATEST;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) { Serial.printf("[cam] init FAILED 0x%04x\n", err); return false; }
  Serial.println("[cam] init OK (RGB565 QVGA)");
  return true;
}

// reduce one RGB565 frame -> mean intensity (0..1) + mean r,g,b (0..1)
void frame_stats(camera_fb_t* fb, float& inten, float& mr, float& mg, float& mb) {
  uint32_t sr = 0, sg = 0, sb = 0, n = 0;
  for (size_t i = 0; i + 1 < fb->len; i += 2 * 4) {     // every 4th pixel is plenty
    uint16_t px = SWAP_RGB565 ? (fb->buf[i] | (fb->buf[i + 1] << 8))
                              : ((fb->buf[i] << 8) | fb->buf[i + 1]);
    sr += ((px >> 11) & 0x1f) << 3;   // 5-bit R -> 0..255
    sg += ((px >> 5) & 0x3f) << 2;    // 6-bit G -> 0..255
    sb += (px & 0x1f) << 3;           // 5-bit B -> 0..255
    n++;
  }
  if (!n) { inten = mr = mg = mb = 0; return; }
  mr = (sr / float(n)) / 255.0f; mg = (sg / float(n)) / 255.0f; mb = (sb / float(n)) / 255.0f;
  inten = 0.299f * mr + 0.587f * mg + 0.114f * mb;   // luma
}

// ---- micro-ROS timer: capture, compute, publish ----
void on_timer(rcl_timer_t*, int64_t) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;
  float inten, mr, mg, mb;
  frame_stats(fb, inten, mr, mg, mb);
  esp_camera_fb_return(fb);
  msg_intensity.data = inten;
  rcl_publish(&pub_intensity, &msg_intensity, NULL);
  msg_color.r = mr; msg_color.g = mg; msg_color.b = mb; msg_color.a = 1.0f;
  rcl_publish(&pub_color, &msg_color, NULL);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32cam_perception", "", &support));
  RCCHECK(rclc_publisher_init_default(&pub_intensity, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/camera/mean_intensity"));
  RCCHECK(rclc_publisher_init_default(&pub_color, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA), "/camera/mean_color"));
  const unsigned int period = (unsigned int)(1000.0f / PUBLISH_HZ);
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(period), on_timer));
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  return true;
}

void destroy_entities() {
  rmw_context_t* rmw = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw, 0);
  rcl_publisher_fini(&pub_intensity, &node);
  rcl_publisher_fini(&pub_color, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ---- MJPEG stream over HTTP (full image) ----
static const char* STREAM_CT = "multipart/x-mixed-replace;boundary=frame";
esp_err_t stream_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, STREAM_CT);
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;
    uint8_t* jpg = NULL; size_t jpg_len = 0;
    bool ok = frame2jpg(fb, 80, &jpg, &jpg_len);   // RGB565 -> JPEG
    esp_camera_fb_return(fb);
    if (!ok) return ESP_FAIL;
    char hdr[64];
    int n = snprintf(hdr, sizeof(hdr),
                     "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                     (unsigned)jpg_len);
    esp_err_t e = httpd_resp_send_chunk(req, hdr, n);
    if (e == ESP_OK) e = httpd_resp_send_chunk(req, (const char*)jpg, jpg_len);
    free(jpg);
    if (e != ESP_OK) break;       // client disconnected
  }
  return ESP_OK;
}

void start_http() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  if (httpd_start(&http_server, &config) == ESP_OK) {
    httpd_uri_t stream = {"/stream", HTTP_GET, stream_handler, NULL};
    httpd_register_uri_handler(http_server, &stream);
    httpd_uri_t root = {"/", HTTP_GET,
      [](httpd_req_t* r) {
        const char* html = "<html><body style='margin:0'>"
          "<img src='/stream' style='width:100%'></body></html>";
        httpd_resp_send(r, html, HTTPD_RESP_USE_STRLEN); return ESP_OK; }, NULL};
    httpd_register_uri_handler(http_server, &root);
    Serial.printf("[http] MJPEG stream at http://%s/stream\n",
                  WiFi.localIP().toString().c_str());
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n========== ESP32-CAM PERCEPTION ==========");
  if (psramFound()) Serial.printf("[cam] PSRAM %u KB\n", ESP.getPsramSize() / 1024);
  if (!init_camera()) { Serial.println(">>> camera init failed; halting."); return; }

  // non-blocking WiFi join with status logging (same pattern as the microbot firmware)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[wifi] joining '%s' ...\n", WIFI_SSID);
  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
    delay(1000);
    Serial.printf("[wifi] status=%d\n", (int)WiFi.status());
  }
  Serial.printf("[wifi] status=%d IP=%s\n", (int)WiFi.status(),
                WiFi.localIP().toString().c_str());

  start_http();   // image stream is independent of the micro-ROS link
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
  Serial.printf("[uros] agent %d.%d.%d.%d:%u\n",
                AGENT_IP[0], AGENT_IP[1], AGENT_IP[2], AGENT_IP[3], AGENT_PORT);
}

void loop() {
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
