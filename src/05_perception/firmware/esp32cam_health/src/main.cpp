// AIR26 Workshop 05 — ESP32-CAM health check (standalone, no micro-ROS).
//
// A "does this board actually work?" bring-up test for ESP32-CAM modules. Over the serial
// console (115200) it reports:
//   - chip + PSRAM status
//   - camera init result (+ error code on failure)
//   - sensor model / PID
//   - a live capture loop: frame W x H, byte size, mean/min/max pixel brightness, fps
//
// The brightness numbers are the real proof: cover the lens -> mean drops toward 0; point at
// a light -> mean climbs toward 255. If mean reacts, the sensor is producing genuine image
// data. The onboard red LED (GPIO33, active-low) blinks once per frame as a liveness beat.
//
// Default pinout = AI-Thinker ESP32-CAM (the common OV2640 board). For other modules
// (M5Camera, ESP-EYE, TTGO T-Camera) swap the CAMERA_MODEL pin block below.

#include <Arduino.h>
#include "esp_camera.h"

// ============================ CONFIG — EDIT ME ============================
// Capture format for the health test. GRAYSCALE makes brightness analysis trivial (1 byte =
// 1 luma sample) and is the most reliable "is the sensor alive" signal. QVGA = 320x240.
#define HEALTH_FRAMESIZE   FRAMESIZE_QVGA
#define HEALTH_PIXFORMAT   PIXFORMAT_GRAYSCALE
#define REPORT_EVERY_MS    1000*5          // how often to print the capture stats

// --- AI-Thinker ESP32-CAM pin map (OV2640) ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define RED_LED_GPIO      33             // onboard status LED (active LOW)
#define FLASH_LED_GPIO     4             // bright white flash LED (active HIGH)
// ========================================================================

static bool camera_ok = false;

static const char* sensor_name(int pid) {
  switch (pid) {
    case 0x26: return "OV2640";
    case 0x36: return "OV3660";
    case 0x56: return "OV5640";
    case 0x77: return "OV7725";
    case 0x73: return "OV7670";
    default:   return "UNKNOWN";
  }
}

bool init_camera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size   = HEALTH_FRAMESIZE;
  config.pixel_format = HEALTH_PIXFORMAT;
  config.fb_count     = 1;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[esp32cam_health] camera init: FAILED (err 0x%04x)\n", err);
    Serial.println("[esp32cam_health]   -> check ribbon seating, 5V supply, and that this is "
                   "an AI-Thinker board (else edit the pin map).");
    return false;
  }
  Serial.println("[esp32cam_health] camera init: OK");
  sensor_t* s = esp_camera_sensor_get();
  if (s) Serial.printf("[esp32cam_health] sensor: %s (PID 0x%02x)\n",
                       sensor_name(s->id.PID), s->id.PID);
  return true;
}

void setup() {
  pinMode(RED_LED_GPIO, OUTPUT);   digitalWrite(RED_LED_GPIO, HIGH);   // off (active low)
  pinMode(FLASH_LED_GPIO, OUTPUT); digitalWrite(FLASH_LED_GPIO, LOW);  // off

  Serial.begin(115200);
  delay(400);
  Serial.println();
  Serial.println("========== ESP32-CAM HEALTH CHECK ==========");
  Serial.printf("[esp32cam_health] chip: %s rev%d, %d core(s), %d MHz\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(),
                ESP.getCpuFreqMHz());
  Serial.printf("[esp32cam_health] flash: %u KB\n", ESP.getFlashChipSize() / 1024);

  // === CHECKPOINT: psram_check ===
  if (psramFound())
    Serial.printf("[esp32cam_health] PSRAM: FOUND (%u KB)\n", ESP.getPsramSize() / 1024);
  else
    Serial.println("[esp32cam_health] PSRAM: NOT FOUND  (camera limited to small frames; "
                   "many fake/clone boards omit PSRAM)");
  // === END CHECKPOINT: psram_check ===

  camera_ok = init_camera();

  // === CHECKPOINT: flash_led_test ===
  // Pulse the bright white LED once so you can confirm GPIO4 + the LED work. Comment out if
  // it's blinding on the bench.
  if (camera_ok) {
    digitalWrite(FLASH_LED_GPIO, HIGH); delay(120); digitalWrite(FLASH_LED_GPIO, LOW);
  }
  // === END CHECKPOINT: flash_led_test ===

  if (!camera_ok)
    Serial.println("[esp32cam_health] >>> HEALTH: FAIL — camera did not initialise.");
  else
    Serial.println("[esp32cam_health] capturing... cover/uncover the lens and watch 'mean'.");
}

void loop() {
  //delay(1000); return;
  if (!camera_ok) { delay(1000); return; }

  static uint32_t frames = 0, win_start = 0, last_report = 0;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[esp32cam_health] capture: FAILED to grab frame buffer");
    delay(200);
    return;
  }
  frames++;

  // liveness beat: toggle red LED each frame
  digitalWrite(RED_LED_GPIO, frames & 1);
  // delay(2000);  // 2s delay to slow down the output for easier reading

  uint32_t now = millis();
  if (win_start == 0) { win_start = now; last_report = now; }

  if (now - last_report >= REPORT_EVERY_MS) {
    // brightness stats over the grayscale buffer (sub-sampled for speed)
    uint32_t sum = 0, count = 0; uint8_t mn = 255, mx = 0;
    for (size_t i = 0; i < fb->len; i += 4) {       // every 4th byte is plenty
      uint8_t p = fb->buf[i];
      sum += p; count++;
      if (p < mn) mn = p;
      if (p > mx) mx = p;
    }
    uint32_t mean = count ? sum / count : 0;
    float fps = frames * 1000.0f / (now - win_start);
    Serial.printf("[esp32cam_health] %ux%u  %u bytes  mean=%u min=%u max=%u  %.1f fps\n",
                  fb->width, fb->height, (unsigned)fb->len, mean, mn, mx, fps);
    last_report = now; win_start = now; frames = 0;
  }

  esp_camera_fb_return(fb);
}
