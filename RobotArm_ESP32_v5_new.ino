// ============================================================
// AI-Based Robotic Arm Vision System — ESP32-CAM Firmware v5
// Hardware: AI-Thinker ESP32-CAM (OV3660), PCA9685, 4x Servos
// SDA=GPIO14  SCL=GPIO15
// ============================================================

#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ========== Soft AP Credentials ==========  ← edit these two lines
const char* AP_SSID = "RobotArm";    // WiFi name your PC will see
const char* AP_PASS = "robot1234";   // min 8 chars (WPA2), or "" for open

// AP network — fixed subnet, change only if 192.168.4.x conflicts
const IPAddress AP_IP   (192, 168, 4, 1);
const IPAddress AP_GW   (192, 168, 4, 1);
const IPAddress AP_MASK (255, 255, 255, 0);

// ========== I2C Pins ==========
#define SDA_PIN 14
#define SCL_PIN 15

// ========== PCA9685 ==========
#define PCA9685_ADDR 0x40
#define SERVO_FREQ   50
#define SERVO_MIN    150   // ~0°
#define SERVO_MAX    600   // ~180°

#define CH_BASE     0
#define CH_SHOULDER 1
#define CH_ELBOW    2
#define CH_GRIPPER  3

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_ADDR);

#define FLASH_LED_PIN 4
#define LED_BRGHTNESS 50

// ========== Servo State ==========
struct ServoAxis {
  int minDeg;
  int maxDeg;
  int current;   // actual position tracked by smooth task
  int target;    // desired position
};

ServoAxis axis[4] = {
  {  0, 180, 90, 90 },   // Base      0-180
  { 45, 135, 90, 90 },   // Shoulder  45-135
  { 10, 120, 45, 45 },   // Elbow     10-120, home 45°
  {  0,  40, 30, 30 }    // Gripper   0-40
};

// ── Configurable home position (degrees) ──────────────────────
// Stored in RAM; updated at runtime via POST /set_home.
// All four values must be within each axis's min/max limits.
int homePos[4] = { 90, 90, 45, 30 };   // Base, Shoulder, Elbow, Gripper

// ── Smooth-move background task ───────────────────────────────
void smoothServoTask(void* pvParam) {
  while (true) {
    for (int i = 0; i < 4; i++) {
      if (axis[i].current != axis[i].target) {
        if (axis[i].current < axis[i].target) axis[i].current++;
        else                                   axis[i].current--;
        int pulse = map(axis[i].current, 0, 180, SERVO_MIN, SERVO_MAX);
        pca.setPWM(i, 0, pulse);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // 50 steps/sec → smooth sweep
  }
}

void setTarget(int idx, int deg) {
  deg = constrain(deg, axis[idx].minDeg, axis[idx].maxDeg);
  axis[idx].target = deg;
  Serial.printf("[SERVO] ch%d -> %d°\n", idx, deg);
}

// Move all axes to the current homePos[] values
void homeAll() {
  for (int i = 0; i < 4; i++) setTarget(i, homePos[i]);
}

// ========== Camera Pins (AI-Thinker ESP32-CAM) ==========
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

bool initCamera() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM; cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM; cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM; cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM; cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM;
  cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 10000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = FRAMESIZE_VGA;
  cfg.jpeg_quality = 10;
  cfg.fb_count     = 3;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init failed: 0x%x\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_quality(s, 12);
    s->set_brightness(s, 1);
    s->set_saturation(s, 0);
  }
  Serial.println("[CAM] OK");
  return true;
}

// ========== HTTP Server ==========
WebServer server(80);

// Replace the entire handleMjpeg() function with this non-blocking version:

void handleMjpeg() {
  WiFiClient client = server.client();
  
  // Send HTTP headers
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: keep-alive");
  client.println();
  
  unsigned long lastFrame = 0;
  const unsigned long frameInterval = 150; // ~15 fps
  
  // Turn on LED flash briefly for better illumination
  ledcWrite(FLASH_LED_PIN, LED_BRGHTNESS);

  // Non-blocking streaming loop
  while (client.connected()) {
    unsigned long now = millis();
    if (now - lastFrame >= frameInterval) {      
      
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb) {
        client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
        client.write(fb->buf, fb->len);
        client.print("\r\n");
        esp_camera_fb_return(fb);
        lastFrame = now;
      }
      
    }
    
    // Yield to allow other requests to be processed
    delay(5);
    
    // Handle any pending HTTP requests
    server.handleClient();
  }

  ledcWrite(FLASH_LED_PIN, 0);

  // Clean up on disconnect
  client.stop();
  
}



void handleSnapshot() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "Camera error"); return; }
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// POST /move  — {"base":90,"shoulder":90,"elbow":45,"gripper":30}
void handleMove() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"no body\"}"); return;
  }
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "application/json", "{\"error\":\"json parse\"}"); return;
  }
  if (doc.containsKey("base"))     setTarget(CH_BASE,     doc["base"]);
  if (doc.containsKey("shoulder")) setTarget(CH_SHOULDER, doc["shoulder"]);
  if (doc.containsKey("elbow"))    setTarget(CH_ELBOW,    doc["elbow"]);
  if (doc.containsKey("gripper"))  setTarget(CH_GRIPPER,  doc["gripper"]);
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// POST /home  — move all axes to homePos[]
void handleHome() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  homeAll();
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// POST /set_home  — {"base":N,"shoulder":N,"elbow":N,"gripper":N}
// Updates homePos[] without moving the arm. Values are clamped
// to each axis's hard limits. Returns the accepted home angles.
void handleSetHome() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"no body\"}"); return;
  }
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "application/json", "{\"error\":\"json parse\"}"); return;
  }
  const char* keys[] = {"base","shoulder","elbow","gripper"};
  for (int i = 0; i < 4; i++) {
    if (doc.containsKey(keys[i])) {
      int v = constrain((int)doc[keys[i]], axis[i].minDeg, axis[i].maxDeg);
      homePos[i] = v;
      Serial.printf("[HOME] ch%d home set to %d°\n", i, v);
    }
  }
  // Respond with the accepted values
  StaticJsonDocument<256> resp;
  resp["status"]   = "ok";
  resp["base"]     = homePos[0];
  resp["shoulder"] = homePos[1];
  resp["elbow"]    = homePos[2];
  resp["gripper"]  = homePos[3];
  String out; serializeJson(resp, out);
  server.send(200, "application/json", out);
}

// GET /status  — current, target, and home per axis
void handleStatus() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  StaticJsonDocument<400> doc;
  const char* names[] = {"base","shoulder","elbow","gripper"};
  for (int i = 0; i < 4; i++) {
    doc[names[i]]["current"] = axis[i].current;
    doc[names[i]]["target"]  = axis[i].target;
    doc[names[i]]["home"]    = homePos[i];
    doc[names[i]]["min"]     = axis[i].minDeg;
    doc[names[i]]["max"]     = axis[i].maxDeg;
  }
  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(204);
}

// ========== Setup & Loop ==========
void setup() {
  Serial.begin(115200);
  delay(1000);

  ledcAttach(FLASH_LED_PIN, 5000, 8);
  ledcWrite(FLASH_LED_PIN, 0);

  Wire.begin(SDA_PIN, SCL_PIN);
  pca.begin();
  delay(1000);
  pca.setPWMFreq(SERVO_FREQ);

  // Initialize servos to home positions
  for (int i = 0; i < 4; i++) {
    axis[i].current = homePos[i];
    axis[i].target  = homePos[i];
    int pulse = map(homePos[i], 0, 180, SERVO_MIN, SERVO_MAX);
    pca.setPWM(i, 0, pulse);
  }
  delay(500);
  Serial.println("[SERVO] Initialized to home positions");

  if (!initCamera()) Serial.println("[WARN] Continuing without camera");

  // Smooth-move task on Core 0 (HTTP server runs on Core 1)
  xTaskCreatePinnedToCore(smoothServoTask, "servoTask", 2048, NULL, 1, NULL, 0);

  // ── Soft AP initialisation ────────────────────────────────
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
  bool apOk = (strlen(AP_PASS) >= 8)
                ? WiFi.softAP(AP_SSID, AP_PASS)
                : WiFi.softAP(AP_SSID);
  if (apOk) {
    Serial.printf("[WIFI] AP ready → SSID: %s  IP: %s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());
    Serial.printf("[WIFI] Connect PC to WiFi '%s' then open http://%s:80\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("[WIFI] AP failed to start — check credentials");
  }

  server.on("/mjpeg",    HTTP_GET,     handleMjpeg);
  server.on("/snapshot", HTTP_GET,     handleSnapshot);
  server.on("/move",     HTTP_POST,    handleMove);
  server.on("/move",     HTTP_OPTIONS, handleCORS);
  server.on("/home",     HTTP_POST,    handleHome);
  server.on("/home",     HTTP_OPTIONS, handleCORS);
  server.on("/set_home", HTTP_POST,    handleSetHome);
  server.on("/set_home", HTTP_OPTIONS, handleCORS);
  server.on("/status",   HTTP_GET,     handleStatus);
  server.begin();
  Serial.println("[HTTP] Server ready");
}

void loop() {
  server.handleClient();
}
