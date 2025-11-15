#include <WiFi.h>
#include <esp_camera.h>
#include <WebServer.h>
#include <HTTPClient.h>

// ===================== User Config =====================
// Wi-Fi
const char* ssid     = "Phattrarawat9916";
const char* password = "pooh9916";

// Flask server endpoint (รับรูป snapshot ไปอ่าน QR)
const char* flaskUploadUrl = "http://10.36.211.109:5000/upload"; // <-- เปลี่ยนให้ตรงเครือข่าย

// ===================== Camera Pins (AI Thinker) =====================
#define CAMERA_MODEL_AI_THINKER
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

// Flash LED (on-board) for AI-Thinker
#define FLASH_GPIO_NUM     4
#define FLASH_LEDC_CH      LEDC_CHANNEL_1

// ===================== Globals =====================
WebServer server(80);

// Forward declarations
void startCameraServer();
void handleStream();
void handleJPG();
void handleUpload();
void handleCamConfig();
void handleFlash();
void handleRoot();
void handlePing();

// ===================== Helpers =====================
static framesize_t strToFrameSize(const String& s) {
  if (s.equalsIgnoreCase("QQVGA")) return FRAMESIZE_QQVGA;   // 160x120
  if (s.equalsIgnoreCase("HQVGA")) return FRAMESIZE_HQVGA;   // 240x176
  if (s.equalsIgnoreCase("QVGA"))  return FRAMESIZE_QVGA;    // 320x240
  if (s.equalsIgnoreCase("CIF"))   return FRAMESIZE_CIF;     // 352x288
  if (s.equalsIgnoreCase("VGA"))   return FRAMESIZE_VGA;     // 640x480
  if (s.equalsIgnoreCase("SVGA"))  return FRAMESIZE_SVGA;    // 800x600
  if (s.equalsIgnoreCase("XGA"))   return FRAMESIZE_XGA;     // 1024x768
  if (s.equalsIgnoreCase("SXGA"))  return FRAMESIZE_SXGA;    // 1280x1024
  if (s.equalsIgnoreCase("UXGA"))  return FRAMESIZE_UXGA;    // 1600x1200
  return FRAMESIZE_INVALID;
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  // --- Camera config ---
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;  // camera uses channel 0 internally
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Default resolution/quality
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_QVGA;  // 320x240 (ลื่น/พอสำหรับ decode)
    config.jpeg_quality = 10;              // 10 = คมขึ้น ไฟล์ใหญ่ขึ้น
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_QQVGA; // 160x120
    config.jpeg_quality = 12;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    delay(3000);
    ESP.restart();
  }

  // Optional: initial sensor tuning
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, config.frame_size);
    s->set_vflip(s, 0);    // 0 or 1 ตามการติดตั้งจริง
    s->set_hmirror(s, 0);  // 0 or 1
    s->set_brightness(s, 0);
    s->set_saturation(s, 0);
    s->set_contrast(s, 0);
    s->set_whitebal(s, 1);
    s->set_aec2(s, 1);
    s->set_awb_gain(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
  }

  // Flash LED PWM (dimmable)
  ledcSetup(FLASH_LEDC_CH, 5000 /*Hz*/, 8 /*bits*/);
  ledcAttachPin(FLASH_GPIO_NUM, FLASH_LEDC_CH);
  ledcWrite(FLASH_LEDC_CH, 0); // off

  // --- Wi-Fi ---
  WiFi.setSleep(false); // ลด latency/คงการเชื่อมต่อให้เสถียร
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("✅ WiFi connected");
  Serial.print("ESP32-CAM IP: ");
  Serial.println(WiFi.localIP());

  startCameraServer();
}

// ===================== HTTP Handlers =====================
void handleRoot() {
  String html;
  html += F("<html><head><meta charset='utf-8'><title>ESP32-CAM</title></head><body>");
  html += F("<h2>ESP32-CAM (AI Thinker)</h2>");
  html += F("<p><a href='/stream' target='_blank'>/stream</a> — MJPEG live stream</p>");
  html += F("<p><a href='/jpg' target='_blank'>/jpg</a> — one snapshot (JPEG)</p>");
  html += F("<p><a href='/upload'>/upload</a> — capture and POST to Flask</p>");
  html += F("<p><a href='/cam?framesize=VGA'>/cam?framesize=VGA</a> — change resolution</p>");
  html += F("<p><a href='/flash?duty=128'>/flash?duty=128</a> — set flash duty (0-255)</p>");
  html += F("<p><a href='/ping'>/ping</a> — PONG</p>");
  html += F("<hr><pre>");
  html += F("IP: "); html += WiFi.localIP().toString(); html += '\n';
  html += F("Upload URL: "); html += flaskUploadUrl; html += '\n';
  html += F("</pre></body></html>");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/html", html);
}

void handlePing() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "PONG");
}

// MJPEG Streaming (multipart/x-mixed-replace)
void handleStream() {
  WiFiClient client = server.client();
  client.print(
    "HTTP/1.1 200 OK\r\n"
    "Cache-Control: no-cache\r\n"
    "Pragma: no-cache\r\n"
    "Connection: close\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n"
  );

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[STREAM] Camera capture failed");
      delay(10);
      continue;
    }

    client.print("--frame\r\n");
    client.print("Content-Type: image/jpeg\r\n");
    client.printf("Content-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    esp_camera_fb_return(fb);

    delay(30); // ~33 FPS (ขึ้นกับ network/CPU)
    if (!client.connected()) break;
    yield();
  }
}

// Return one JPEG frame to browser
void handleJPG() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Disposition", "inline; filename=snapshot.jpg");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// Capture & POST to Flask (octet-stream)
void handleUpload() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  uint32_t t0 = millis();
  HTTPClient http;
  http.setTimeout(5000);
  http.begin(flaskUploadUrl);
  http.addHeader("Content-Type", "application/octet-stream");
  http.addHeader("User-Agent", "ESP32-CAM");
  int httpCode = http.POST(fb->buf, fb->len);
  uint32_t dt = millis() - t0;
  http.end();

  esp_camera_fb_return(fb);

  Serial.printf("[UPLOAD] code=%d, bytes=%u, dt=%lums\n", httpCode, fb->len, (unsigned long)dt);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", String("Uploaded: ") + httpCode + ", dt(ms)=" + dt);
}

// Adjust camera params via query string
// /cam?framesize=VGA&quality=10&vflip=0&hmirror=1&brightness=0&contrast=0&saturation=0
void handleCamConfig() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    server.send(500, "text/plain", "No sensor");
    return;
  }

  String msg; msg.reserve(256); msg += "OK\n";

  if (server.hasArg("framesize")) {
    framesize_t fs = strToFrameSize(server.arg("framesize"));
    if (fs != FRAMESIZE_INVALID) { s->set_framesize(s, fs); msg += "framesize=" + server.arg("framesize") + "\n"; }
  }
  if (server.hasArg("quality"))    { s->set_quality(s, server.arg("quality").toInt());       msg += "quality="    + server.arg("quality")    + "\n"; }
  if (server.hasArg("vflip"))      { s->set_vflip(s, server.arg("vflip").toInt());           msg += "vflip="      + server.arg("vflip")      + "\n"; }
  if (server.hasArg("hmirror"))    { s->set_hmirror(s, server.arg("hmirror").toInt());       msg += "hmirror="    + server.arg("hmirror")    + "\n"; }
  if (server.hasArg("brightness")) { s->set_brightness(s, server.arg("brightness").toInt()); msg += "brightness=" + server.arg("brightness") + "\n"; }
  if (server.hasArg("contrast"))   { s->set_contrast(s, server.arg("contrast").toInt());     msg += "contrast="   + server.arg("contrast")   + "\n"; }
  if (server.hasArg("saturation")) { s->set_saturation(s, server.arg("saturation").toInt()); msg += "saturation=" + server.arg("saturation") + "\n"; }
  if (server.hasArg("awb"))        { s->set_whitebal(s, server.arg("awb").toInt());          msg += "awb="        + server.arg("awb")        + "\n"; }
  if (server.hasArg("aec"))        { s->set_exposure_ctrl(s, server.arg("aec").toInt());     msg += "aec="        + server.arg("aec")        + "\n"; }
  if (server.hasArg("agc"))        { s->set_gain_ctrl(s, server.arg("agc").toInt());         msg += "agc="        + server.arg("agc")        + "\n"; }

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", msg);
}

// Control onboard flash LED brightness: /flash?duty=0..255
void handleFlash() {
  int duty = server.hasArg("duty") ? constrain(server.arg("duty").toInt(), 0, 255) : 0;
  ledcWrite(FLASH_LEDC_CH, duty);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", String("flash duty=") + duty);
}

// ===================== Server =====================
void startCameraServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/ping", HTTP_GET, handlePing);
  server.on("/stream", HTTP_GET, handleStream);
  server.on("/jpg", HTTP_GET, handleJPG);
  server.on("/upload", HTTP_GET, handleUpload);   // Trigger snapshot + upload to Flask
  server.on("/cam", HTTP_GET, handleCamConfig);   // Adjust camera params
  server.on("/flash", HTTP_GET, handleFlash);     // Flash LED PWM control

  server.onNotFound([](){
    String m = "404 Not Found\n";
    m += "Try: /, /stream, /jpg, /upload, /cam, /flash, /ping\n";
    server.send(404, "text/plain", m);
  });

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
