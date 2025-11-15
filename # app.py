// === IRandMotor (simplified) — Auto = motor_wi core, Manual kept, + pause_auto + Telemetry ===
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>

// ================== WiFi ==================
const char* ssid     = "Phattrarawat9916";
const char* password = "pooh9916";

// Flask (RTT/Log)
const char* serverPingUrl = "http://10.36.211.109:5000/ping";
const char* serverLogUrl  = "http://10.36.211.109:5000/log_rtt";

// Telemetry -> Flask
const char* serverTelemUrl = "http://10.36.211.109:5000/log_telemetry";
unsigned long lastTelemMs = 0;
const uint16_t TELEMETRY_PERIOD_MS = 200;   // ส่ง 5 Hz
// ==================

// ===== Wiring/Direction toggles =====
bool LEFT_DIR_INVERT  = false; // ถ้าล้อซ้ายเดินหน้ากลายเป็นถอย ให้ true
bool RIGHT_DIR_INVERT = false; // ถ้าล้อขวาเดินหน้ากลายเป็นถอย ให้ true
bool SWAP_MANUAL_TURN = false;  // ถ้ากด LEFT แล้วรถหมุนขวา/กลับกัน ให้สลับเป็น true/false

// ================== Motor Pins (MATCH motor_wi) ==================
int IN1=16, IN2=17;   // RIGHT DIR
int IN3=18, IN4=19;   // LEFT  DIR
int ENA=25, ENB=26;   // ENA=RIGHT PWM, ENB=LEFT PWM

// ================== IR Pins (digital, MATCH motor_wi) ==================
int IR1=34, IR2=35, IR3=32, IR4=33, IR5=27; // L2 L1 C R1 R2
bool IR_BLACK_IS_LOW = true;  // โมดูล IR ให้ "ดำ=LOW"

// ================== Web Server ==================
WebServer server(80);

// ================== Mode/State ==================
String mode = "manual";      // "manual" | "auto"

// ===== Manual drive =====
enum ManualDrive { MD_STOP, MD_FORWARD, MD_BACKWARD };
ManualDrive manualDrive = MD_STOP;

bool manualPulseActive = false;
int  manualPulseDir = 0;               // -1 left, +1 right
unsigned long manualPulseUntil = 0;
const uint16_t MANUAL_TURN_MS   = 140; // ระยะเวลาพัลส์เลี้ยว
const uint16_t MANUAL_PULSE_MAX = 1200;

const int      MANUAL_MAX_PWM = 90;   // จำกัดเพดาน manual ให้ไม่พุ่ง
const int      MIN_START_PWM  = 70;   // deadzone mapping เริ่มหมุนจริง
const uint16_t KICK_MS        = 60;   // kick สั้น ๆ ตอนเริ่ม pulse
const uint8_t  KICK_PWM       = 120;

int manualBaseSpeed = 70;             // /speed → ปรับสำหรับ manual เท่านั้น

// ================== LEDC (global) ==================
const int PWM_FREQ_HZ   = 1000;
const int PWM_RES_BITS  = 8;
const int PWM_CH_RIGHT  = 0; // ENA (RIGHT)
const int PWM_CH_LEFT   = 1; // ENB (LEFT)

// ================== Loop timing ==================
unsigned long lastLoopMs = 0;
const uint16_t LOOP_DT_MS = 10;   // ~100 Hz

// ================== Ping ==================
unsigned long lastPingMs = 0;
const uint16_t PING_PERIOD_MS = 5000;

// ================== Manual ramp ==================
int curLeftPWM  = 0;
int curRightPWM = 0;
int tgtLeftPWM  = 0;
int tgtRightPWM = 0;
const int RAMP_STEP = 8;
const int MANUAL_RAMP_STEP = 64;

// ================== AUTO pause (called by Flask /pause_auto) ==================
volatile bool   pauseActive  = false;
unsigned long   pauseUntilMs = 0;
String          pauseLastId  = "";
unsigned long   pauseLastTs  = 0;

// ================== motor_wi CORE (AUTO) ==================
// PID config
float Kp_mwi = 25, Ki_mwi = 0, Kd_mwi = 20;
const float TURN_SCALE_MWI = 0.60f;

// speed shaping
const int   STRAIGHT_MAX_MWI      = 90;
const int   STRAIGHT_HARD_CAP_MWI = 85;   // กดเพดานตอนตรงชัดให้ต่ำกว่าฐานนิดนึง
const int   CURVE_MIN_MWI         = 58;
const int   GLOBAL_MAX_PWM_MWI    = 92;
const float BASE_SCALE_MWI  = 1.0f;
const float SPEED_SCALE_MWI = 1.0f;

// derivative smoothing + slew
static float dLP_mwi = 0;
const  float D_ALPHA_MWI = 0.30f;
int PWM_SLEW_MWI = 11;

// reverse when lost
const int REVERSE_PWM_MWI = 80;

// states/log
static float integral_mwi = 0;
static int   lastError_mwi = 0;
int prevLeftPWM_mwi = 0, prevRightPWM_mwi = 0;

const unsigned long LOG_EVERY_MS=100;
unsigned long lastLogMs=0;
int g_error=0, g_output=0, g_baseDyn=0;
int g_S1=1, g_S2=1, g_S3=1, g_S4=1, g_S5=1;

// บอกว่า tick นี้ auto เขียน PWM โดยตรงแล้ว (กัน rampApply ทับ)
volatile bool autoDirectDriveLastTick = false;

// ====== Prototypes ======
void http_start();
void FollowLine_mwi();
void drivePID_mwi(int S1,int S2,int S3,int S4,int S5);
void writeSignedPWM_mwi(int L, int R);
void applyPWM_mwi(int leftPWM,int rightPWM);

void readIR5(int& S1,int& S2,int& S3,int& S4,int& S5);
bool isAllWhite5(int S1,int S2,int S3,int S4,int S5);

inline int clampi(int v, int lo, int hi){ return (v<lo)?lo:((v>hi)?hi:v); }
void setMotorRaw(int leftPWM, int rightPWM, bool leftFwd, bool rightFwd);
void stopAll(bool brake=true);
void rampApply();
void doPingOnce();
void handle_pause_auto();
void sendTelemetryOnce(uint32_t now_ms);

// ====== Helpers (manual) ======
int applyDeadzone(int cmd){
  if (cmd <= 0) return 0;
  int out = MIN_START_PWM + (cmd * (255 - MIN_START_PWM)) / 255;
  return clampi(out, MIN_START_PWM, 255);
}
void triggerManualTurnPulse(bool left, int ms){
  int dur = clampi(ms, 40, 2000);
  if (manualPulseActive){
    unsigned long cap = millis() + MANUAL_PULSE_MAX;
    unsigned long extended = manualPulseUntil + dur;
    manualPulseUntil = (extended > cap) ? cap : extended;
  } else {
    bool L = SWAP_MANUAL_TURN ? !left : left;   // ใช้สวิตช์สลับทิศ
    manualPulseActive = true;
    manualPulseDir = L ? -1 : +1;
    manualPulseUntil = millis() + dur;
    // pivot in place (นุ่ม ๆ)
    if (L)  setMotorRaw(manualBaseSpeed, manualBaseSpeed, false, true);
    else    setMotorRaw(manualBaseSpeed, manualBaseSpeed, true,  false);
  }
}

// ================== SETUP/LOOP ==================
void setup(){
  Serial.begin(115200);
  Serial.println("\n[BOOT] IRandMotor (simplified, Auto=motor_wi, +pause_auto +telemetry)");

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // LEDC: ENA(RIGHT)=ch0, ENB(LEFT)=ch1
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(ENA, PWM_CH_RIGHT);
  ledcSetup(PWM_CH_LEFT,  PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(ENB, PWM_CH_LEFT);

  // IR pins (27 ใช้ PULLDOWN ตาม motor_wi)
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT_PULLDOWN); // 27

  // WiFi
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);
  Serial.print("[WiFi] Connecting");
  while (WiFi.status() != WL_CONNECTED){ delay(300); Serial.print("."); }
  Serial.printf("\n[WiFi] Connected. IP=%s\n", WiFi.localIP().toString().c_str());

  http_start();
  lastLoopMs = millis();

  Serial.println("IR1 IR2 IR3 IR4 IR5 => ms:..., PvLPWM:..., PvRPWM:..., error:..., output:..., baseDyn:...");
}

void loop(){
  server.handleClient();

  // Wi-Fi keepalive (เบา ๆ)
  static unsigned long lastWiFiCheck=0;
  if (millis() - lastWiFiCheck > 2500) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) WiFi.begin(ssid, password);
  }

  // ส่ง Telemetry นอกวง PID เพื่อไม่บล็อกการคุมมอเตอร์
  unsigned long nowGlobal = millis();
  if ((nowGlobal - lastTelemMs) >= TELEMETRY_PERIOD_MS){
    lastTelemMs = nowGlobal;
    sendTelemetryOnce(nowGlobal);
  }

  unsigned long now = millis();
  if (now - lastPingMs >= PING_PERIOD_MS){ lastPingMs = now; doPingOnce(); }

  if (now - lastLoopMs >= LOOP_DT_MS){
    lastLoopMs = now;

    // >>> ถ้ามีคำสั่งหยุดจาก Flask — คุมให้นิ่ง <<<
    if (pauseActive){
      long left = (long)pauseUntilMs - (long)millis();
      if (left > 0){
        stopAll(true);
        ledcWrite(PWM_CH_LEFT,  0);
        ledcWrite(PWM_CH_RIGHT, 0);
        autoDirectDriveLastTick = false; // กัน auto เขียนทับ
        return; // ข้าม tick นี้
      }else{
        pauseActive = false;
      }
    }

    if (mode == "auto"){
      FollowLine_mwi();
    } else {
      // Manual upkeep
      if (manualPulseActive && (long)(millis() - manualPulseUntil) >= 0){
        manualPulseActive = false;
        manualDrive = MD_STOP;
        stopAll();
      } else if (!manualPulseActive){
        if (manualDrive == MD_FORWARD)       setMotorRaw(manualBaseSpeed, manualBaseSpeed, true,  true);
        else if (manualDrive == MD_BACKWARD) setMotorRaw(manualBaseSpeed, manualBaseSpeed, false, false);
      }
    }

    rampApply(); // จะข้ามอัตโนมัติถ้า tick นี้ Auto เขียนแล้ว
  }
}

// ================== HTTP API ==================
void resp200(const String& s){ server.sendHeader("Access-Control-Allow-Origin","*"); server.send(200,"text/plain",s); }

void handle_forward(){ mode="manual"; manualPulseActive=false; manualDrive=MD_FORWARD;  setMotorRaw(manualBaseSpeed, manualBaseSpeed, true,  true);  resp200("forward"); }
void handle_backward(){mode="manual"; manualPulseActive=false; manualDrive=MD_BACKWARD; setMotorRaw(manualBaseSpeed, manualBaseSpeed, false, false); resp200("backward");}
void handle_left(){
  mode = "manual"; manualDrive = MD_STOP;
  int ms = MANUAL_TURN_MS;
  if (server.hasArg("ms")) ms = clampi(server.arg("ms").toInt(), 40, 2000);
  triggerManualTurnPulse(true, ms);
  resp200(String("left_pulse ")+ms+" ms");
}
void handle_right(){
  mode = "manual"; manualDrive = MD_STOP;
  int ms = MANUAL_TURN_MS;
  if (server.hasArg("ms")) ms = clampi(server.arg("ms").toInt(), 40, 2000);
  triggerManualTurnPulse(false, ms);
  resp200(String("right_pulse ")+ms+" ms");
}
void handle_stop(){ mode="manual"; manualPulseActive=false; manualDrive=MD_STOP; stopAll(); resp200("stopped"); }

void handle_speed(){ // ปรับเฉพาะ manual
  if (server.hasArg("value")) manualBaseSpeed = clampi(server.arg("value").toInt(), 0, 255);
  resp200(String("manual_speed=")+manualBaseSpeed);
}
void handle_mode(){
  if (server.hasArg("value")) mode = server.arg("value");
  resp200(String("mode=")+mode);
}
void handle_set_pid(){ // (ยังคง endpoint เผื่ออนาคต แต่ UI ฝั่ง Flask ตัดออกแล้ว)
  if (server.hasArg("kp")) Kp_mwi = server.arg("kp").toFloat();
  if (server.hasArg("ki")) Ki_mwi = server.arg("ki").toFloat();
  if (server.hasArg("kd")) Kd_mwi = server.arg("kd").toFloat();
  resp200(String("pid_mwi=")+String(Kp_mwi,2)+","+String(Ki_mwi,2)+","+String(Kd_mwi,2));
}
void handle_status(){
  char buf[360];
  long paused_left = pauseActive ? (long)(pauseUntilMs - millis()) : 0;
  if (paused_left < 0) paused_left = 0;

  sprintf(buf,
    "{\"mode\":\"%s\",\"manual_speed\":%d,\"speed\":%d,"
    "\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f,"
    "\"ir\":[%d,%d,%d,%d,%d],"
    "\"paused_ms_left\":%ld,"
    "\"ip\":\"%s\"}",
    mode.c_str(), manualBaseSpeed, manualBaseSpeed,
    Kp_mwi, Ki_mwi, Kd_mwi,
    g_S1,g_S2,g_S3,g_S4,g_S5,
    paused_left,
    WiFi.localIP().toString().c_str());
  server.sendHeader("Access-Control-Allow-Origin","*");
  server.send(200, "application/json", buf);
}
void handle_pause_auto(){
  String id = server.hasArg("id") ? server.arg("id") : "";
  int ms = server.hasArg("ms") ? clampi(server.arg("ms").toInt(), 100, 5000) : 1000;

  pauseActive  = true;
  pauseUntilMs = millis() + ms;
  pauseLastId  = id;
  pauseLastTs  = millis();

  stopAll(true);
  prevLeftPWM_mwi = 0;
  prevRightPWM_mwi = 0;

  resp200(String("pause_auto: id=")+id+", ms="+ms);
}

void http_start(){
  server.on("/forward",  HTTP_GET, handle_forward);
  server.on("/backward", HTTP_GET, handle_backward);
  server.on("/left",     HTTP_GET, handle_left);
  server.on("/right",    HTTP_GET, handle_right);
  server.on("/stop",     HTTP_GET, handle_stop);
  server.on("/speed",    HTTP_GET, handle_speed);
  server.on("/mode",     HTTP_GET, handle_mode);
  server.on("/set_pid",  HTTP_GET, handle_set_pid); // UI ไม่ใช้แล้ว แต่คง endpoint ไว้
  server.on("/status",   HTTP_GET, handle_status);
  server.on("/pause_auto", HTTP_GET, handle_pause_auto);
  server.onNotFound([](){ server.send(404,"text/plain","404"); });
  server.begin();
  Serial.println("[HTTP] server started");
}

// ================== AUTO: motor_wi ==================
void FollowLine_mwi(){
  // อ่าน IR (ขาว=1, ดำ=0)
  int S1,S2,S3,S4,S5;
  readIR5(S1,S2,S3,S4,S5);
  g_S1=S1; g_S2=S2; g_S3=S3; g_S4=S4; g_S5=S5;

  // ดำหมด → soft stop
  if (S1==0 && S2==0 && S3==0 && S4==0 && S5==0){
    for(int sp=max(prevLeftPWM_mwi,prevRightPWM_mwi); sp>=0; sp-=20){
      applyPWM_mwi(sp,sp); delay(15);
    }
    digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
    prevLeftPWM_mwi=prevRightPWM_mwi=0;
    return;
  }

  // ขาวหมด → ถอยหาเส้น
  if (isAllWhite5(S1,S2,S3,S4,S5)){
    // ตั้งทิศถอยทั้งคู่
    bool Lf = false ^ LEFT_DIR_INVERT;
    bool Rf = false ^ RIGHT_DIR_INVERT;
    digitalWrite(IN3, Lf?HIGH:LOW);  digitalWrite(IN4, Lf?LOW:HIGH);
    digitalWrite(IN1, Rf?HIGH:LOW);  digitalWrite(IN2, Rf?LOW:HIGH);
    applyPWM_mwi(REVERSE_PWM_MWI, REVERSE_PWM_MWI);
    return;
  }

  // PID ปกติ
  drivePID_mwi(S1,S2,S3,S4,S5);
}

void drivePID_mwi(int S1,int S2,int S3,int S4,int S5){
  int position = S1*-2 + S2*-1 + S3*0 + S4*1 + S5*2;  // ขาว=1 ดำ=0
  int error = position;

  // PID (anti-windup + derivative smoothing)
  integral_mwi += error;
  integral_mwi = constrain(integral_mwi, -120, 120);
  int dRaw = error - lastError_mwi;
  dLP_mwi = D_ALPHA_MWI * dRaw + (1.0f - D_ALPHA_MWI) * dLP_mwi;
  int turn = (int)((Kp_mwi*error + Ki_mwi*integral_mwi + Kd_mwi*dLP_mwi) * TURN_SCALE_MWI);
  lastError_mwi = error;

  // base speed: ยิ่งเอียงมาก ยิ่งลดฐานลง
  float eNorm = min(abs(error), 2) / 2.0f; // 0..1
  float alpha = 1.2f;
  float w = pow(eNorm, alpha);
  int baseDyn = STRAIGHT_MAX_MWI - (int)((STRAIGHT_MAX_MWI - CURVE_MIN_MWI) * w);
  baseDyn = (int)(baseDyn * BASE_SCALE_MWI);
  baseDyn = min(baseDyn, GLOBAL_MAX_PWM_MWI);

  // ทางตรงชัด: กดเพดาน
  bool straightPattern = (S3==0) && ((S2==1)||(S4==1));
  if (straightPattern) baseDyn = min(baseDyn, STRAIGHT_HARD_CAP_MWI);

  // signed mix (ล้อในให้ถอยได้เมื่อ turn ใหญ่)
  int leftSigned  = (int)((baseDyn - turn) * SPEED_SCALE_MWI);
  int rightSigned = (int)((baseDyn + turn) * SPEED_SCALE_MWI);

  writeSignedPWM_mwi(leftSigned, rightSigned);

  // logs
  g_error=error; g_output=turn; g_baseDyn=baseDyn;
}

void writeSignedPWM_mwi(int L, int R){
  L = constrain(L, -GLOBAL_MAX_PWM_MWI, GLOBAL_MAX_PWM_MWI);
  R = constrain(R, -GLOBAL_MAX_PWM_MWI, GLOBAL_MAX_PWM_MWI);

  bool Lf = (L >= 0) ^ LEFT_DIR_INVERT;
  bool Rf = (R >= 0) ^ RIGHT_DIR_INVERT;

  digitalWrite(IN3, Lf ? HIGH : LOW);
  digitalWrite(IN4, Lf ? LOW  : HIGH);
  digitalWrite(IN1, Rf ? HIGH : LOW);
  digitalWrite(IN2, Rf ? LOW  : HIGH);

  applyPWM_mwi(abs(L), abs(R));
}

int slewStep_mwi(int prevVal,int targetVal,int maxStep){
  if(targetVal>prevVal+maxStep) return prevVal+maxStep;
  if(targetVal<prevVal-maxStep) return prevVal-maxStep;
  return targetVal;
}
void applyPWM_mwi(int leftPWM,int rightPWM){
  prevLeftPWM_mwi  = slewStep_mwi(prevLeftPWM_mwi,  leftPWM,  PWM_SLEW_MWI);
  prevRightPWM_mwi = slewStep_mwi(prevRightPWM_mwi, rightPWM, PWM_SLEW_MWI);

  ledcWrite(PWM_CH_LEFT,  prevLeftPWM_mwi);
  ledcWrite(PWM_CH_RIGHT, prevRightPWM_mwi);

  // กัน rampApply เขียนทับใน tick นี้
  autoDirectDriveLastTick = true;

  // log ทุก 100ms (Serial เท่านั้น)
  unsigned long now=millis();
  if(now-lastLogMs>=LOG_EVERY_MS){
    lastLogMs=now;
    Serial.print(g_S1); Serial.print(" ");
    Serial.print(g_S2); Serial.print(" ");
    Serial.print(g_S3); Serial.print(" ");
    Serial.print(g_S4); Serial.print(" ");
    Serial.print(g_S5); Serial.print(" => ");
    Serial.print("ms:");       Serial.print(now);
    Serial.print(", PvLPWM:"); Serial.print(prevLeftPWM_mwi);
    Serial.print(", PvRPWM:"); Serial.print(prevRightPWM_mwi);
    Serial.print(", error:");  Serial.print(g_error);
    Serial.print(", output:"); Serial.print(g_output);
    Serial.print(", baseDyn:");Serial.println(g_baseDyn);
  }
}

// ================== IR ==================
void readIR5(int& S1,int& S2,int& S3,int& S4,int& S5){
  int v1=digitalRead(IR1), v2=digitalRead(IR2), v3=digitalRead(IR3), v4=digitalRead(IR4), v5=digitalRead(IR5);
  auto norm = [&](int v)->int{
    if (IR_BLACK_IS_LOW) return (v==LOW)?0:1;
    else                 return (v==HIGH)?0:1;
  };
  S1=norm(v1); S2=norm(v2); S3=norm(v3); S4=norm(v4); S5=norm(v5);
}
bool isAllWhite5(int S1,int S2,int S3,int S4,int S5){
  return (S1==1 && S2==1 && S3==1 && S4==1 && S5==1);
}

// ================== Motor Low-level (Manual only uses these) ==================
void setMotorRaw(int leftPWM, int rightPWM, bool leftFwd, bool rightFwd){
  leftPWM  = clampi(leftPWM,  0, 255);
  rightPWM = clampi(rightPWM, 0, 255);

  bool Lf = leftFwd  ^ LEFT_DIR_INVERT;
  bool Rf = rightFwd ^ RIGHT_DIR_INVERT;

  digitalWrite(IN3, Lf?HIGH:LOW);
  digitalWrite(IN4, Lf?LOW:HIGH);
  digitalWrite(IN1, Rf?HIGH:LOW);
  digitalWrite(IN2, Rf?LOW:HIGH);

  // set targets (rampApply จะขับไปหา)
  tgtLeftPWM = leftPWM; tgtRightPWM = rightPWM;
}
void stopAll(bool brake){
  if (brake){
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  }
  tgtLeftPWM = 0; tgtRightPWM = 0;
}
void rampApply(){
  // ถ้า tick นี้ auto เพิ่งเขียน PWM แล้ว → อย่าเขียนทับ
  if (autoDirectDriveLastTick){ autoDirectDriveLastTick = false; return; }

  int step = (mode=="manual" && manualPulseActive) ? MANUAL_RAMP_STEP : RAMP_STEP;

  if (curLeftPWM <  tgtLeftPWM) curLeftPWM  = min(curLeftPWM  + step, tgtLeftPWM);
  if (curLeftPWM >  tgtLeftPWM) curLeftPWM  = max(curLeftPWM  - step, tgtLeftPWM);
  if (curRightPWM < tgtRightPWM) curRightPWM = min(curRightPWM + step, tgtRightPWM);
  if (curRightPWM > tgtRightPWM) curRightPWM = max(curRightPWM - step, tgtRightPWM);

  int outL = curLeftPWM;
  int outR = curRightPWM;

  // เฉพาะ manual: ปรับ deadzone เฉพาะตอนพัลส์เลี้ยว
  if (mode=="manual"){
    if (manualPulseActive){
      outL = applyDeadzone(outL);
      outR = applyDeadzone(outR);
      outL = min(outL, MANUAL_MAX_PWM);
      outR = min(outR, MANUAL_MAX_PWM);
      if ((long)(millis() - manualPulseUntil) < 0){
        outL = max(outL, (int)KICK_PWM);
        outR = max(outR, (int)KICK_PWM);
      }
    } else {
      // เดินหน้า/ถอย: ไม่บังคับ deadzone เพื่อให้เริ่มช้าได้จริง
      outL = min(outL, MANUAL_MAX_PWM);
      outR = min(outR, MANUAL_MAX_PWM);
    }
  }

  ledcWrite(PWM_CH_LEFT,  outL);
  ledcWrite(PWM_CH_RIGHT, outR);
}

// ================== RTT Ping/Log ==================
void doPingOnce(){
  if (!serverPingUrl || !strlen(serverPingUrl)) return;
  HTTPClient http; http.setTimeout(3000);
  unsigned long t0 = millis();
  if (http.begin(serverPingUrl)){
    int code = http.GET();
    unsigned long dt = millis() - t0;
    Serial.printf("[PING] code=%d, dt=%lums\n", code, dt);
    http.end();

    if (serverLogUrl && strlen(serverLogUrl)){
      HTTPClient logh; logh.setTimeout(2000);
      if (logh.begin(serverLogUrl)){
        String msg = String("ESP32-MOTOR PING ")+dt+" ms";
        logh.addHeader("Content-Type","text/plain");
        logh.POST(msg);
        logh.end();
      }
    }
  }
}

// ================== Telemetry (non-blocking-ish) ==================
void sendTelemetryOnce(uint32_t now_ms){
  if (!serverTelemUrl || !strlen(serverTelemUrl)) return;
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.setTimeout(150); // สั้น ๆ กันค้าง

  String url = String(serverTelemUrl);
  url += "?ms=" + String(now_ms);
  url += "&ir=" + String(g_S1) + "," + String(g_S2) + "," + String(g_S3) + "," + String(g_S4) + "," + String(g_S5);
  url += "&PvLPWM=" + String(prevLeftPWM_mwi);
  url += "&PvRPWM=" + String(prevRightPWM_mwi);
  url += "&error=" + String(g_error);
  url += "&output=" + String(g_output);
  url += "&baseDyn=" + String(g_baseDyn);

  if (http.begin(url)) {
    http.GET(); // ไม่ต้องอ่าน body ลดเวลากดคอขวด
    http.end();
  }
}