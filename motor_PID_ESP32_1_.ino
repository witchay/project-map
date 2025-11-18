// === Line Follower + IR Log + LEDC PWM (PID-only) ===

int IN1=16, IN2=17, IN3=18, IN4=19;   // RIGHT=IN1,IN2  LEFT=IN3,IN4
int ENA=25, ENB=26;                   // RIGHT=ENA, LEFT=ENB
int IR1=34, IR2=35, IR3=32, IR4=33, IR5=27;

// ===== PID CONFIG =====
float Kp = 35, Ki = 0, Kd = 35;
const float TURN_SCALE = 0.55f;

const int   STRAIGHT_MAX      = 85;   // ฐานตรง
const int   STRAIGHT_HARD_CAP = 83;   // เพดานตอนตรงชัด
const int   CURVE_MIN         = 65;   // ฐานต่ำสุดตอนโค้งจัด (ยิ่งต่ำยิ่งช้า)
const int   GLOBAL_MAX_PWM    = 100;   // เพดานรวม PWM (ทั้งเดินหน้า/ถอย)

const float BASE_SCALE  = 1.0f;
const float SPEED_SCALE = 1.0f;

// smoothing ของอนุพันธ์เล็กน้อยให้วิ่งนิ่งขึ้น
static float dLP = 0;
const  float D_ALPHA = 0.30f;         // 0..1 (มาก = ไว)

// ===== Reverse speed =====
const int REVERSE_PWM = 80;           // ถอยช้า (ตอน 11111)

// ===== Slew =====
int PWM_SLEW = 30;

// ===== State / Logs =====
static float integral=0; 
static int   lastError=0;
int prevLeftPWM=0, prevRightPWM=0;

const unsigned long LOG_EVERY_MS=100;
unsigned long lastLogMs=0;
int g_error=0, g_output=0, g_baseDyn=0;
int g_S1=1, g_S2=1, g_S3=1, g_S4=1, g_S5=1;

// ===== LEDC =====
const int PWM_FREQ_HZ   = 1000;
const int PWM_RES_BITS  = 8;
const int PWM_CH_RIGHT  = 0; // ENA
const int PWM_CH_LEFT   = 1; // ENB

// ===== Prototypes =====
void drivePID(int S1,int S2,int S3,int S4,int S5);
void writeSignedPWM(int L, int R);

void setup(){
  Serial.begin(115200);
  while(!Serial){;}
  Serial.println("IR1 IR2 IR3 IR4 IR5 => ms:..., PvLPWM:..., PvRPWM:..., error:..., output:..., baseDyn:...");

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  ledcSetup(PWM_CH_RIGHT, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(ENA, PWM_CH_RIGHT);
  ledcSetup(PWM_CH_LEFT,  PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(ENB, PWM_CH_LEFT);

  pinMode(IR1,INPUT); pinMode(IR2,INPUT); pinMode(IR3,INPUT);
  pinMode(IR4,INPUT); pinMode(IR5,INPUT_PULLDOWN);
}

void loop(){ FollowLine(); }

// ===== Helpers =====
int slewStep(int prevVal,int targetVal,int maxStep){
  if(targetVal>prevVal+maxStep) return prevVal+maxStep;
  if(targetVal<prevVal-maxStep) return prevVal-maxStep;
  return targetVal;
}

void applyPWM(int leftPWM,int rightPWM){
  prevLeftPWM  = slewStep(prevLeftPWM,  leftPWM,  PWM_SLEW);
  prevRightPWM = slewStep(prevRightPWM, rightPWM, PWM_SLEW);

  ledcWrite(PWM_CH_RIGHT, prevRightPWM);  // ENA (RIGHT)
  ledcWrite(PWM_CH_LEFT,  prevLeftPWM);   // ENB (LEFT)

  unsigned long now=millis();
  if(now-lastLogMs>=LOG_EVERY_MS){
    lastLogMs=now;
    Serial.print(g_S1); Serial.print(" ");
    Serial.print(g_S2); Serial.print(" ");
    Serial.print(g_S3); Serial.print(" ");
    Serial.print(g_S4); Serial.print(" ");
    Serial.print(g_S5); Serial.print(" => ");
    Serial.print("ms:");       Serial.print(now);
    Serial.print(", PvLPWM:"); Serial.print(prevLeftPWM);
    Serial.print(", PvRPWM:"); Serial.print(prevRightPWM);
    Serial.print(", error:");  Serial.print(g_error);
    Serial.print(", output:"); Serial.print(g_output);
    Serial.print(", baseDyn:");Serial.println(g_baseDyn);
  }
}

// ===== Main control =====
void FollowLine(){
  // อ่าน IR (ขาว=1, ดำ=0)
  int S1=digitalRead(IR1), S2=digitalRead(IR2), S3=digitalRead(IR3),
      S4=digitalRead(IR4), S5=digitalRead(IR5);

  g_S1=S1; g_S2=S2; g_S3=S3; g_S4=S4; g_S5=S5;

  // ดำหมด → หยุด
  if (S1==0 && S2==0 && S3==0 && S4==0 && S5==0){
    // soft stop
    for(int sp=max(prevLeftPWM,prevRightPWM); sp>=0; sp-=20){
      applyPWM(sp,sp); delay(15);
    }
    digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
    prevLeftPWM=prevRightPWM=0;
    return;
  }

  // ขาวหมด → ถอยช้า (ค้นหาเส้น)
  if (S1==1 && S2==1 && S3==1 && S4==1 && S5==1){
    digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); // RIGHT backward
    digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); // LEFT  backward
    applyPWM(REVERSE_PWM, REVERSE_PWM);
    return;
  }

  // ปกติทั้งหมด → ใช้ PID คุมเลี้ยว/ความเร็ว (รวมถึงถอยล้อในเมื่อจำเป็น)
  drivePID(S1,S2,S3,S4,S5);
}

// ===== PID core (signed output) =====
void drivePID(int S1,int S2,int S3,int S4,int S5){
  // 1) error: ตำแหน่งเส้น (ขาว=1, ดำ=0) ตรงกลางดำ → error~0
  int position = S1*-2 + S2*-1 + S3*0 + S4*1 + S5*2;
  int error = position;

  // 2) PID (anti-windup + derivative smoothing)
  integral += error;
  integral = constrain(integral, -120, 120);
  int dRaw = error - lastError;
  dLP = D_ALPHA * dRaw + (1.0f - D_ALPHA) * dLP;
  int turn = (int)((Kp*error + Ki*integral + Kd*dLP) * TURN_SCALE);
  lastError = error;

  // 3) ฐานความเร็ว: ยิ่งเอียง (|error| สูง) ยิ่งลดฐานลงไปหา CURVE_MIN
  float eNorm = min(abs(error), 2) / 2.0f; // 0..1
  float alpha = 1.2f;                      // ความชัน 1.0–1.4
  float w = pow(eNorm, alpha);
  int baseDyn = STRAIGHT_MAX - (int)((STRAIGHT_MAX - CURVE_MIN) * w);
  baseDyn = (int)(baseDyn * BASE_SCALE);
  baseDyn = min(baseDyn, GLOBAL_MAX_PWM);

  // ทางตรงชัด: กดเพดานเพิ่ม
  bool straightPattern = (S3==0) && ((S2==1)||(S4==1));
  if (straightPattern) baseDyn = min(baseDyn, STRAIGHT_HARD_CAP);

  // 4) ผสมแบบ signed (ติดลบ = ถอยล้อใน)
  int leftSigned  = (int)((baseDyn - turn) * SPEED_SCALE);
  int rightSigned = (int)((baseDyn + turn) * SPEED_SCALE);

  // 5) ส่งออกแบบกำหนดทิศจากเครื่องหมาย
  writeSignedPWM(leftSigned, rightSigned);

  // logs
  g_error=error; g_output=turn; g_baseDyn=baseDyn;
}

// เขียน PWM แบบ signed: กำหนดทิศให้เองแล้วส่ง PWM เป็นค่าสัมบูรณ์
void writeSignedPWM(int L, int R){
  // หนีบเพดานทั้งบวก/ลบ
  L = constrain(L, -GLOBAL_MAX_PWM, GLOBAL_MAX_PWM);
  R = constrain(R, -GLOBAL_MAX_PWM, GLOBAL_MAX_PWM);

  // ตั้งทิศ LEFT (IN3,IN4) / RIGHT (IN1,IN2)
  if (L >= 0){ digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  } // LEFT forward
  else        { digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); } // LEFT backward

  if (R >= 0){ digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  } // RIGHT forward
  else        { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); } // RIGHT backward

  // ส่ง PWM
  applyPWM(abs(L), abs(R));
}
