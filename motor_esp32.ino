// === Line Follower + IR Log + LEDC PWM ===

int IN1=16, IN2=17, IN3=18, IN4=19;
int ENA=25, ENB=26;
int IR1=34, IR2=35, IR3=32, IR4=33, IR5=36;

// PID
float Kp=25, Ki=0, Kd=20;

// ---- ค่าคงที่ "A" ----
int STRAIGHT_MAX      = 70;
int CURVE_MAX         = 100;
int STRAIGHT_HARD_CAP = 70;

int PWM_SLEW = 12;

float integral=0; int lastError=0;
int prevLeftPWM=0, prevRightPWM=0;

const unsigned long LOG_EVERY_MS=100;
unsigned long lastLogMs=0;
int g_error=0, g_output=0, g_baseDyn=0;
int g_S1=1, g_S2=1, g_S3=1, g_S4=1, g_S5=1;
int g_curveId=0;

const float BASE_SCALE  = 1.0f;
const float SPEED_SCALE = 1.00f;
const float TURN_SCALE  = 0.60f;

// ---- เพดาน & ค่าขั้นต่ำ ----
int GLOBAL_MAX_PWM = 70;
const int MIN_RUN  = 90;
const int KICK_PWM = 180;
const int KICK_MS  = 100;
const int TURN_FLOOR = 60;

// ===== LEDC =====
const int PWM_FREQ_HZ   = 1000;
const int PWM_RES_BITS  = 8;
const int PWM_CH_RIGHT  = 0; // ENA
const int PWM_CH_LEFT   = 1; // ENB

// ---- เพดานโค้ง 8 แพทเทิร์น ----
const int TR1_CURVE_MAX1 = 100;
const int TR2_CURVE_MAX2 = 95;
const int TR3_CURVE_MAX3 = 90;
const int TR4_CURVE_MAX4 = 85;
const int TL1_CURVE_MAX8 = 100;
const int TL2_CURVE_MAX7 = 95;
const int TL3_CURVE_MAX6 = 90;
const int TL4_CURVE_MAX5 = 85;

// --------- เกณฑ์ pivot / reverse ----------
const int PIVOT_TH = 18;
const int REV_TH   = 35;

// ---- บูสต์ทางตรง ----
const int START_MIN = 70;
const int START_MS  = 200;
static unsigned long straightBoostUntil = 0;

// ---- "Gating" MIN_RUN ----
const int MIN_GATING_MS = 350;
static unsigned long minRunUntil = 0;

// ======== Prototypes (ต้องอยู่เหนือ FollowLine) ========
void forward();
void Reverse();
void TurnRight1();
void TurnLeft1();
void TurnRight2();
void TurnLeft2();
void TurnRight3();
void TurnLeft3();
void TurnRight4();
void TurnLeft4();
// ========================================================

void setup(){
  Serial.begin(115200);
  while(!Serial){;}
  Serial.println("IR1 IR2 IR3 IR4 IR5 => ms:..., PvLPWM:..., PvRPWM:..., error:..., output:..., baseDyn:..., curveId:...");

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  ledcSetup(PWM_CH_RIGHT, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(ENA, PWM_CH_RIGHT);
  ledcSetup(PWM_CH_LEFT,  PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(ENB, PWM_CH_LEFT);

  pinMode(IR1,INPUT); pinMode(IR2,INPUT); pinMode(IR3,INPUT);
  pinMode(IR4,INPUT); pinMode(IR5,INPUT);
}

void loop(){ FollowLine(); }

int slewStep(int prevVal,int targetVal,int maxStep){
  if(targetVal>prevVal+maxStep) return prevVal+maxStep;
  if(targetVal<prevVal-maxStep) return prevVal-maxStep;
  return targetVal;
}

void applyPWM(int leftPWM,int rightPWM){
  prevLeftPWM  = slewStep(prevLeftPWM,  leftPWM,  PWM_SLEW);
  prevRightPWM = slewStep(prevRightPWM, rightPWM, PWM_SLEW);

  ledcWrite(PWM_CH_RIGHT, prevRightPWM);
  ledcWrite(PWM_CH_LEFT,  prevLeftPWM);

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
    Serial.print(", baseDyn:");Serial.print(g_baseDyn);
    Serial.print(", curveId:");Serial.println(g_curveId);
  }
}

inline int packIR(int s1,int s2,int s3,int s4,int s5){
  return (s1<<4) | (s2<<3) | (s3<<2) | (s4<<1) | s5;
}
bool curveCapFromPattern(int s1,int s2,int s3,int s4,int s5, int &capOut, int &idOut){
  switch (packIR(s1,s2,s3,s4,s5)){
    case 0b11110: capOut = TR1_CURVE_MAX1; idOut = 1; return true;
    case 0b11100: capOut = TR2_CURVE_MAX2; idOut = 2; return true;
    case 0b11101: capOut = TR3_CURVE_MAX3; idOut = 3; return true;
    case 0b11001: capOut = TR4_CURVE_MAX4; idOut = 4; return true;
    case 0b10011: capOut = TL4_CURVE_MAX5; idOut = 5; return true;
    case 0b10111: capOut = TL3_CURVE_MAX6; idOut = 6; return true;
    case 0b00111: capOut = TL2_CURVE_MAX7; idOut = 7; return true;
    case 0b01111: capOut = TL1_CURVE_MAX8; idOut = 8; return true;
    default: return false;
  }
}

void FollowLine(){
  // อ่านค่า IR
  int S1=digitalRead(IR1), S2=digitalRead(IR2), S3=digitalRead(IR3),
      S4=digitalRead(IR4), S5=digitalRead(IR5);

  // อัปเดตตัวแปร log
  g_S1=S1; g_S2=S2; g_S3=S3; g_S4=S4; g_S5=S5;
  g_error=0; g_output=0; g_baseDyn=0;

  // ดำหมด → หยุด
  if (S1==0 && S2==0 && S3==0 && S4==0 && S5==0) {
    Stop();
    return;
  }

  // ---- แพทเทิร์นตามที่กำหนด ----
  if (S1==1 && S2==1 && S3==0 && S4==1 && S5==1) { // ขาว ขาว ดำ ขาว ขาว
    forward(); return;
  }
  else if (S1==1 && S2==1 && S3==1 && S4==1 && S5==1) { // ขาว ขาว ขาว ขาว ขาว
    Reverse(); return;
  }
  else if (S1==1 && S2==1 && S3==1 && S4==1 && S5==0) { // ขาว ขาว ขาว ขาว ดำ
    TurnRight1(); return;
  }
  else if (S1==0 && S2==1 && S3==1 && S4==1 && S5==1) { // ดำ ขาว ขาว ขาว ขาว
    TurnLeft1(); return;
  }
  else if (S1==0 && S2==1 && S3==1 && S4==0 && S5==0) { // ดำ ขาว ขาว ดำ ดำ
    TurnRight2(); return;
  }
  else if (S1==0 && S2==0 && S3==1 && S4==1 && S5==1) { // ดำ ดำ ขาว ขาว ขาว
    TurnLeft2(); return;
  }
  else if (S1==1 && S2==1 && S3==1 && S4==0 && S5==1) { // ขาว ขาว ขาว ดำ ขาว
    TurnRight3(); return;
  }
  else if (S1==1 && S2==0 && S3==1 && S4==1 && S5==1) { // ขาว ดำ ขาว ขาว ขาว
    TurnLeft3(); return;
  }
  else if (S1==1 && S2==1 && S3==0 && S4==0 && S5==1) { // ขาว ขาว ดำ ดำ ขาว
    TurnRight4(); return;
  }
  else if (S1==1 && S2==0 && S3==0 && S4==1 && S5==1) { // ขาว ดำ ดำ ขาว ขาว
    TurnLeft4(); return;
  }

  // ---- fallback ----
  if (S3==0) {
    forward();
  } else {
    int leftBlack  = (S1==0) + (S2==0);
    int rightBlack = (S4==0) + (S5==0);
    if (rightBlack > leftBlack)      TurnRight3();
    else if (leftBlack > rightBlack) TurnLeft3();
    else                              forward();
  }
}

void Stop(){
  for(int sp=max(prevLeftPWM,prevRightPWM); sp>=0; sp-=20){
    applyPWM(sp,sp); delay(15);
    digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
  }
  prevLeftPWM=prevRightPWM=0;
}

// ======== Action functions ========

void Reverse(){ // ขาว ขาว ขาว ขาว ขาว
  // RIGHT ถอย, LEFT ถอย
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); // RIGHT backward
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); // LEFT  backward
  applyPWM(70,70);
}

void forward(){  // ขาว ขาว ดำ ขาว ขาว
  // RIGHT เดินหน้า, LEFT เดินหน้า
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  // RIGHT forward
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  // LEFT  forward
  applyPWM(70,70);
}

void TurnRight1(){ // ขาว ขาว ขาว ขาว ดำ
  // RIGHT ถอย, LEFT เดินหน้า
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); // RIGHT backward
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  // LEFT  forward
  applyPWM(120,70); // (LEFT, RIGHT)
}

void TurnLeft1(){ // ดำ ขาว ขาว ขาว ขาว
  // RIGHT เดินหน้า, LEFT ถอย
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  // RIGHT forward
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); // LEFT  backward
  applyPWM(70,120);
}

void TurnRight2(){ // ขาว ขาว ขาว ดำ ดำ
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); // RIGHT backward
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  // LEFT  forward
  applyPWM(115,70);
}

void TurnLeft2(){ // ดำ ดำ ขาว ขาว ขาว
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  // RIGHT forward
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); // LEFT  backward
  applyPWM(70,115);
}

void TurnRight3(){ // ขาว ขาว ขาว ดำ ขาว
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); // RIGHT backward
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  // LEFT  forward
  applyPWM(110,70);
}

void TurnLeft3(){ // ขาว ดำ ขาว ขาว ขาว
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  // RIGHT forward
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); // LEFT  backward
  applyPWM(70,110);
}

void TurnRight4(){ // ขาว ขาว ดำ ดำ ขาว
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); // RIGHT backward
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  // LEFT  forward
  applyPWM(105,70);
}

void TurnLeft4(){ // ขาว ดำ ดำ ขาว ขาว
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  // RIGHT forward
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); // LEFT  backward
  applyPWM(70,105);
}
