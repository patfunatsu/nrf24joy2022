// PCA9685サーボモーターを動かす(受信側)、構造体7ch
// 表示機能追加 タイムアウト追加
// 電圧表示機能
// IR sensor表示機能 2022.07.31
// Steer制限 2022.08.06
// Ackerman geometry計算機能 2022.08.20

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <MsTimer2.h>
#include <PCA9685.h>  //PCA9685用ヘッダーファイル
#include <RF24.h>
#include <SPI.h>  // ライブラリのインクルード
#include <Wire.h>
#include <math.h>
#include <nRF24L01.h>
#define PI 3.141592653589793

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

#define SERVOMIN 122  //サーボの最小パルス幅 (0 to 4096)
#define SERVOMAX 492  //サーボの最大パルス幅 (0 to 4096)
#define SCNV 0.71     //角度変換係数

#define SERVO_CH1 0  // Drive 1
#define SERVO_CH2 1
#define SERVO_CH3 2
#define SERVO_CH4 3
#define SERVO_CH5 4  // Steer 5
#define SERVO_CH6 5
#define SERVO_CH7 6
#define SERVO_CH8 7

#define PowerPIN 9  // PCA9685 power
#define OEpin 10
#define RedLED 13

#define IRadd 8  // Inductive Radio

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

RF24 radio(7, 8);  // CE,CSNピンの指定
PCA9685 pwm =
  PCA9685(0x40);  // PCA9685のアドレス指定（アドレスジャンパ未接続時）

const byte address[6] = "20227";  // データを受信するアドレス

// アナログ入力ピンの設定 (認識用)
#define X1_PIN A1
#define Y1_PIN A0
#define X2_PIN A3
#define Y2_PIN A2
#define X3_PIN A6
#define X4_PIN A7

// デジタル入力ピンの設定 (認識用)
#define B1 2
#define B2 3
#define B3 4
#define B4 5
#define B5 6
#define S1 9
#define S2 10

// swdata , Limdataのビット構成
#define SWB1 0  // 上矢印
#define SWB2 1  // 左矢印
#define SWB3 2  // ホーム
#define SWB4 3  // 右矢印
#define SWB5 4  // 下矢印
#define SWB6 5  // 0;normal/ 1;trim 設定SW
#define SWB7 6  // 0;joystick/ 1;servo trim 設定SW
#define SWB8 7  // 予備

// IRのビット構成
#define LLbit 3
#define LFbit 2
#define RFbit 1
#define RRbit 0

byte IRdata = 0;  // IR data

// LimitスイッチのPort構成
#define FLSW 40
#define FRSW 41
#define RLSW 42
#define RRSW 43

struct Data_Package {
  byte Area1;
  byte Area2;
  byte Area3;
  byte Area4;
  byte Area5;
  byte Area6;
  byte Area7;
  byte Area8;
  byte swdata;
};
/*
  struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j2PotX;
  byte j2PotY;
  byte v3PotX;
  byte v4PotX;
  byte swdata;
  };
*/

// steer mode
#define TURN 1       // 前輪操舵
#define CRAB 2       // 横向き操舵
#define OPPOSIT 3    // 逆位相操舵
#define SPIN 4       // 超信地旋回
#define SAME 5       // 同位相操舵
#define STEERMAX 30  // 最大操舵角
#define UNLIMIT 80   //　無制限

//#define FIXSTEER 127 // 操舵無し

int steermode = OPPOSIT;
byte swdata = 0;    // リモコンスイッチ
byte limdata = 0;   // Limitスイッチ
Data_Package data;  // 構造体の宣言

// Joystickの受信値
int j1PotX_send = 0;  // sendは無線受信値
int j1PotY_send = 0;
int j2PotX_send = 0;
int j2PotY_send = 0;
int v3PotX_send = 0;
int v4PotX_send = 0;

// サーボのSend
int Svo1_send = 0;  // sendは出力
int Svo2_send = 0;
int Svo3_send = 0;
int Svo4_send = 0;
int Svo5_send = 0;
int Svo6_send = 0;
int Svo7_send = 0;
int Svo8_send = 0;

// サーボのTrims
int Svo1_fine = 0;  // 中心値(127)からのトリム調整用
int Svo2_fine = 0;
int Svo3_fine = 0;
int Svo4_fine = 0;
int Svo5_fine = 0;
int Svo6_fine = 0;
int Svo7_fine = 0;
int Svo8_fine = 0;

//サーボのinvert
bool Svo1_invert = 1;  // invertスイッチ
bool Svo2_invert = 0;
bool Svo3_invert = 1;
bool Svo4_invert = 0;
bool Svo5_invert = 0;
bool Svo6_invert = 0;
bool Svo7_invert = 0;
bool Svo8_invert = 0;
#define INVT true
#define NotINVT false
#define WB 39.0  // ホイールベース軸長
#define TR 25.0  // トレッド車幅

#define CNTIME 1000

unsigned long time_data = 0;
unsigned long time_old = 0;

#define divR1 11.01  // A0用
#define divR2 10.93  // A1用

float CPUBattery = 0.0;
float SRVBattery = 0.0;

void setup() {
  // Serial.begin(9600);
  Serial.begin(115200);
  analogReference(INTERNAL1V1);  // MEG用ADCを内部refで使う FS=1.1V
  Wire.begin();

  if (EEPROM.read(1) != 55) {  // cheack End mark
    EEPROM.write(2, 127);      // Servo 1 to 8
    EEPROM.write(3, 127);
    EEPROM.write(4, 127);
    EEPROM.write(5, 127);
    EEPROM.write(6, 127);
    EEPROM.write(7, 127);
    EEPROM.write(8, 127);
    EEPROM.write(9, 127);
    EEPROM.write(1, 55);  // End mark
  }

  Svo1_fine = EEPROM.read(2);
  Svo2_fine = EEPROM.read(3);
  Svo3_fine = EEPROM.read(4);
  Svo4_fine = EEPROM.read(5);
  Svo5_fine = EEPROM.read(6);
  Svo6_fine = EEPROM.read(7);
  Svo7_fine = EEPROM.read(8);
  Svo8_fine = EEPROM.read(9);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("nRF24L01 Reciver V0.0");
  display.display();

  radio.begin();                      // 無線オブジェクトの初期化
  radio.openReadingPipe(0, address);  // データ受信アドレスを指定
  radio.setPALevel(RF24_PA_MIN);      // 出力を最小に
  radio.startListening();             // 受信側として設定

  pinMode(PowerPIN, OUTPUT);
  pinMode(OEpin, OUTPUT);
  pinMode(RedLED, OUTPUT);

  OFF9685();  // PCA9685電源停止
  digitalWrite(RedLED, LOW);

  pwm.begin();  //初期設定 (アドレス0x40用)
  pwm.reset();
  pwm.setPWMFreq(50);  // PWM周期を50Hzに設定 (アドレス0x40用)

  delay(1000);

  ON9685();  // PCA9685電源投入
}

void loop() {
  int Angle;

  time_data = millis();

  if (radio.available()) {
    time_old = time_data;

    radio.read(&data, sizeof(Data_Package));  // アナログ値を受信する
    swdata = data.swdata;

    if (bitRead(swdata, SWB6) == false) {  // SWB6=0 normal
      transJoy();
      normalDisp();

      if (bitRead(swdata, SWB3) == false) {
        steermode = OPPOSIT;
      } else if (bitRead(swdata, SWB5) == false) {
        steermode = SAME;
      } else if (bitRead(swdata, SWB1) == false) {
        steermode = TURN;
      } else if (bitRead(swdata, SWB2) == false) {
        steermode = CRAB;
      } else if (bitRead(swdata, SWB4) == false) {
        steermode = SPIN;
      }

      // Steer
      switch (steermode) {
        case OPPOSIT:
          oppositSteer();
          break;
        case SAME:
          sameSteer();
          break;
        case TURN:
          turnSteer();
          break;
        case CRAB:
          crabSteer();
          break;
        case SPIN:
          spinSteer();
          break;
        default:
          oppositSteer();
      }
    }
    if (bitRead(swdata, SWB6) == true) {    // 1;trim
      if (bitRead(swdata, SWB7) == true) {  // 1;servo
        transServo();
        servoFine();
        Angle = map(Svo1_fine, 0, 255, 0, 180);
        servo_write(SERVO_CH1, Angle);
        Angle = map(Svo2_fine, 0, 255, 0, 180);
        servo_write(SERVO_CH2, Angle);
        Angle = map(Svo3_fine, 0, 255, 0, 180);
        servo_write(SERVO_CH3, Angle);
        Angle = map(Svo4_fine, 0, 255, 0, 180);
        servo_write(SERVO_CH4, Angle);
        Angle = map(Svo5_fine, 0, 255, 0, 180);
        servo_write(SERVO_CH5, Angle);
        Angle = map(Svo6_fine, 0, 255, 0, 180);
        servo_write(SERVO_CH6, Angle);
        Angle = map(Svo7_fine, 0, 255, 0, 180);
        servo_write(SERVO_CH7, Angle);
        Angle = map(Svo8_fine, 0, 255, 0, 180);
        servo_write(SERVO_CH8, Angle);
        if (bitRead(swdata, SWB3) == false) {  // false:0 Home
          EEPROMwrite();
          display.fillRect(0, 0, 128, 64, BLACK);
          display.setTextSize(2);
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.print("SAVE");
        }
      }
    }
  }
  if ((time_data - time_old) > CNTIME) {
    display.fillRect(0, 0, 128, 64, BLACK);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.print("Not Connect!!");
  }

  CPUBattery = batteryVoltage(A0, divR1);  // A0で係数divR1を使用
  // Serial.print(" CPU Volt =");
  // Serial.print(CPUBattery);
  SRVBattery = batteryVoltage(A1, divR2);  // A1で係数divR2を使用
  // Serial.print(" SRV Volt =");
  // Serial.println(SRVBattery);
  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print(CPUBattery, 1);
  display.setCursor(30, 40);
  display.print(SRVBattery, 1);

  // limit SWの読み出し
  limdata = limitset(limdata, FLSW, SWB1);
  limdata = limitset(limdata, FRSW, SWB2);
  limdata = limitset(limdata, RLSW, SWB3);
  limdata = limitset(limdata, RRSW, SWB4);

  display.setCursor(54, 40);
  display.print(limdata, BIN);
  display.setCursor(78, 40);
  display.print(steermode);  //操舵モード番号表示

  Wire.requestFrom(IRadd, 1);  // リクエスト1バイト
  while (Wire.available()) {
    IRdata = Wire.read();
    display.setCursor(90, 40);
    display.print(IRdata, BIN);
  }
  display.display();
  delay(10);
}

// ********** loop end ***********

//動かすサーボチャンネルと角度を指定
void servo_write(int ch, int ang) {
  //角度（0～180）をPWMのパルス幅（122～492）に変換
  ang = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(ch, 0, ang);
}

// バッテリー電圧測定
float batteryVoltage(int chnum, float divR) {
  int data;
  float voltage;
  data = analogRead(chnum);  // ADCを読む AnalogRef
  // Serial.print(data);
  voltage = divR * 1.1 * data / 1024.0;  // 電圧に換算
  return voltage;                        // 電圧をfloatで返す
}

// サーボの角度算出
int calcAngle(int Svosend, int fine, int stmax, bool reverse) {
  int val;
  float steer;
  if (reverse == true) {
    Svosend = 255 - Svosend;
  }
  Svosend = Svosend + fine - 127;
  val = map(Svosend, 0, 255, 0, 180);
  steer = fine * SCNV;
  val = constrain(val, steer - stmax, steer + stmax);
  return val;
}

// 駆動用Driveの角度算出
int driveAngle(int Svosend, int fine, bool reverse) {
  int val;
  if (reverse == true) {
    Svosend = 255 - Svosend;
  }
  Svosend = Svosend + fine - 127;
  val = map(Svosend, 0, 255, 55, 130);
  val = constrain(val, 60, 140); //速度制限
  return val;
}


// Ackermann Steer
int ackerSteer(int x, char ch) {
  int tpa;
  tpa = joyoutput(x);
  switch (ch) {
    case 'R':
      if (tpa >= 0) {
        tpa = Ackermann(tpa);
        tpa = constrain(tpa, -60, 25);
      }
      break;
    case 'L':
      if (tpa < 0) {
        tpa = Ackermann(tpa);
        tpa = constrain(tpa, -25, 60);
      }
      break;
    default:
      break;
  }
  return tpa;
}

// Ackermann geometry
// degreeで内側取角入力して、degreeで外側取角を返す
// 0 to 70degまでを想定
double Ackermann(double degree) {
  double radian;
  double alfer;
  double beta;
  int flag = 1;

  if (degree <= 0 ) {
    degree = abs(degree);
    flag = -1;
  }

  radian = degree * (PI / 180.0);
  alfer = 1 / (TR / WB + (1 / tan(radian)));
  beta = atan(alfer) * (180.0 / PI);
  //Serial.print(radian);Serial.print(",");
  if (flag <= 0) {
    beta = beta * ( -1.0);
  }
  return beta;
}

// Joystick(-90,0,90) input(30,0,225)と互換性をとる
int joyinput(int x) {
  int val;
  val = map(x, -90, 90, 30, 225);
  val = constrain(val, 30, 225);
  return val;
}

// output(30,0,225) Joystick(-90,0,90)と互換性をとる
int joyoutput(int y) {
  int val;
  val = map(y, 30, 225, -90, 90);
  val = constrain(val, -90, 90);
  return val;
}

// float 精度のmap関数
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// joystickのlimitマッピング
int map_limit(int val, int lower, int middle, int upper, bool reverse) {
  val = constrain(val, lower, upper);
  if (val < middle) {
    val = map(val, lower, middle, 0, 128);
  } else {
    val = map(val, middle, upper, 128, 255);
  }
  return (reverse ? 255 - val : val);
}

// LimitSW操作
// limdata,ポート,bit
byte limitset(byte x, int port, int n) {
  if (digitalRead(port) == false) {
    bitSet(x, n);
  } else {
    bitClear(x, n);
  }
  return x;
}

//  通常表示
void normalDisp() {
  display.fillRect(0, 0, 128, 64, BLACK);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(j1PotX_send);  // j1PotX
  display.setCursor(32, 0);
  display.print(j1PotY_send);  // j1PotY
  display.setCursor(64, 0);
  display.print(j2PotX_send);  // j2PotX
  display.setCursor(96, 0);
  display.print(j2PotY_send);  // j2PotY
  display.setCursor(0, 10);
  display.print(v3PotX_send);  // v3PotX
  display.setCursor(32, 10);
  display.print(v4PotX_send);  // v4PotX
  display.setCursor(0, 20);
  display.print(swdata, BIN);
}

// Servo Fine
void servoFine() {
  display.fillRect(0, 0, 128, 64, BLACK);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Servo Fine");
  display.setCursor(0, 10);
  display.print(Svo1_fine);
  display.setCursor(32, 10);
  display.print(Svo2_fine);
  display.setCursor(64, 10);
  display.print(Svo3_fine);
  display.setCursor(96, 10);
  display.print(Svo4_fine);
  display.setCursor(0, 20);
  display.print(Svo5_fine);
  display.setCursor(32, 20);
  display.print(Svo6_fine);
  display.setCursor(64, 20);
  display.print(Svo7_fine);
  display.setCursor(96, 20);
  display.print(Svo8_fine);
  display.setCursor(0, 30);
  display.print(swdata, BIN);
}

// Joystickデータ読込
void transJoy() {
  j1PotX_send = data.Area1;
  j1PotY_send = data.Area2;
  j2PotX_send = data.Area3;
  j2PotY_send = data.Area4;
  v3PotX_send = data.Area5;
  v4PotX_send = data.Area6;
}

// Servoのtrim読込
void transServo() {
  Svo1_fine = data.Area1;
  Svo2_fine = data.Area2;
  Svo3_fine = data.Area3;
  Svo4_fine = data.Area4;
  Svo5_fine = data.Area5;
  Svo6_fine = data.Area6;
  Svo7_fine = data.Area7;
  Svo8_fine = data.Area8;
}

// PCA9685電源停止
void OFF9685() {
  digitalWrite(PowerPIN, LOW);  // PCA9685電源停止
  delay(10);
  digitalWrite(OEpin, HIGH);  // PCA9685停止
}

// PCA9685電源投入
void ON9685() {
  digitalWrite(PowerPIN, HIGH);  // PCA9685電源投入
  delay(10);
  digitalWrite(OEpin, LOW);  // PCA9685作動開始
}

void EEPROMwrite() {
  EEPROM.write(2, Svo1_fine);  // Servo 1 to 8
  EEPROM.write(3, Svo2_fine);
  EEPROM.write(4, Svo3_fine);
  EEPROM.write(5, Svo4_fine);
  EEPROM.write(6, Svo5_fine);
  EEPROM.write(7, Svo6_fine);
  EEPROM.write(8, Svo7_fine);
  EEPROM.write(9, Svo8_fine);
}

// 逆位相操舵
void oppositSteer() {
  int Angle;
  int tpa;
  int tpb;

  // Steer
  Serial.print("3,");
  //Serial.print(j1PotY_send); Serial.print(":");
  //Serial.print(joyoutput(j1PotY_send));
  //Serial.print(":[");
  tpa = ackerSteer(j1PotY_send, 'R');
  tpb = joyinput(tpa);
  //Serial.print(tpa); Serial.print(":");
  //Serial.print(tpb); Serial.print(":");
  Angle = calcAngle(tpb, Svo5_fine, STEERMAX, INVT);
  servo_write(SERVO_CH5, Angle);
  Serial.print(Angle); Serial.print(",");

  tpa = ackerSteer(j1PotY_send, 'L');
  tpb = joyinput(tpa);
  //Serial.print(tpa); Serial.print(":");
  //Serial.print(tpb); Serial.print(":");
  Angle = calcAngle(tpb, Svo6_fine, STEERMAX, INVT);
  servo_write(SERVO_CH6, Angle) ;
  Serial.print(Angle); Serial.print(",");

  tpa = ackerSteer(j1PotY_send, 'R');
  tpb = joyinput(tpa);
  //Serial.print(tpa); Serial.print(":");
  //Serial.print(tpb); Serial.print(":");
  Angle = calcAngle(tpb, Svo7_fine, STEERMAX, NotINVT);
  servo_write(SERVO_CH7, Angle) ;
  Serial.print(Angle); Serial.print(",");

  tpa = ackerSteer(j1PotY_send, 'L');
  tpb = joyinput(tpa);
  //Serial.print(tpa); Serial.print(":");
  //Serial.print(tpb); Serial.print(":");
  Angle = calcAngle(tpb, Svo8_fine, STEERMAX, NotINVT);
  servo_write(SERVO_CH8, Angle) ;
  Serial.print(Angle); Serial.print(",");
  //Serial.println("");


  // Drive
  Angle = driveAngle(j2PotX_send, Svo1_fine, INVT);
  servo_write(SERVO_CH1, Angle) ;
  Serial.print(Angle); Serial.print(",");

  Angle = driveAngle(j2PotX_send, Svo2_fine, NotINVT);
  servo_write(SERVO_CH2, Angle) ;
  Serial.print(Angle); Serial.print(",");

  Angle = driveAngle(j2PotX_send, Svo3_fine, INVT);
  servo_write(SERVO_CH3, Angle) ;
  Serial.print(Angle); Serial.print(",");

  Angle = driveAngle(j2PotX_send, Svo4_fine, NotINVT);
  servo_write(SERVO_CH4, Angle) ;
  Serial.println(Angle);

}

// 同位相操舵
void sameSteer() {
  int Angle;
  // Steer
  Serial.print("5,");
  Angle = calcAngle(j1PotY_send, Svo5_fine, STEERMAX, INVT);
  servo_write(SERVO_CH5, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = calcAngle(j1PotY_send, Svo6_fine, STEERMAX, INVT);
  servo_write(SERVO_CH6, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = calcAngle(j1PotY_send, Svo7_fine, STEERMAX, INVT);
  servo_write(SERVO_CH7, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = calcAngle(j1PotY_send, Svo8_fine, STEERMAX, INVT);
  servo_write(SERVO_CH8, Angle);
  Serial.print(Angle);
  Serial.print(",");

  // Drive
  Angle = driveAngle(j2PotX_send, Svo1_fine, INVT);
  servo_write(SERVO_CH1, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = driveAngle(j2PotX_send, Svo2_fine, NotINVT);
  servo_write(SERVO_CH2, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = driveAngle(j2PotX_send, Svo3_fine, INVT);
  servo_write(SERVO_CH3, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = driveAngle(j2PotX_send, Svo4_fine, NotINVT);
  servo_write(SERVO_CH4, Angle);
  Serial.println(Angle);
}

// 緩旋回操舵
void turnSteer() {
  int Angle;
  int tpa;
  int tpb;

  // Steer
  Serial.print("1,");
  tpa = ackerSteer(j1PotY_send, 'R');
  tpb = joyinput(tpa);
  //Serial.print(tpa); Serial.print(":");
  //Serial.print(tpb); Serial.print(":");
  Angle = calcAngle(tpb, Svo5_fine, STEERMAX, INVT);
  servo_write(SERVO_CH5, Angle);
  Serial.print(Angle);
  Serial.print(",");

  tpa = ackerSteer(j1PotY_send, 'L');
  tpb = joyinput(tpa);
  //Serial.print(tpa); Serial.print(":");
  //Serial.print(tpb); Serial.print(":");
  Angle = calcAngle(tpb, Svo6_fine, STEERMAX, INVT);
  servo_write(SERVO_CH6, Angle);
  Serial.print(Angle);
  Serial.print(",");

  tpa = joyinput(0);
  Angle = calcAngle(tpa, Svo7_fine, STEERMAX, INVT);
  servo_write(SERVO_CH7, Angle);
  Serial.print(Angle);
  Serial.print(",");

  tpa = joyinput(0);
  Angle = calcAngle(tpa, Svo8_fine, STEERMAX, INVT);
  servo_write(SERVO_CH8, Angle);
  Serial.print(Angle);
  Serial.print(",");

  // Drive
  Angle = driveAngle(j2PotX_send, Svo1_fine, INVT);
  servo_write(SERVO_CH1, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = driveAngle(j2PotX_send, Svo2_fine, NotINVT);
  servo_write(SERVO_CH2, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = driveAngle(j2PotX_send, Svo3_fine, INVT);
  servo_write(SERVO_CH3, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = driveAngle(j2PotX_send, Svo4_fine, NotINVT);
  servo_write(SERVO_CH4, Angle);
  Serial.println(Angle);
}

// かに歩き操舵
void crabSteer() {
  int Angle;
  int tpa;

  // Steer
  Serial.print("2,");
  // tpa = 225;
  tpa = joyinput(90);
  Angle = calcAngle(tpa, Svo5_fine, UNLIMIT, INVT);
  servo_write(SERVO_CH5, Angle);
  // Serial.print(tpa); Serial.print(",");
  Serial.print(Angle);
  Serial.print(",");

  // tpa = 30;
  tpa = joyinput(-90);
  Angle = calcAngle(tpa, Svo6_fine, UNLIMIT, INVT);
  servo_write(SERVO_CH6, Angle);
  Serial.print(Angle);
  Serial.print(",");

  // tpa = 225;
  tpa = joyinput(90);
  Angle = calcAngle(tpa, Svo7_fine, UNLIMIT, NotINVT);
  servo_write(SERVO_CH7, Angle);
  Serial.print(Angle);
  Serial.print(",");

  // tpa = 30;
  tpa = joyinput(-90);
  Angle = calcAngle(tpa, Svo8_fine, UNLIMIT, NotINVT);
  servo_write(SERVO_CH8, Angle);
  Serial.print(Angle);
  Serial.print(",");

  // Drive 右stick 左右で操作
  Angle = driveAngle(j2PotY_send, Svo1_fine, INVT);
  // Angle = driveAngle(v3PotX_send, Svo1_fine, INVT); //テスト環境
  servo_write(SERVO_CH1, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = driveAngle(j2PotY_send, Svo2_fine, INVT);
  servo_write(SERVO_CH2, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = driveAngle(j2PotY_send, Svo3_fine, NotINVT);
  servo_write(SERVO_CH3, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = driveAngle(j2PotY_send, Svo4_fine, NotINVT);
  servo_write(SERVO_CH4, Angle);
  Serial.println(Angle);
}

// 超信地操舵
void spinSteer() {
  int Angle;
  int tpa;

  // Steer
  Serial.print("4,");
  tpa = joyinput(50);
  Angle = calcAngle(tpa, Svo5_fine, UNLIMIT, INVT);
  servo_write(SERVO_CH5, Angle);
  // Serial.print(tpa); Serial.print(",");
  Serial.print(Angle);
  Serial.print(",");

  tpa = joyinput(-50);
  Angle = calcAngle(tpa, Svo6_fine, UNLIMIT, INVT);
  servo_write(SERVO_CH6, Angle);
  Serial.print(Angle);
  Serial.print(",");

  tpa = joyinput(50);
  Angle = calcAngle(tpa, Svo7_fine, UNLIMIT, NotINVT);
  servo_write(SERVO_CH7, Angle);
  Serial.print(Angle);
  Serial.print(",");

  tpa = joyinput(-50);
  Angle = calcAngle(tpa, Svo8_fine, UNLIMIT, NotINVT);
  servo_write(SERVO_CH8, Angle);
  Serial.print(Angle);
  Serial.print(",");

  // Drive 右stick 左右で操作
  Angle = driveAngle(j2PotY_send, Svo1_fine, INVT);
  // Angle = driveAngle(v3PotX_send, Svo1_fine, INVT); //テスト環境
  servo_write(SERVO_CH1, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = driveAngle(j2PotY_send, Svo2_fine, INVT);
  servo_write(SERVO_CH2, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = driveAngle(j2PotY_send, Svo3_fine, INVT);
  servo_write(SERVO_CH3, Angle);
  Serial.print(Angle);
  Serial.print(",");

  Angle = driveAngle(j2PotY_send, Svo4_fine, INVT);
  servo_write(SERVO_CH4, Angle);
  Serial.println(Angle);
}
