/*
  表示機能付き送信機,構造体4ch,センター調整,ボタン入力追加,スライダー追加
  MCP3425 A/D による電池電圧測定 2022.04.15
  Servoのトリム調整追加 2022.07.01
  exponentialの機能追加 2022.08.05
  チャタリング防止のためBounce2.hを導入 2022.08.07 Ver 0.6

*/

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <RF24.h>
#include <SPI.h>  // I2Cライブラリのインクルード
#include <Wire.h>
#include <nRF24L01.h>
#include <Bounce2.h> //チャタリング防止

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

#define VR_IDEL 1  // 0 to 255 Center 127
#define LOW_IDEL (127 - VR_IDEL)
#define HI_IDEL (127 + VR_IDEL)

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

RF24 radio(7, 8);                 // CE,CSNピンの指定
const byte address[6] = "20227";  // データを送信するアドレス

// アナログ入力ピンの設定
#define X1_PIN A1
#define Y1_PIN A0
#define X2_PIN A3
#define Y2_PIN A2
#define X3_PIN A6
#define X4_PIN A7

// デジタル入力ピンの設定
#define B1 2
#define B2 3
#define B3 4
#define B4 5
#define B5 6
#define S1 9
#define S2 10

// Bounce関係の設定
#define INTER 5 // ms
Bounce bounB1 = Bounce();
Bounce bounB2 = Bounce();
Bounce bounB3 = Bounce();
Bounce bounB4 = Bounce();
Bounce bounB5 = Bounce();
Bounce bounS1 = Bounce();
Bounce bounS2 = Bounce();

// swdataのビット構成
#define SWB1 0 // 上矢印
#define SWB2 1 // 左矢印
#define SWB3 2 // ホーム,invertで使用
#define SWB4 3 // 右矢印
#define SWB5 4 // 下矢印
#define SWB6 5 // normal/trim 設定SW
#define SWB7 6 // joystick/servo trim 設定SW
#define SWB8 7 // 予備

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
byte swdata = 0;
Data_Package data;  // 構造体の宣言

// Variables
int j1PotX_fine = 0;  // fineはトリム調整用
int j1PotY_fine = 0;
int j2PotX_fine = 0;
int j2PotY_fine = 0;
int v3PotX_fine = 0;
int v4PotX_fine = 0;

int j1PotX_send = 0;  // sendは無線送信前の値
int j1PotY_send = 0;
int j2PotX_send = 0;
int j2PotY_send = 0;
int v3PotX_send = 0;
int v4PotX_send = 0;

bool j1PotX_invert = 0;  // invertスイッチ
bool j1PotY_invert = 0;
bool j2PotX_invert = 0;
bool j2PotY_invert = 0;
bool v3PotX_invert = 0;
bool v4PotX_invert = 0;

// サーボのTrims
int Svo1_fine = 0;  // 中心値(127)からのトリム調整用
int Svo2_fine = 0;
int Svo3_fine = 0;
int Svo4_fine = 0;
int Svo5_fine = 0;
int Svo6_fine = 0;
int Svo7_fine = 0;
int Svo8_fine = 0;

#define MCP3425_address 0x68
#define configRegister 0b10011000  // 16bit 15sps PGA x1
float Volts;
float Vref = 2.048;

// 重み付け
bool joymode = false; // normal(true) or exponential(false)

void setup() {
  Serial.begin(9600);

  if (EEPROM.read(1) != 55) {  // cheack End mark
    EEPROM.write(2, 127);      // jPotX1 to v4PotX
    EEPROM.write(3, 127);
    EEPROM.write(4, 127);
    EEPROM.write(5, 127);
    EEPROM.write(6, 127);
    EEPROM.write(7, 127);
    EEPROM.write(8, 0);  // jPotX1 invert to v4PotX
    EEPROM.write(9, 0);
    EEPROM.write(10, 0);
    EEPROM.write(11, 0);
    EEPROM.write(12, 0);
    EEPROM.write(13, 0);
    EEPROM.write(14, 127);  // Svo1 to Svo8
    EEPROM.write(15, 127);
    EEPROM.write(16, 127);
    EEPROM.write(17, 127);
    EEPROM.write(18, 127);
    EEPROM.write(19, 127);
    EEPROM.write(20, 127);
    EEPROM.write(21, 127);
    EEPROM.write(1, 55);  // End mark
  }

  j1PotX_fine = EEPROM.read(2);
  j1PotY_fine = EEPROM.read(3);
  j2PotX_fine = EEPROM.read(4);
  j2PotY_fine = EEPROM.read(5);
  v3PotX_fine = EEPROM.read(6);
  v4PotX_fine = EEPROM.read(7);

  j1PotX_invert = EEPROM.read(8);
  j1PotY_invert = EEPROM.read(9);
  j2PotX_invert = EEPROM.read(10);
  j2PotY_invert = EEPROM.read(11);
  v3PotX_invert = EEPROM.read(12);
  v4PotX_invert = EEPROM.read(13);

  Svo1_fine = EEPROM.read(14);
  Svo2_fine = EEPROM.read(15);
  Svo3_fine = EEPROM.read(16);
  Svo4_fine = EEPROM.read(17);
  Svo5_fine = EEPROM.read(18);
  Svo6_fine = EEPROM.read(19);
  Svo7_fine = EEPROM.read(20);
  Svo8_fine = EEPROM.read(21);

  pinMode(X1_PIN, INPUT);
  pinMode(Y1_PIN, INPUT);
  pinMode(X2_PIN, INPUT);
  pinMode(Y2_PIN, INPUT);
  pinMode(X3_PIN, INPUT);
  pinMode(X4_PIN, INPUT);
  pinMode(B1, INPUT_PULLUP);
  bounB1.attach(B1);
  bounB1.interval(INTER);
  pinMode(B2, INPUT_PULLUP);
  bounB2.attach(B2);
  bounB2.interval(INTER);
  pinMode(B3, INPUT_PULLUP);
  bounB3.attach(B3);
  bounB3.interval(INTER);
  pinMode(B4, INPUT_PULLUP);
  bounB4.attach(B4);
  bounB4.interval(INTER);
  pinMode(B5, INPUT_PULLUP);
  bounB5.attach(B5);
  bounB5.interval(INTER);
  pinMode(S1, INPUT_PULLUP);
  bounS1.attach(S1);
  bounS1.interval(INTER);
  pinMode(S2, INPUT_PULLUP);
  bounS2.attach(S2);
  bounS2.interval(INTER);

  radio.begin();                   // 無線オブジェクトの初期化
  radio.openWritingPipe(address);  // データ送信先のアドレスを指定
  radio.setPALevel(RF24_PA_HIGH);  // 出力を最大に
  radio.stopListening();           // 送信側として設定

  Wire.begin();
  Wire.beginTransmission(MCP3425_address);  // MCP3425の設定
  Wire.write(configRegister);
  Wire.endTransmission();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.fillRect(0, 0, 128, 64, BLACK);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Ver 0.6");
  display.display();
  delay(2000);
}

void loop() {
  int vrtemp;
  int adcValue;

  // チャタリング防止update
  bounUpdate(); // Update the Bounce instances

  vrtemp = map_normal(analogRead(X1_PIN), 0, 512, 1023, j1PotX_invert);
  j1PotX_send = j1PotX_send * 0.4 + VRadjust(vrtemp) * 0.6;

  // Y1,X2だけ重み付けする
  if (joymode == true) {
    vrtemp = map_normal(analogRead(Y1_PIN), 0, 512, 1023, j1PotY_invert);
  } else {
    vrtemp = map_exponential(analogRead(Y1_PIN), j1PotY_invert);
  }
  j1PotY_send = j1PotY_send * 0.4 + VRadjust(vrtemp) * 0.6;

  if (joymode == true) {
    vrtemp = map_normal(analogRead(X2_PIN), 0, 512, 1023, j2PotX_invert);
  } else {
    vrtemp = map_exponential(analogRead(X2_PIN), j2PotX_invert);
  }
  j2PotX_send = j2PotX_send * 0.4 + VRadjust(vrtemp) * 0.6;

  vrtemp = map_normal(analogRead(Y2_PIN), 0, 512, 1023, j2PotY_invert);
  j2PotY_send = j2PotY_send * 0.4 + VRadjust(vrtemp) * 0.6;

  vrtemp = map_normal(analogRead(X3_PIN), 0, 512, 1023, v3PotX_invert);
  v3PotX_send = v3PotX_send * 0.4 + VRadjust(vrtemp) * 0.6;

  vrtemp = map_normal(analogRead(X4_PIN), 0, 512, 1023, v4PotX_invert);
  v4PotX_send = v4PotX_send * 0.4 + VRadjust(vrtemp) * 0.6;

  j1PotX_send = j1PotX_send + j1PotX_fine - 127;
  j1PotY_send = j1PotY_send + j1PotY_fine - 127;
  j2PotX_send = j2PotX_send + j2PotX_fine - 127;
  j2PotY_send = j2PotY_send + j2PotY_fine - 127;
  v3PotX_send = v3PotX_send + v3PotX_fine - 127;
  v4PotX_send = v4PotX_send + v4PotX_fine - 127;

  swdata = swdataset(swdata, bounB1.read(), SWB1);
  swdata = swdataset(swdata, bounB2.read(), SWB2);
  swdata = swdataset(swdata, bounB3.read(), SWB3);
  swdata = swdataset(swdata, bounB4.read(), SWB4);
  swdata = swdataset(swdata, bounB5.read(), SWB5);
  swdata = swdataset(swdata, bounS1.read(), SWB6);
  swdata = swdataset(swdata, bounS2.read(), SWB7);
  data.swdata = swdata;

  if (bitRead(swdata, SWB6) == false) {  // 1: 通常のJoystick(false=0)
    //swdata = bitSet(swdata, SWB8);       // Servo trim設定でない SWB8=(true=1)
    data.swdata = swdata;
    data.Area1 = constrain(j1PotX_send, 0, 255);
    data.Area2 = constrain(j1PotY_send, 0, 255);
    data.Area3 = constrain(j2PotX_send, 0, 255);
    data.Area4 = constrain(j2PotY_send, 0, 255);
    data.Area5 = constrain(v3PotX_send, 0, 255);
    data.Area6 = constrain(v4PotX_send, 0, 255);
    data.Area7 = 127;
    data.Area8 = 127;
  }
  if (bitRead(swdata, SWB6) == true) {
    if (bitRead(swdata, SWB7) == true) {  // trim設定 SW6=1,SW7=1:Servo
      //swdata = bitClear(swdata, SWB8);           // Servo trim設定　SWB8=(false=0)
      data.swdata = swdata;
      data.Area1 = constrain(Svo1_fine, 0, 255);
      data.Area2 = constrain(Svo2_fine, 0, 255);
      data.Area3 = constrain(Svo3_fine, 0, 255);
      data.Area4 = constrain(Svo4_fine, 0, 255);
      data.Area5 = constrain(Svo5_fine, 0, 255);
      data.Area6 = constrain(Svo6_fine, 0, 255);
      data.Area7 = constrain(Svo7_fine, 0, 255);
      data.Area8 = constrain(Svo8_fine, 0, 255);
    }
  }

  radio.write(&data, sizeof(Data_Package));

  adcValue = readADC();
  // Volts = (int)adcValue * Vref / 32767.0;
  Volts = (int)adcValue * Vref / 32767.0 * 3.255;
  // Serial.println(String(Volts, 5));

  //delay(10);

  if (bitRead(swdata, SWB6) == false) {  // SWB6スイッチは通常とトリム調整
    normalDisp();
  } else if (bitRead(swdata, SWB7) ==
             false) {  // SWB7スイッチは　joystickとservo
    fineDisp();
    fine_set();
    //delay(10);
  } else if (bitRead(swdata, SWB7) == true) {
    servoFine();
    servo_set();
  }
  display.display();
  //delay(10);
}

// ********** END LOOP **********

// 線形マッピング
int map_normal(int val, int lower, int middle, int upper, bool reverse) {
  val = constrain(val, lower, upper);
  if (val < middle) {
    val = map(val, lower, middle, 0, 127);
  } else {
    val = map(val, middle, upper, 127, 255);
  }
  return (reverse ? 255 - val : val);
}

// 指数関数マッピング
int map_exponential(int val, bool reverse) {
  val = constrain(val, 0, 1023);
  float cube = ((pow((val - 512), 3) / 520200) + 258.012) / 2;
  return ( reverse ? 255 - cube : cube);
}

// VR center adjust
int VRadjust(int x) {
  int y;
  if (x <= LOW_IDEL) {
    y = map(x, 0, LOW_IDEL, 0, 126);
  } else if ((x > LOW_IDEL) & (x < HI_IDEL)) {
    y = 127;
  } else if (x >= HI_IDEL) {
    y = map(x, HI_IDEL, 255, 128, 255);
  }
  return y;
}

// swdataの収集
byte swdataset(byte x, int port, int n) {
  if (port == true) {
    bitSet(x, n);
  } else {
    bitClear(x, n);
  }
  return x;
}

// Bounce Update
void bounUpdate() {
  bounB1.update();
  bounB2.update();
  bounB3.update();
  bounB4.update();
  bounB5.update();
  bounS1.update();
  bounS2.update();
}

// MCP3425の電圧測定
int readADC() {
  Wire.requestFrom(MCP3425_address, 2);
  int nfRtn = (Wire.read() << 8) + Wire.read();
  return (nfRtn);
}

// 通常表示
void normalDisp() {
  display.fillRect(0, 0, 128, 64, BLACK);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(data.Area1);
  display.setCursor(32, 0);
  display.print(data.Area2);
  display.setCursor(64, 0);
  display.print(data.Area3);
  display.setCursor(96, 0);
  display.print(data.Area4);
  display.setCursor(0, 10);
  display.print(data.Area5);
  display.setCursor(32, 10);
  display.print(data.Area6);
  display.setCursor(0, 20);
  display.print(data.swdata, BIN);
  display.setCursor(54, 20);
  display.print(Volts, 2);
}

// Joystick fine値を表示
void fineDisp() {
  display.fillRect(0, 0, 128, 64, BLACK);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("CH:");
  display.setCursor(0, 10);
  display.print("Fine:");
  display.setCursor(0, 20);
  display.print("Invt:");
  display.setCursor(0, 30);
  display.print(j1PotX_fine);
  display.setCursor(32, 30);
  display.print(j1PotY_fine);
  display.setCursor(64, 30);
  display.print(j2PotX_fine);
  display.setCursor(96, 30);
  display.print(j2PotY_fine);
  display.setCursor(0, 40);
  display.print(v3PotX_fine);
  display.setCursor(32, 40);
  display.print(v4PotX_fine);
}

// Servo Fine
void servoFine() {
  display.fillRect(0, 0, 128, 64, BLACK);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Svo:");
  display.setCursor(0, 10);
  display.print("Fine:");
  display.setCursor(0, 30);
  display.print(data.Area1);
  display.setCursor(32, 30);
  display.print(data.Area2);
  display.setCursor(64, 30);
  display.print(data.Area3);
  display.setCursor(96, 30);
  display.print(data.Area4);
  display.setCursor(0, 40);
  display.print(data.Area5);
  display.setCursor(32, 40);
  display.print(data.Area6);
  display.setCursor(64, 40);
  display.print(data.Area7);
  display.setCursor(96, 40);
  display.print(data.Area8);
}

void fine_set() {
  static int pagevar = 1;

  display.setTextColor(WHITE);
  display.setCursor(32, 0);
  display.print(pagevar);  // CH番号の表示

  switch (pagevar) { // CH毎に設定
    case 1:
      display.setCursor(32, 10);
      display.print(j1PotX_fine);
      display.setCursor(32, 20);
      display.print(j1PotX_invert);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        j1PotX_fine = j1PotX_fine + 1;
        EEPROM.write(2, j1PotX_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        j1PotX_fine = j1PotX_fine - 1;
        EEPROM.write(2, j1PotX_fine);
      }
      if ((bitRead(swdata, SWB3) == false) && bounB3.fell()) {
        j1PotX_invert = !j1PotX_invert;
        EEPROM.write(8, j1PotX_invert);
      }
      if ((bitRead(swdata, SWB2) == false) && bounB2.fell()) {
        pagevar = pagevar - 1;
        if (pagevar <= 1) {
          pagevar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false) && bounB4.fell()) {
        pagevar = pagevar + 1;
        if (pagevar >= 6) {
          pagevar = 6;
        }
      }
      break;
    case 2:
      display.setCursor(32, 10);
      display.print(j1PotY_fine);
      display.setCursor(32, 20);
      display.print(j1PotY_invert);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        j1PotY_fine = j1PotY_fine + 1;
        EEPROM.write(3, j1PotY_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        j1PotY_fine = j1PotY_fine - 1;
        EEPROM.write(3, j1PotY_fine);
      }
      if ((bitRead(swdata, SWB3) == false) && bounB3.fell()) {
        j1PotY_invert = !j1PotY_invert;
        EEPROM.write(9, j1PotY_invert);
      }
      if ((bitRead(swdata, SWB2) == false)  && bounB2.fell()) {
        pagevar = pagevar - 1;
        if (pagevar <= 1) {
          pagevar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false)  && bounB4.fell()) {
        pagevar = pagevar + 1;
        if (pagevar >= 6) {
          pagevar = 6;
        }
      }
      break;
    case 3:
      display.setCursor(32, 10);
      display.print(j2PotX_fine);
      display.setCursor(32, 20);
      display.print(j2PotX_invert);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        j2PotX_fine = j2PotX_fine + 1;
        EEPROM.write(4, j2PotX_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        j2PotX_fine = j2PotX_fine - 1;
        EEPROM.write(4, j2PotX_fine);
      }
      if ((bitRead(swdata, SWB3) == false) && bounB3.fell()) {
        j2PotX_invert = !j2PotX_invert;
        EEPROM.write(10, j2PotX_invert);
      }
      if ((bitRead(swdata, SWB2) == false) && bounB2.fell()) {
        pagevar = pagevar - 1;
        if (pagevar <= 1) {
          pagevar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false) && bounB4.fell()) {
        pagevar = pagevar + 1;
        if (pagevar >= 6) {
          pagevar = 6;
        }
      }
      break;
    case 4:
      display.setCursor(32, 10);
      display.print(j2PotY_fine);
      display.setCursor(32, 20);
      display.print(j2PotY_invert);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        j2PotY_fine = j2PotY_fine + 1;
        EEPROM.write(5, j2PotY_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        j2PotY_fine = j2PotY_fine - 1;
        EEPROM.write(5, j2PotY_fine);
      }
      if ((bitRead(swdata, SWB3) == false) && bounB3.fell()) {
        j2PotY_invert = !j2PotY_invert;
        EEPROM.write(11, j2PotY_invert);
      }
      if ((bitRead(swdata, SWB2) == false) && bounB2.fell()) {
        pagevar = pagevar - 1;
        if (pagevar <= 1) {
          pagevar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false) && bounB4.fell()) {
        pagevar = pagevar + 1;
        if (pagevar >= 6) {
          pagevar = 6;
        }
      }
      break;
    case 5:
      display.setCursor(32, 10);
      display.print(v3PotX_fine);
      display.setCursor(32, 20);
      display.print(v3PotX_invert);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        v3PotX_fine = v3PotX_fine + 1;
        EEPROM.write(6, v3PotX_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        v3PotX_fine = v3PotX_fine - 1;
        EEPROM.write(6, v3PotX_fine);
      }
      if ((bitRead(swdata, SWB3) == false) && bounB3.fell()) {
        v3PotX_invert = !v3PotX_invert;
        EEPROM.write(12, v3PotX_invert);
      }
      if ((bitRead(swdata, SWB2) == false) && bounB2.fell()) {
        pagevar = pagevar - 1;
        if (pagevar <= 1) {
          pagevar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false) && bounB4.fell()) {
        pagevar = pagevar + 1;
        if (pagevar >= 6) {
          pagevar = 6;
        }
      }
      break;
    case 6:
      display.setCursor(32, 10);
      display.print(v4PotX_fine);
      display.setCursor(32, 20);
      display.print(v4PotX_invert);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        v4PotX_fine = v4PotX_fine + 1;
        EEPROM.write(7, v4PotX_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        v4PotX_fine = v4PotX_fine - 1;
        EEPROM.write(7, v4PotX_fine);
      }
      if ((bitRead(swdata, SWB3) == false) && bounB3.fell()) {
        v4PotX_invert = !v4PotX_invert;
        EEPROM.write(13, v4PotX_invert);
      }
      if ((bitRead(swdata, SWB2) == false) && bounB2.fell()) {
        pagevar = pagevar - 1;
        if (pagevar <= 1) {
          pagevar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false)  && bounB4.fell()) {
        pagevar = pagevar + 1;
        if (pagevar >= 6) {
          pagevar = 6;
        }
      }
      break;
    default:
      if (pagevar >= 6) {
        pagevar = 6;
      }
      break;
  }
}

// Servoのトリム調整
void servo_set() {
  static int servovar = 1;

  display.setTextColor(WHITE);
  display.setCursor(32, 0);
  display.print(servovar);  // Servo番号の表示

  switch (servovar) {
    case 1:
      display.setCursor(32, 10);
      display.print(Svo1_fine);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        Svo1_fine = Svo1_fine + 1;
        EEPROM.write(14, Svo1_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        Svo1_fine = Svo1_fine - 1;
        EEPROM.write(14, Svo1_fine);
      }
      if ((bitRead(swdata, SWB2) == false) && bounB2.fell()) {
        servovar = servovar - 1;
        if (servovar <= 1) {
          servovar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false) && bounB4.fell()) {
        servovar = servovar + 1;
        if (servovar >= 8) {
          servovar = 8;
        }
      }
      break;
    case 2:
      display.setCursor(32, 10);
      display.print(Svo2_fine);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        Svo2_fine = Svo2_fine + 1;
        EEPROM.write(15, Svo2_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        Svo2_fine = Svo2_fine - 1;
        EEPROM.write(15, Svo2_fine);
      }
      if ((bitRead(swdata, SWB2) == false) && bounB2.fell()) {
        servovar = servovar - 1;
        if (servovar <= 1) {
          servovar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false) && bounB4.fell()) {
        servovar = servovar + 1;
        if (servovar >= 8) {
          servovar = 8;
        }
      }
      break;
    case 3:
      display.setCursor(32, 10);
      display.print(Svo3_fine);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        Svo3_fine = Svo3_fine + 1;
        EEPROM.write(16, Svo3_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        Svo3_fine = Svo3_fine - 1;
        EEPROM.write(16, Svo3_fine);
      }
      if ((bitRead(swdata, SWB2) == false) && bounB2.fell()) {
        servovar = servovar - 1;
        if (servovar <= 1) {
          servovar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false) && bounB4.fell()) {
        servovar = servovar + 1;
        if (servovar >= 8) {
          servovar = 8;
        }
      }
      break;
    case 4:
      display.setCursor(32, 10);
      display.print(Svo4_fine);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        Svo4_fine = Svo4_fine + 1;
        EEPROM.write(17, Svo4_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        Svo4_fine = Svo4_fine - 1;
        EEPROM.write(17, Svo4_fine);
      }
      if ((bitRead(swdata, SWB2) == false) && bounB2.fell()) {
        servovar = servovar - 1;
        if (servovar <= 1) {
          servovar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false) && bounB4.fell()) {
        servovar = servovar + 1;
        if (servovar >= 8) {
          servovar = 8;
        }
      }
      break;
    case 5:
      display.setCursor(32, 10);
      display.print(Svo5_fine);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        Svo5_fine = Svo5_fine + 1;
        EEPROM.write(18, Svo5_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        Svo5_fine = Svo5_fine - 1;
        EEPROM.write(18, Svo5_fine);
      }
      if ((bitRead(swdata, SWB2) == false) && bounB2.fell()) {
        servovar = servovar - 1;
        if (servovar <= 1) {
          servovar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false) && bounB4.fell()) {
        servovar = servovar + 1;
        if (servovar >= 8) {
          servovar = 8;
        }
      }
      break;
    case 6:
      display.setCursor(32, 10);
      display.print(Svo6_fine);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        Svo6_fine = Svo6_fine + 1;
        EEPROM.write(19, Svo6_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        Svo6_fine = Svo6_fine - 1;
        EEPROM.write(19, Svo6_fine);
      }
      if ((bitRead(swdata, SWB2) == false) && bounB2.fell()) {
        servovar = servovar - 1;
        if (servovar <= 1) {
          servovar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false) && bounB4.fell()) {
        servovar = servovar + 1;
        if (servovar >= 8) {
          servovar = 8;
        }
      }
      break;
    case 7:
      display.setCursor(32, 10);
      display.print(Svo7_fine);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        Svo7_fine = Svo7_fine + 1;
        EEPROM.write(20, Svo7_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        Svo7_fine = Svo7_fine - 1;
        EEPROM.write(20, Svo7_fine);
      }
      if ((bitRead(swdata, SWB2) == false) && bounB2.fell()) {
        servovar = servovar - 1;
        if (servovar <= 1) {
          servovar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false) && bounB4.fell()) {
        servovar = servovar + 1;
        if (servovar >= 8) {
          servovar = 8;
        }
      }
      break;
    case 8:
      display.setCursor(32, 10);
      display.print(Svo8_fine);
      if ((bitRead(swdata, SWB1) == false) && bounB1.fell()) {
        Svo8_fine = Svo8_fine + 1;
        EEPROM.write(21, Svo8_fine);
      }
      if ((bitRead(swdata, SWB5) == false) && bounB5.fell()) {
        Svo8_fine = Svo8_fine - 1;
        EEPROM.write(21, Svo8_fine);
      }
      if ((bitRead(swdata, SWB2) == false) && bounB2.fell()) {
        servovar = servovar - 1;
        if (servovar <= 1) {
          servovar = 1;
        }
      }
      if ((bitRead(swdata, SWB4) == false) && bounB4.fell()) {
        servovar = servovar + 1;
        if (servovar >= 8) {
          servovar = 8;
        }
      }
      break;
    default:
      if (servovar >= 8) {
        servovar = 8;
      }
      break;
  }
}
