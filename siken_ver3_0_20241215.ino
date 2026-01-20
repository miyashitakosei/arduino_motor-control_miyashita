//2026年1月20日追記
//学部の卒業研究で使用したプログラムです。

#include <SPI.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>

// モータ1のピン設定
#define CW_PIN1 7
#define CCW_PIN1 6
#define STOP_MODE_PIN1 5
#define SPD_SET_PIN1 4

// モータ2のピン設定
#define CW_PIN2 13
#define CCW_PIN2 12
#define STOP_MODE_PIN2 11
#define SPD_SET_PIN2 10

// MCP4922の設定
#define LDAC 22    // MCP4922のラッチ動作出力ピン
#define CS_PIN 53  // MCP4922のチップセレクトピン
#define VREF 5.0   // 基準電圧（5V）

// マルチプレクサTCA9548AのI2Cアドレス
#define TCA9548A_ADDR 0x70

// エンコーダのピン設定
#define encoderPinA1 3
#define encoderPinB1 2
#define encoderPinA2 18
#define encoderPinB2 19

#define DEBUG_MODE 1  //1でON

// エンコーダ関連
volatile int encoderPos1 = 0;
volatile int encoderPos2 = 0;
const int pulsesPerRevolution = 2048;  // 1回転あたりのパルス数
float currentAngle1 = 0, currentAngle2 = 0;
float rpm1 = 0, rpm2 = 0;
unsigned long prevTime1 = 0, prevTime2 = 0;
unsigned long timeElapsed1 = 0, timeElapsed2 = 0;
int prevEncoderPos1 = 0, prevEncoderPos2 = 0;

// 距離センサの設定
VL53L0X sensor1;
VL53L0X sensor2;

// 固定値
float ZaiKata = 1000; //座位の肩までの距離
float Lt = 71.2;  //転子高さ
float ZaiKataTensi = (float)ZaiKata - (float)Lt; 
float Lb = float(ZaiKataTensi) * 0.61;    // 重心までの距離 (mm)

float Lc = 425;   //椅子の高さ
float Ls = 110;  // センサ間の垂直距離 (mm)
float Lct = (float)Lc + (float)Lt; //椅子高さと転子高さの合計
float Lm = 635;   //地面から装置センサ２までの距離
float Lh; //重心位置計算のため
float Lg = 250;   //装置センサ位置から理想重心位置までの距離
float angle_target = 46.5;

//目標２のために追加
float L1, L2;
float y1, y2, y3, y4, y5, y6, y7, y8, y9;
float s1, s2, s3, s4, s5;
float Lw = 63;
float Lk = 140;   //腰当長さ
float Lp = 100;   //200-71.2 腸骨稜から転子点を引く 骨盤部分長さ100
unsigned long L1z;
unsigned long y2z;
unsigned long Lkz;
float s1d;

float ka_p;
float ka_i;
float kp_p;
float kp_i;

// グローバル変数としてセンサ値を宣言
uint16_t distance1 = 0;
uint16_t distance2 = 0;

float distance_diff = 0.0;  //初期値

//　ワイヤ長さ初期値
float wireLength1 = 0.0;
float wireLength2 = 0.0;

// モータ制御変数
float angle_formula, angle_measured;
float position_formula, position_measured;
float angle_difference, position_difference;
float duty_top, duty_under;
int count = 1;
unsigned long lastMeasureTime = 0;  // 最後に測定を行った時間
bool motorRunning = false;
unsigned long distanceReadTime = 0;
unsigned long currentTime = 0;
unsigned long previousLoopTime = 0; // 各ループの実行時間を管理
float x3, x4, x5;
float angle_dash;
float bunsi;
float bunbo;

//1129追加分
float Kangle_measured;
float y1d;
float x1d, x2d;
float pasent;

// MCP4922初期化
void setupDAC() {
  pinMode(LDAC, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  SPI.begin();  // SPI通信の初期化
  SPI.setClockDivider(SPI_CLOCK_DIV8);
}

// MCP4922でアナログ電圧をモータドライバのSPD_SET_PINに設定（最大5.0V）
void setMotorSpeedDAC(float voltage, int motor) {
  // 電圧を最大5.0Vに制限（30RPM対応）
  if (voltage > 5.0) {
    voltage = 5.0;  // 電圧を5.0Vに制限
  }

  // 電圧を12ビットのデジタル値に変換（0～4095）
  int digitalValue = (int)((voltage / VREF) * 4095);
  digitalWrite(CS_PIN, LOW);
  if (motor == 1) {
    SPI.transfer(0b00110000 | ((digitalValue >> 8) & 0x0F));  // VOUTAをモータ1に
  } else {
    SPI.transfer(0b10110000 | ((digitalValue >> 8) & 0x0F));  // VOUTBをモータ2に
  }
  SPI.transfer(digitalValue & 0xFF);
  digitalWrite(CS_PIN, HIGH);
  
  // ラッチ
  digitalWrite(LDAC, LOW);
  delayMicroseconds(1);
  digitalWrite(LDAC, HIGH);
}

// TCA9548AでI2Cチャネルを選択
void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// センサの初期化
void setupSensor() {
  // センサ1の初期化
  tcaSelect(0);  // マルチプレクサのチャネル0（センサ1）
  sensor1.init();
  sensor1.setTimeout(500);
  sensor1.setMeasurementTimingBudget(100000);  // 100msごとに測定
  sensor1.startContinuous();

  // センサ2の初期化
  tcaSelect(1);  // マルチプレクサのチャネル1（センサ2）
  sensor2.init();
  sensor2.setTimeout(500);
  sensor2.setMeasurementTimingBudget(100000);  // 100msごとに測定
  sensor2.startContinuous();
}

// エンコーダ1の値を読み取る関数
void readEncoder1() {
  static int lastA1 = LOW;
  int currentA1 = digitalRead(encoderPinA1);
  int currentB1 = digitalRead(encoderPinB1);

  if (lastA1 == LOW && currentA1 == HIGH) {
    if (currentB1 == LOW) {
      encoderPos1++;
    } else {
      encoderPos1--;
    }
  }
  lastA1 = currentA1;
}

// エンコーダ2の値を読み取る関数
void readEncoder2() {
  static int lastA2 = LOW;
  int currentA2 = digitalRead(encoderPinA2);
  int currentB2 = digitalRead(encoderPinB2);

  if (lastA2 == LOW && currentA2 == HIGH) {
    if (currentB2 == LOW) {
      encoderPos2++;
    } else {
      encoderPos2--;
    }
  }
  lastA2 = currentA2;
}

// モータを停止する関数
void stopMotors() {
  digitalWrite(CW_PIN1, LOW);  // モータ1の回転を停止
  digitalWrite(CCW_PIN1, LOW);
  digitalWrite(STOP_MODE_PIN1, HIGH);  // 停止信号を送る

  digitalWrite(CW_PIN2, LOW);  // モータ2の回転を停止
  digitalWrite(CCW_PIN2, LOW);
  digitalWrite(STOP_MODE_PIN2, HIGH);  // 停止信号を送る

  // モータの速度設定ピン（SPD_SET_PIN）は0Vに設定
  setMotorSpeedDAC(0, 1);  // モータ1の回転速度を0に
  setMotorSpeedDAC(0, 2);  // モータ2の回転速度を0に
}

void stopBothMotors() {
  stopMotors();  // モータ1とモータ2を同時に停止
}

// 状態を記録するための変数を追加1128
unsigned long withinErrorStartTime = 0;  // 誤差が許容範囲に入った時の開始時間
bool isWithinError = false;             // 誤差が許容範囲に収まっているかのフラグ

void setup() {
  Serial.begin(115200);  // シリアル通信ボーレートを74880に設定

  // SPIとI2Cの初期化
  setupDAC();   // MCP4922の初期化
  Wire.begin();  // I2Cの初期化

  // TCA9548Aとセンサの初期化
  setupSensor();  // 100ms間隔で測定

  pinMode(CW_PIN1, OUTPUT);
  pinMode(CCW_PIN1, OUTPUT);
  pinMode(STOP_MODE_PIN1, OUTPUT);
  pinMode(SPD_SET_PIN1, OUTPUT);

  pinMode(CW_PIN2, OUTPUT);
  pinMode(CCW_PIN2, OUTPUT);
  pinMode(STOP_MODE_PIN2, OUTPUT);
  pinMode(SPD_SET_PIN2, OUTPUT);

  // エンコーダピンの設定
  pinMode(encoderPinA1, INPUT_PULLUP);
  pinMode(encoderPinB1, INPUT_PULLUP);
  pinMode(encoderPinA2, INPUT_PULLUP);
  pinMode(encoderPinB2, INPUT_PULLUP);

  // 割り込みを使ってエンコーダの値を読み取る
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), readEncoder2, CHANGE);

  Serial.println("Enter 'start' to begin motor control:");
}

void loop() {
  if (!motorRunning) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command == "start") {
        Serial.println("Motor control will start in 10 seconds...");

        // センサ1とセンサ2の初期値を読み取り、表示
        tcaSelect(0);
        distance1 = sensor1.readRangeContinuousMillimeters();
        tcaSelect(1);
        distance2 = sensor2.readRangeContinuousMillimeters();

        // 測距センサの初期値を表示
        Serial.print("Start command received. Initial Sensor1 Distance: ");
        Serial.print(distance2);
        Serial.print(" mm, Initial Sensor2 Distance: ");
        Serial.println(distance1);

        delay(10000);  // 10秒待機
        motorRunning = true;
        lastMeasureTime = millis();
        previousLoopTime = millis();  // ループ開始時間を初期化
      }
    }
    return;
  } else {
    currentTime = millis();

    // 100msごとにループを実行
    if (currentTime - previousLoopTime >= 100) {
      previousLoopTime = currentTime;

      if(Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if(command == "stop") {
          //モータ停止
          stopMotors();
          Serial.println("装置コマンド停止");
          motorRunning = false;
          return;
        }
      }

      // センサ1とセンサ2の読み取り
      tcaSelect(0);
      distance1 = sensor1.readRangeContinuousMillimeters();
      tcaSelect(1);
      distance2 = sensor2.readRangeContinuousMillimeters();

      // エンコーダの値を読み取る
      timeElapsed1 = currentTime - prevTime1;
      timeElapsed2 = currentTime - prevTime2;

      if (timeElapsed1 >= 100) {    //エンコーダ１
        int deltaPos1 = encoderPos1 - prevEncoderPos1;
        prevEncoderPos1 = encoderPos1;
        currentAngle1 = (encoderPos1 / (float)pulsesPerRevolution) * 360.0;
        rpm1 = (deltaPos1 / (float)pulsesPerRevolution) * 60.0 * (1000.0 / timeElapsed1);
        prevTime1 = currentTime;
      }

      if (timeElapsed2 >= 100) {    //エンコーダ２
        int deltaPos2 = encoderPos2 - prevEncoderPos2;
        prevEncoderPos2 = encoderPos2;
        currentAngle2 = (encoderPos2 / (float)pulsesPerRevolution) * 360.0;
        rpm2 = (deltaPos2 / (float)pulsesPerRevolution) * 60.0 * (1000.0 / timeElapsed2);
        prevTime2 = currentTime;
      }

      //ワイヤ長さ計測1
      float angle_abs1 = abs(currentAngle1);  //エンコーダ値の絶対値
      if (angle_abs1 <= 360) {    //360度以下
        wireLength1 = 135.09*(angle_abs1 / 360);  //135.09
        }
        else if (360 < angle_abs1 && angle_abs1 <= 720){
        wireLength1 = 135.09+(141.37*((angle_abs1-360)/360));   //144.51
        }
        else {
        wireLength1 = 135.09+141.37+(147.65*((angle_abs1-720)/360));    //150.79
      }

      //ワイヤ長さ計測2
      float angle_abs2 = abs(currentAngle2);  //エンコーダ値の絶対値
      if (angle_abs2 <= 360) {    //360度以下
        wireLength2 = 135.09*(angle_abs2 / 360);
        }
        else if (360 < angle_abs2 && angle_abs2 <= 720){
        wireLength2 = 135.09+(141.37*((angle_abs2-360)/360));
        }
        else {
        wireLength2 = 135.09+141.37+(147.65*((angle_abs2-720)/360));
      }

      // 理想角度と理想位置
      angle_formula = 46.5;
      position_formula = 200;

      //　角度計算
      float x1 = (float)distance2;
      float x2 = (float)distance1;

      //各要素計算
      L1 = 830 - wireLength1;
      L2 = 790 - wireLength2;

      pasent = (wireLength1 / 830);
      pasent = 1 - pasent;
      Lh = 135 * pasent;

      //前傾角度計算
      s2 = acos(Lh / L2); //Θ２のラジアン
      y1 = Lh * tan(s2);
      s3 = atan((Lh + Lw)/y1);
      y2 = y1 / cos(s3);
      s4 = acos(((y2*y2)+(Lk*Lk)-(L1*L1))/(2*y2*Lk));
      s3 = degrees(s3);
      s4 = degrees(s4);
      if (isnan(s4)){
        Kangle_measured = s3 + 2;
      }
      else {
        Kangle_measured = s3 + s4;
      }

      //体の厚み考慮
      y1d = y1 - (140);

      //骨前傾角度から上半身前傾点の位置を計算
      if (Kangle_measured > 90){
        s1d = 180 - Kangle_measured;
        s1d = s1d * (PI / 180);
        y3 = Lp * cos(s1d);
        y4 = Lp * sin(s1d);
        y5 = y1d + y3;
      } else if (Kangle_measured < 90){
        s1d = Kangle_measured * (PI / 180);
        y3 = Lp * cos(s1d);
        y4 = Lp * sin(s1d);
        y5 = y1d - y3;
      } else{
        s1d = Kangle_measured * (PI / 180);
        y4 = Lp;
        y5 = y1d;
      }

      //体の厚み考慮
      x1d = x1 + (110);
      x2d = x2 + (193/2);

      //y4は高さ、y5は装置からの上半身前傾点までの距離
      if (x1d < y5){
        y6 = Lm + Ls - Lc - Lt - y4;
        y7 = y5 - x1d;
        s5 = atan(y6 / y7);
        y8 = Lb - Lp;
        y9 = y8 * cos(s5);
        position_measured = y5 - y9;
      } else if (x1d > y5){
        y6 = Lm + Ls - Lc - Lt - y4;
        y7 = x1d - y5;
        s5 = atan(y6 / y7);
        y8 = Lb - Lp;
        y9 = y8 * cos(s5);
        position_measured = y5 + y9;
      } else{
        position_measured = y5;
      }

      angle_measured = degrees(s5); //体幹前傾角度s5を前傾角度に変更

      // 角度と位置の差分計算
      angle_difference = angle_target - angle_measured;
      position_difference = (float)Lg - position_measured;

      //停止条件の確認
      bool angleWithinError = abs(angle_difference) <= (angle_target * 0.18);
      bool positionWithinError = abs(position_difference) <= (Lg * 0.14);

      //計算角度が指定未満or計算位置が指定未満
      bool angleBelow = angle_measured < 20.0;
      bool positionBelow = position_measured < 215.0;

      if(angleWithinError && positionWithinError) {
        if (!isWithinError) {
          //誤差範囲内に入った瞬間にタイマーをリセット
          withinErrorStartTime = currentTime;
          isWithinError = true;
        } else if(currentTime - withinErrorStartTime >= 1000) {
          //指定時間以上誤差が指定％以内に収まっている場合、モータ停止
          stopMotors();
          Serial.println("装置停止");
          motorRunning = false;
        }
      } else {
        //想定範囲外の場合はタイマーをリセット
        isWithinError = false;
      }

      //追加の未満条件
      if (angleBelow || positionBelow) {
        stopMotors();
        Serial.print("計算値未満検知:");
        if (angleBelow) Serial.print("角度未満");
        if (positionBelow) Serial.print("位置未満");
        Serial.println();
        motorRunning = false;
      }

      //PI制御ゲイン
      ka_p = ;  //角度の比例ゲイン
      ka_i = ;  //角度の積分ゲイン
      kp_p = ;  //位置の比例ゲイン
      kp_i = ;  //位置の積分ゲイン

      //積分成分
      static float angle_integral = 0;
      static float position_integral = 0;

      angle_integral += angle_difference;
      position_integral += position_difference;

      //出力計算(1215時点)
      float duty_top = kp_p * fabs(position_difference) + kp_i * fabs(position_integral) + ka_p * fabs(angle_difference) + ka_i * fabs(angle_integral);
      float duty_under = kp_p * fabs(position_difference) + kp_i * fabs(position_integral) - ka_p * fabs(angle_difference) + ka_i * fabs(angle_integral);

      //duty_cycleの制限　0～1に収める
      duty_top = constrain(duty_top, 0.0, 30.0);
      duty_under = constrain(duty_under, 0.0, 30.0);

      //duty_cycleから電圧へ変換
      float voltageTop = (duty_top / 30.0) * 5.0;
      float voltageUnder = (duty_under / 30.0) * 5.0;

      // モータ制御処理
      if (!motorRunning) return;

      setMotorSpeedDAC(voltageTop, 1);
      setMotorSpeedDAC(voltageUnder, 2);

      // モータの回転方向設定（モータ2の回転方向を逆）0
      if (angle_difference > 0) {
        digitalWrite(CW_PIN1, LOW);
        digitalWrite(CCW_PIN1, HIGH);
      } else {
        digitalWrite(CW_PIN1, HIGH);
        digitalWrite(CCW_PIN1, LOW);
      }

      if (position_difference > 0) {
        digitalWrite(CW_PIN2, HIGH);
        digitalWrite(CCW_PIN2, LOW);
      } else {
        digitalWrite(CW_PIN2, LOW);
        digitalWrite(CCW_PIN2, HIGH);
      }

      // シリアルモニタに表示（見やすく名称を変更）
      //Serial.print("Lh: ");
      //Serial.print(Lh);
      //Serial.print("L2: ");
      //Serial.print(L2);
      //Serial.print("L1: ");
      //Serial.print(L1);
      //Serial.print("y2z + Lkz - L1z: ");
      //Serial.println(y2z + Lkz - L1z);
      //Serial.print("2 * y2 * Lk: ");
      //Serial.println(2 * y2 * Lk);
      //Serial.print("acos argument: ");
      //Serial.println((y2z + Lkz - L1z) / (2 * y2 * Lk));
      Serial.print("x1: ");
      Serial.print(distance2);
      Serial.print(" mm, x2: ");
      Serial.print(distance1);
      Serial.print(" mm, 理想角度: ");
      Serial.print(angle_target);
      Serial.print(" deg, 計算角度: ");
      Serial.print(angle_measured);
      Serial.print(" deg, 理想位置: ");
      Serial.print(Lg);
      Serial.print(" mm, 計算位置: ");
      Serial.print(position_measured);
      Serial.print(" mm, Angle_diff: ");
      Serial.print(angle_difference);
      Serial.print(" mm, position_diff: ");
      Serial.print(position_difference);

      Serial.print(" mm, duty top: ");
      Serial.print(duty_top);
      Serial.print(" , duty_under: ");
      Serial.print(duty_under);


      if(DEBUG_MODE){
      Serial.print(" mm, Lh: ");
      Serial.print(Lh);
      Serial.print(" mm, s2: ");
      Serial.print(s2);
      Serial.print(" °, s3: ");
      Serial.print(s3);
      Serial.print(" °, s4: ");
      Serial.print(s4);
      Serial.print(" °, s5: ");
      Serial.print(s5);
      Serial.print(" °, 骨盤角度: ");
      Serial.print(Kangle_measured);
      Serial.print(" °, y1: ");
      Serial.print(y1);
      Serial.print(" mm, y1d: ");
      Serial.print(y1d);
      Serial.print(" mm, y2: ");
      Serial.print(y2);
      Serial.print(" mm, y3: ");
      Serial.print(y3);
      Serial.print(" mm, y4: ");
      Serial.print(y4);
      Serial.print(" mm, y5: ");
      Serial.print(y5);
      Serial.print(" mm, y6: ");
      Serial.print(y6);
      Serial.print(" mm, y7: ");
      Serial.print(y7);
      Serial.print(" mm, y8: ");
      Serial.print(y8);
      Serial.print(" mm, y9: ");
      Serial.print(y9);
      Serial.print(" mm, x1: ");
      Serial.print(x1d);

      Serial.print(" mm, ワイヤ１: ");
      Serial.print(wireLength1);
      Serial.print(" mm, ワイヤ２: ");
      Serial.print(wireLength2);

      Serial.print(" mm, M1指令: ");
      Serial.print(voltageTop );  // 30RPMに対応
      Serial.print(" RPM, M2指令: ");
      Serial.print(voltageUnder);
      Serial.print(" RPM, M1角度: ");
      Serial.print(currentAngle1);
      Serial.print(" deg, M1速度: ");
      Serial.print(rpm1);
      Serial.print(" RPM, M2角度: ");
      Serial.print(currentAngle2);
      Serial.print(" deg, M2速度: ");
      Serial.println(rpm2);
      }

      // ループカウンタの更新
      count++;
    }
  }
}

