#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <AsyncUDP.h>
#include "m2006_twai.h"
#include "as5600_tca9548a.h"
#include "esp_intr_alloc.h"
#include <Adafruit_NeoPixel.h>

#define MAX_AMPERE 3000
#define MAX_ROTATE_RPM 150
#define LED_BUILTIN GPIO_NUM_2
#define CW                                  true
#define CCW                                 false

#define SDA_PIN GPIO_NUM_5
#define SCL_PIN GPIO_NUM_6
#define LED_PIN GPIO_NUM_1

#define NUMPIXELS 27

Adafruit_NeoPixel LEDArray(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// WiFiを使う時はコメントアウト(ロボプロのルーター)
#define WiFi_ON

// 僕の家のWiFi
// #define HOUSE_WiFi_MODE
const char* ssid = "alvin";
const char* password = "bobalvin";

CANWrapper canWrapper;

AsyncUDP udp;

TaskHandle_t thp[3];

int8_t send_current_data1[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int8_t send_current_data2[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 使うのは6個目まで
int16_t raw_current_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int16_t test_raw_current_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

float target_speed[3] = {0, 0, 0}; // 目標速度
float target_angle[3] = {0, 0, 0}; // 目標角度
float past_target_angle[3] = {0, 0, 0};
float rotate_rpm[3] = {0, 0, 0};
float target_rpm[6] = {0, 0, 0, 0, 0, 0}; // 目標回転数（出力）
float i = 0.0;

// PIDの計算用
float prev_time = 0.0;
float dt = 0.0;

// PID制御(速度制御)
float error[6];
float integral[6];
float derivative[6];
float pre_error[6];
// 相対誤差
float error_relative[6]; 

// PID制御(角度制御)
float error_angle[3];
float integral_angle[3];
float derivative_angle[3];
float pre_error_angle[3];

float Kp_angle = 1; 
float Ki_angle = 0; 
float Kd_angle = 0;

// 3倍はやくする（angleよりも速くする）
float Kp = Kp_angle * 3;
float Ki = Ki_angle * 3;
float Kd = Kd_angle * 3;

// Differential Swerve Drive
float w_w[3] = {0, 0, 0}; // 移動方向の速度
float w_a[3] = {0, 0, 0}; // 方位方向の速度
float d_a[3] = {0, 0, 0}; // 方位方向の角度

// AS5600_TCA9548A
float offset1[3] = {0.0, 0.0, 0.0}; // 静止時のオフセット値(3個分)
float offset2[3] = {0.0, 0.0, 0.0}; // 回転時のオフセット値(3個分)
float as5600_angle[3] = {0.0, 0.0, 0.0}; // AS5600の現在の角度(3個分) (0 to 360)
float current_angle[3] = {0.0, 0.0, 0.0}; // 現在の角度(3個分) (0 to 360)

void Core0a(void *args);
void Core0b(void *args);
void Core1b(void *args);

// 入力
float in_V = 0.0;
float in_V_direction = 0.0;
float in_R = 0.0;

float V = 0.0; // 移動速度[rpm]
float V_direction = 0.0; // 移動方向[rad]
float R = 0.0; // 回転速度[rpm]

float Vx, Vy = 0.0;
float Vx_list[3] = {0.0, 0.0, 0.0};
float Vy_list[3] = {0.0, 0.0, 0.0};
float wheel_spacing = 2 * PI / 3;
void litUpWheel(uint8_t R, uint8_t G, uint8_t B, bool Dir, uint16_t interval){
  LEDArray.clear();
  if(Dir){
    for(int i=0;i<=NUMPIXELS;i++){
      for(int j=0;j<=i;j++){
      LEDArray.setPixelColor(j,LEDArray.Color(R,G,B));
      LEDArray.show();
      }
      delay(interval/NUMPIXELS);
    }
  }
  else {
    for(int i=NUMPIXELS;i>=0;i--){
      for(int j=NUMPIXELS;j>=i;j--){
      LEDArray.setPixelColor(j,LEDArray.Color(R,G,B));
      LEDArray.show();
      }
      delay(interval/NUMPIXELS);
    }
  }
}

void litUpAllLED(uint8_t R, uint8_t G, uint8_t B){
  LEDArray.clear();
  for(int i=0;i<NUMPIXELS;i++){
      LEDArray.setPixelColor(i,LEDArray.Color(R,G,B));
    }
  LEDArray.show();
}


void make_current_data(int16_t current_data_in[8], int8_t current_data_out1[8], int8_t current_data_out2[8]) {
  // 0x200用のデータを作成（ID1-4）
  for (int i = 0; i < 4; i++) {
    current_data_out1[0 + (i * 2)] = (current_data_in[i] >> 8) & 0xFF;
    current_data_out1[1 + (i * 2)] = current_data_in[i] & 0xFF;
  }
  // 0x1FF用のデータを作成（ID5-8）
  for (int i = 0; i < 4; i++) {
    current_data_out2[0 + (i * 2)] = (current_data_in[i + 4] >> 8) & 0xFF;
    current_data_out2[1 + (i * 2)] = current_data_in[i + 4] & 0xFF;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // I2Cのピンをプルアップ
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  LEDArray.begin();

  // CAN
  canWrapper.begin();

  #ifdef WiFi_ON
  // WiFI
  litUpWheel(255, 0, 0, CCW, 2000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.print(".");
    litUpWheel(255, 0, 0, CW, 500);
  }
  Serial.println("WiFi Ready");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());
  litUpAllLED(0, 255, 0);

      // Receive
    // udp.onPacket([](AsyncUDPPacket packet) { // packetは，受け取ったデータ
    //   // packet.data()をスペース区切りで分割
    //   // [V]_[V_direction]_[R]

    //   String str = String((char*)packet.data());
    //   in_V = str.substring(0, str.indexOf(" ")).toFloat();

    //   str = str.substring(str.indexOf(" ") + 1);
    //   in_V_direction = str.substring(0, str.indexOf(" ")).toFloat();

    //   str = str.substring(str.indexOf(" ") + 1);
    //   in_R = str.substring(0, str.indexOf(" ")).toFloat();
    // });

    if (udp.listen(12345)) {
    udp.onPacket([](AsyncUDPPacket packet) {
      // Print the received packet data as a string
      Serial.print("Packet received from ");
      Serial.print(packet.remoteIP());
      Serial.print(": ");
      
      // Ensure packet data is null-terminated
      char* data = (char*)malloc(packet.length() + 1);
      memcpy(data, packet.data(), packet.length());
      data[packet.length()] = '\0';

      // Print the data
      Serial.println(data);

      // Example of parsing the data
      // If data is comma-separated like "x_val,y_val,degree"
      String dataStr(data);
      int firstComma = dataStr.indexOf(',');
      int secondComma = dataStr.indexOf(',', firstComma + 1);

      String xValStr = dataStr.substring(0, firstComma);
      String yValStr = dataStr.substring(firstComma + 1, secondComma);
      String degreeStr = dataStr.substring(secondComma + 1);

      // Convert the string values to float
      in_V = xValStr.toFloat();
      in_V_direction = yValStr.toFloat()*180/PI;
      in_R = degreeStr.toFloat();

      Serial.print("V Value: "); Serial.println(in_V);
      Serial.print("R Value: "); Serial.println(in_R);
      Serial.print("Dir Value: "); Serial.println(in_V_direction);

      free(data);
    });
  }
  
  #endif

  // AS5600_TCA9548A
  as5600_tca9548a_init();
  Serial.println("AS5600_TCA9548A: Started.");
  as5600_tca9548a_get_offset(offset1);
  Serial.println("AS5600_TCA9548A: Offset1: " + String(offset1[0]) + ", " + String(offset1[1]) + ", " + String(offset1[2]));

  // 横向きにタイヤを揃える. x軸を0度とする．
  offset1[0] = 12.22; // 右前
  offset1[1] = 26.89; // 左前
  offset1[2] = 109.42; // 後
  // 反時計回り正（-180 ~ 180)

  xTaskCreatePinnedToCore(Core0a, "Core0a", 4096, NULL, 1, &thp[0], 0);
  xTaskCreatePinnedToCore(Core0b, "Core0b", 4096 * 2, NULL, 1, &thp[1], 0); // 引数(関数, タスク名, スタックサイズ, 引数, 優先度, タスクハンドル, コア番号)
  xTaskCreatePinnedToCore(Core1b, "Core1b", 4096, NULL, 1, &thp[2], 1);
}

void loop() {
  
  V = (in_V * 30) / (PI * 0.035); // [rpm]
  V_direction = in_V_direction * (PI / 180.0); // [rad]
  R = (in_R * 30) / (PI * 0.5); // [rpm]

  // V_direction 入力は正面が0度になるから90度足す．
  // 本来の座標系は，右が0度，反時計回りに増加する．
  Vx = V * cos(V_direction); 
  Vy = V * sin(V_direction); 

  // 3つのタイヤの速度を計算
  // 横軸が0度のとき，タイヤの角度は, 30, 150, 270度. 1,2,3
  for (int i = 0; i < 3; i++){
    Vx_list[i] = Vx + 0.5 * cos((PI / 6.0) + wheel_spacing * i + PI / 2.0) * R;
    Vy_list[i] = Vy + 0.5 * sin((PI / 6.0) + wheel_spacing * i + PI / 2.0) * R;
  }

  // ここの＋，－は，タイヤの向きによって変わる．
  for (int i = 0; i < 3; i++){
    target_speed[i] = (sqrt(pow(Vx_list[i], 2) + pow(Vy_list[i], 2)));
    target_angle[i] = (atan2(Vy_list[i], Vx_list[i]) * 180.0 / PI);
    target_angle[i] = constrain(target_angle[i], -180, 180);
  } 

  // 経過時間の計算
  dt = millis() - prev_time; // ms
  prev_time = millis();

  // Diffencial Swerve Driveの式
  for (int i = 0; i < 3; i++){
    w_w[i] = ((m_rpm[0 + (i * 2)] + m_rpm[1 + (i * 2)]) / 2.0); // 移動方向の速度 
    w_a[i] = ((m_rpm[0 + (i * 2)] - m_rpm[1 + (i * 2)]) / 2.0); // 方位方向の速度 Yaw
    d_a[i] += (float)(w_a[i] * dt / 1000.0) / 1.3;
    if (d_a[i] > 360) d_a[i] -= 360;
    else if (d_a[i] < 0) d_a[i] += 360;
    current_angle[i] = as5600_angle[i];
  }

  // 問題はas5600の精度が低いことによって正負が遅くなる

  for (int i = 0; i < 3; i++){
    double angle_diff = target_angle[i] - current_angle[i];
    if (angle_diff < -270.0) {
      target_angle[i] += 360;
    } else if (angle_diff > 270.0) {
      target_angle[i] -= 360;
    }
    if (target_angle[i] - current_angle[i] < -90) { // 180度以上の差がある場合
      target_angle[i] += 180;
      target_speed[i] = -target_speed[i];
    } else if (target_angle[i] - current_angle[i] > 90) { // 180度以上の差がある場合
      target_angle[i] -= 180;
      target_speed[i] = -target_speed[i];
    } 
  }

  // PID制御(角度制御)
  for (int i = 0; i < 3; i++){
    error_angle[i] = target_angle[i] - current_angle[i]; 
    integral_angle[i] += error_angle[i] * dt; // 積分
    derivative_angle[i] = (error_angle[i] - pre_error_angle[i]) / dt; // 微分
    pre_error_angle[i] = error_angle[i]; // 前回の誤差を保存
    if (target_angle[i] == 0) integral_angle[i] = 0; // 目標値が0のときは積分をリセット
  }

  // 回転速度の計算
  for (int i = 0; i < 3; i++){
    rotate_rpm[i] = (Kp_angle * error_angle[i] + Ki_angle * integral_angle[i] + Kd_angle * derivative_angle[i]);
    rotate_rpm[i] = constrain(rotate_rpm[i], -MAX_ROTATE_RPM, MAX_ROTATE_RPM); // 回転速度の制限
  }

  // 目標回転数の計算(Total)
  for (int i = 0; i < 3; i++){
    target_rpm[0 + (i * 2)] = target_speed[i] - rotate_rpm[i];
    target_rpm[1 + (i * 2)] = target_speed[i] + rotate_rpm[i];
  }

  // PID制御(速度制御)
  for (int i = 0; i < 6; i++){
    error[i] = target_rpm[i] - m_rpm[i];
    integral[i] += error[i] * dt; // 積分
    derivative[i] = (error[i] - pre_error[i]) / dt; // 微分
    pre_error[i] = error[i]; // 前回の誤差を保存
    if (target_rpm[i] == 0) integral[i] = 0; // 目標値が0のときは積分をリセット
  }

  // 電流値の計算(-3000 ~ 3000)
  for (int i = 0; i < 3; i++){
    raw_current_data[0 + (i * 2)] = Kp * error[0 + (i * 2)] + Ki * integral[0 + (i * 2)] + Kd * derivative[0 + (i * 2)];
    raw_current_data[1 + (i * 2)] = Kp * error[1 + (i * 2)] + Ki * integral[1 + (i * 2)] + Kd * derivative[1 + (i * 2)];

    raw_current_data[0 + (i * 2)] = constrain(raw_current_data[0 + (i * 2)], -MAX_AMPERE, MAX_AMPERE); // -3000 ~ 3000
    raw_current_data[1 + (i * 2)] = constrain(raw_current_data[1 + (i * 2)], -MAX_AMPERE, MAX_AMPERE); // -3000 ~ 3000
  }

  make_current_data(raw_current_data, send_current_data1, send_current_data2);
  canWrapper.sendMessage(0x200, send_current_data1);
  canWrapper.sendMessage(0x1FF, send_current_data2);


  delay(10);
}

void Core1b(void *args) {
   while (1){
  //   // Serial 入力
  //   if (Serial.available() > 0) {
  //     // スペース区切りで分割
  //     in_V = Serial.parseFloat(); // 移動速度[m/s]
  //     in_V = constrain(in_V, -1, 1);
  //     in_V_direction = Serial.parseFloat(); // 移動方向[degrees]
  //     in_V_direction = constrain(in_V_direction, 0, 360);
  //     in_R = Serial.parseFloat(); // 回転速度[m/s]
  //     in_R = constrain(in_R, -1, 1);
  //   }
     delay(10);
     }
}

void Core0a(void *args) {
  while (1) {
    canWrapper.update();
    as5600_tca9548a_get_current_angle(as5600_angle, offset1, offset2);
    delay(1);
  }
}

// Serial Ploting
void Core0b(void *args) {
  while (1) {
    // for teleplot
    for (int i = 0; i < 3; i++){
    //   String str = ">target_speed" + String(i) + ": " + String(target_speed[i]) + "\n";
    //   Serial.println(str);
    //   str = ">target_angle" + String(i) + ": " + String(target_angle[i]) + "\n";
    //   Serial.println(str);
      // str = ">current_angle" + String(i) + ": " + String(current_angle[i]) + "\n";
      // Serial.println(str);
    //   str = ">d_a" + String(i) + ": " + String(d_a[i]) + "\n";
    //   Serial.println(str);
    //   String str = ">as5600_angle" + String(i) + ": " + String(as5600_angle[i]) + "\n";
    // Serial.println(str);
     }

    // for (int i = 0; i < 6; i++){
    //   String str = ">target_rpm" + String(i) + ": " + String(target_rpm[i]) + "\n";
    //   Serial.println(str);
    //   str = ">raw_current_data" + String(i) + ": " + String(raw_current_data[i]) + "\n";
    //   Serial.println(str);
    //   str = ">m_rpm" + String(i) + ": " + String(m_rpm[i]) + "\n";
    //   Serial.println(str);
    //   str = ">m_torque" + String(i) + ": " + String(m_torque[i]) + "\n";
    //   Serial.println(str);
    // }
    delay(100);
  }
}