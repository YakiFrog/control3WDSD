#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include "m2006_twai.h"
#include "as5600_tca9548a.h"
#include "esp_intr_alloc.h"
#include <Adafruit_NeoPixel.h>

#define MAX_AMPERE 3000
#define MAX_ROTATE_RPM 150
#define LED_BUILTIN GPIO_NUM_2

// I2C
#define SDA_PIN GPIO_NUM_5
#define SCL_PIN GPIO_NUM_6

// LED
#define LED_PIN GPIO_NUM_1
#define NUMPIXELS 27

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

// #define WiFi_ON
// #define HOUSE_WiFi_MODE

#ifndef HOUSE_WiFi_MODE
const char* ssid = "Buffalo-G-8360";
const char* password = "tn3krc7dabknc";
IPAddress ipAddress(192, 168, 11, 2); // Server側のIPアドレス
uint16_t port = 8000; // ポート番号
#else
const char* ssid = "Buffalo-G-2EC8";
const char* password = "exkfnthn7x7k5";
IPAddress ipAddress(192, 168, 11, 60); // Server側のIPアドレス
uint16_t port = 8000; // ポート番号
#endif

CANWrapper canWrapper;

AsyncUDP udp;

TaskHandle_t thp[3];

int8_t send_current_data1[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int8_t send_current_data2[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 使うのは6個目まで
int16_t raw_current_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

float target_speed[3] = {0, 0, 0}; // 目標速度
float target_angle[3] = {0, 0, 0}; // 目標角度
float past_target_angle[3] = {0, 0, 0};
float rotate_rpm[3] = {0, 0, 0};
float target_rpm[6] = {0, 0, 0, 0, 0, 0}; // 目標回転数（出力）

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

float Kp_angle = 2; 
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

// float l_x = 0.0; // 左スティックのX軸
// float l_y = 0.0; // 左スティックのY軸
// float r_x = 0.0; // 右スティックのX軸
// float r_y = 0.0; // 右スティックのY軸

// float lstick_x = l_x;
// float lstick_y = l_y;
// float rstick_x = r_x; 
// float rstick_y = r_y;

void Core0a(void *args);
void Core0b(void *args);
// void Core1b(void *args);

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

  // CAN
  canWrapper.begin();
  // LED
  pixels.begin();
  pixels.show();
  pixels.setBrightness(255);

  #ifdef WiFi_ON
  // WiFI
  delay(2000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    delay(1000);
  }
  Serial.println("WiFi Ready");

  if (udp.connect(ipAddress, port)) {
    Serial.println("UDP connected");
    Serial.println(WiFi.localIP());

    // Receive
    udp.onPacket([](AsyncUDPPacket packet) { // packetは，受け取ったデータ
      // packet.data()をスペース区切りで分割
      // [V]_[V_direction]_[R]

      String str = String((char*)packet.data());
      in_V = str.substring(0, str.indexOf(" ")).toFloat();

      str = str.substring(str.indexOf(" ") + 1);
      in_V_direction = str.substring(0, str.indexOf(" ")).toFloat();

      str = str.substring(str.indexOf(" ") + 1);
      in_R = str.substring(0, str.indexOf(" ")).toFloat();
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
  // xTaskCreatePinnedToCore(Core1b, "Core1b", 4096, NULL, 1, &thp[2], 1);
}

void loop() {
  #ifdef WiFi_ON
  udp.print("");
  #endif
  
  V = (in_V * 30) / (PI * 0.035); // [rpm]
  V_direction = in_V_direction * PI / 180.0; // [rad]
  R = (in_R * 30) / (PI * 0.5); // [rpm]

  // V_direction 入力は正面が0度になるから90度足す．
  // 本来の座標系は，右が0度，反時計回りに増加する．
  Vx = V * cos(V_direction + PI / 2.0); 
  Vy = V * sin(V_direction + PI / 2.0); 

  // 3つのタイヤの速度を計算
  // 横軸が0度のとき，タイヤの角度は, 30, 150, 270度. 1,2,3
  for (int i = 0; i < 3; i++){
    Vx_list[i] = Vx + 0.5 * cos((PI / 6.0) + wheel_spacing * i + PI / 2.0) * R;
    Vy_list[i] = Vy + 0.5 * sin((PI / 6.0) + wheel_spacing * i + PI / 2.0) * R;
  }

  // ここの＋，－は，タイヤの向きによって変わる．
  for (int i = 0; i < 3; i++){
    target_speed[i] = (sqrt(pow(Vx_list[i], 2) + pow(Vy_list[i], 2))) * (i % 2 == 0 ? 1 : -1);
    target_angle[i] = (atan2(Vy_list[i], Vx_list[i]) * 180.0 / PI) * (i % 2 == 0 ? 1 : -1);
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

  for (int i = 0; i < 3; i++){
    if (target_angle[i] - current_angle[i] < -270.0) {
      target_angle[i] += 360;
    } else if (target_angle[i] - current_angle[i] > 270.0) {
      target_angle[i] -= 360;
    } else if (target_angle[i] - current_angle[i] < -90.0) {
      target_angle[i] += 180;
      target_speed[i] = -target_speed[i];
    } else if (target_angle[i] - current_angle[i] > 90.0) {
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
    target_rpm[0 + (i * 2)] = target_speed[i] + rotate_rpm[i];
    target_rpm[1 + (i * 2)] = target_speed[i] - rotate_rpm[i];
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
    raw_current_data[0 + (i * 2)] = (Kp * error[0 + (i * 2)] + Ki * integral[0 + (i * 2)] + Kd * derivative[0 + (i * 2)]);
    raw_current_data[1 + (i * 2)] = (Kp * error[1 + (i * 2)] + Ki * integral[1 + (i * 2)] + Kd * derivative[1 + (i * 2)]);

    raw_current_data[0 + (i * 2)] = constrain(raw_current_data[0 + (i * 2)], -MAX_AMPERE, MAX_AMPERE); // -3000 ~ 3000
    raw_current_data[1 + (i * 2)] = constrain(raw_current_data[1 + (i * 2)], -MAX_AMPERE, MAX_AMPERE); // -3000 ~ 3000
  }

  make_current_data(raw_current_data, send_current_data1, send_current_data2);
  // canWrapper.sendMessage(0x200, send_current_data1);
  // canWrapper.sendMessage(0x1FF, send_current_data2);

  delay(1);
}

/*
void Core1b(void *args) {
  while (1){
    if (PS4.LatestPacket() && PS4.isConnected()){
      // 移動・回転
      l_x = PS4.LStickX() / 127.0; // -127 ~ 127
      l_y = PS4.LStickY() / 127.0; // -127 ~ 127
      r_x = PS4.RStickX() / 127.0; // -127 ~ 127
      r_y = PS4.RStickY() / 127.0; // -127 ~ 127

      if (l_x < 0.1 && l_x > -0.1) l_x = 0;
      if (l_y < 0.1 && l_y > -0.1) l_y = 0;
      if (r_x < 0.1 && r_x > -0.1) r_x = 0;
      if (r_y < 0.1 && r_y > -0.1) r_y = 0;

      l_x = l_x * abs(l_x);
      l_y = l_y * abs(l_y);
      r_x = r_x * abs(r_x);
      r_y = r_y * abs(r_y);

      if (PS4.L1()){
          l_x *= 0.4;
          l_y *= 0.4;
          r_x *= 0.15;
          r_y *= 0.15;
      }

    lstick_x = l_x * 300.0; // MAX 380
    lstick_y = l_y * 300.0;
    rstick_x = r_x * (400 - lstick_x);
    rstick_y = r_y * (400 - lstick_y);
    delay(10);
    }
  }
}
*/

void Core0a(void *args) {
  while (1) {
    // Serial 入力
    if (Serial.available() > 0) {
      // スペース区切りで分割
      in_V = Serial.parseFloat();
      in_V_direction = Serial.parseFloat();
      in_R = Serial.parseFloat();
    }

    canWrapper.update();
    as5600_tca9548a_get_current_angle(as5600_angle, offset1, offset2);

    // pixels.clear();
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 255, 255));
    }
    pixels.show();
    delay(1);
  }
}

// Serial Ploting
void Core0b(void *args) {
  while (1) {
    // for teleplot
    for (int i = 0; i < 3; i++){
      String str = ">target_speed" + String(i) + ": " + String(target_speed[i]) + "\n";
      Serial.println(str);
      str = ">target_angle" + String(i) + ": " + String(target_angle[i]) + "\n";
      Serial.println(str);
      str = ">current_angle" + String(i) + ": " + String(current_angle[i]) + "\n";
      Serial.println(str);
      str = ">d_a" + String(i) + ": " + String(d_a[i]) + "\n";
      Serial.println(str);
      str = ">as5600_angle" + String(i) + ": " + String(as5600_angle[i]) + "\n";
      Serial.println(str);
    }

    for (int i = 0; i < 6; i++){
      String str = ">target_rpm" + String(i) + ": " + String(target_rpm[i]) + "\n";
      Serial.println(str);
      str = ">raw_current_data" + String(i) + ": " + String(raw_current_data[i]) + "\n";
      Serial.println(str);
      str = ">m_rpm" + String(i) + ": " + String(m_rpm[i]) + "\n";
      Serial.println(str);
    }

    String str = ">V: " + String(V) + "\n";
    Serial.println(str);
    str = ">V_direction: " + String(V_direction) + "\n";
    Serial.println(str);
    str = ">R: " + String(R) + "\n";
    Serial.println(str);
    delay(500);
  }
}