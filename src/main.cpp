#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include "m2006_twai.h"
#include "as5600_tca9548a.h"
#include "led_controller.h"
#include "esp_intr_alloc.h"
#include "pid.h"

#define MAX_AMPERE 3000
#define MAX_YAW_RPM 250

// LED
#define NUMPIXELS 27
#define LED_PIN GPIO_NUM_1

// I2C
#define SDA_PIN GPIO_NUM_5
#define SCL_PIN GPIO_NUM_6

#define WiFi_ON
#define HOME_WiFi
// #define ANNEX_WiFi

#ifdef ANNEX_WiFi
const char* ssid = "Buffalo-G-8360";
const char* password = "tn3krc7dabknc";
uint16_t local_port = 12345;
#endif

#ifdef HOME_WiFi
const char* ssid = "Buffalo-G-2EC8";
const char* password = "exkfnthn7x7k5";
uint16_t local_port = 12345;
#endif

// UDP
float recv_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// AS5600
float init_offset[3] = {0, 0, 0};
float live_offset[3] = {0, 0, 0};
float as5600_angle[3] = {0, 0, 0};

TaskHandle_t thp[3]; // 3つのタスクを作成
AsyncUDP udp; // UDP object
CANWrapper can_wrapper;
LEDController led(LED_PIN, NUMPIXELS);

float Kp1 = 10;
float Ki1 = 0;
float Kd1 = 0;

float Kp2 = Kp1 * 3;
float Ki2 = Ki1 * 3;
float Kd2 = Kd1 * 3;

PIDController pid1(Kp1, Ki1, Kd1); // PID object
PIDController pid2(Kp2, Ki2, Kd2); // PID object

int16_t calc_current_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // ID: 1-8
int8_t send_current_data1[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // ID: 1-8
int8_t send_current_data2[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // ID: 1-8

void recv_cb(AsyncUDPPacket packet);
void task1(void *args);
void task2(void *args);
void make_current_data(int16_t current_data_in[8], int8_t current_data_out1[8], int8_t current_data_out2[8]);

void setup() {
    Serial.begin(115200);
    delay(5000);
    Serial.println("Start");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    // led
    led.init();

    #ifdef WiFi_ON
    // WiFi
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_13dBm);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting to WiFi...");
        led.set_led_cw_ccw(255, 0, 0, true, 1000);
    }
    Serial.println("Connected to WiFi");
    Serial.println("IP address: " + WiFi.localIP().toString());
    led.set_all_led(0, 255, 0);

    // UDP
    if (udp.listen(local_port)) {
        udp.onPacket(recv_cb);
    }
    #endif

    // CAN
    while (!can_wrapper.begin()) {
        Serial.println("Failed to start CAN");
        led.set_led_cw_ccw(255, 0, 0, true, 1000);
    }
    Serial.println("CAN started");
    led.set_all_led(0, 0, 255);

    // AS5600
    as5600_tca9548a_init();
    as5600_tca9548a_get_offset(init_offset);
    init_offset[0] = 12.22;
    init_offset[1] = 26.89;
    init_offset[2] = 109.42;

    // Task
    xTaskCreatePinnedToCore(task1, "task1", 4096, NULL, 1, &thp[0], 1);
    xTaskCreatePinnedToCore(task2, "task2", 4096, NULL, 1, &thp[1], 1);
}

// 時間変数
float dt = 0;
float prev_time = 0;

// 車輪半径
float wheel_radius = 0.035; // [m]
// 車体中心から車輪までの距離
float wheel_distance = 0.5; // [m]
// 車輪間の角度
float wheel_angle = 2 * PI / 3.0; // [rad]  

// 入力
float V = 0.0; // [rpm]
float V_dir = 0.0; // [rad]
float V_yaw = 0.0; // [rpm]

// 車体速度
float Vx = 0.0; // [rpm]
float Vy = 0.0; // [rpm]

// 車輪速度
float Vx_list[3] = {0.0, 0.0, 0.0}; // [rpm]
float Vy_list[3] = {0.0, 0.0, 0.0}; // [rpm]

// 目標値
float target_speed[3] = {0.0, 0.0, 0.0}; // [rpm]
float target_angle[3] = {0.0, 0.0, 0.0}; // [degree]

float rotate_speed[3] = {0.0, 0.0, 0.0}; // [rpm]
float target_rpm[3] = {0.0, 0.0, 0.0}; // [rpm]

void loop() {
    V = (recv_data[0] * 30) / (PI * wheel_radius); // [rpm]
    V_dir = recv_data[1] * (PI / 180); // [rad]
    V_yaw = (recv_data[2] * 30) / (PI * wheel_radius); // [rpm]

    // 車体速度
    Vx = V * cos(V_dir - PI / 2); // [rpm]
    Vy = V * sin(V_dir - PI / 2); // [rpm]

    // 3つの車輪の速度を計算（ベクトル）
    for (int i = 0; i < 3; i++) {
        Vx_list[i] = Vx + wheel_distance * cos((PI / 6.0) + wheel_angle * i + PI / 2) * V_yaw; // [rpm]
        Vy_list[i] = Vy + wheel_distance * sin((PI / 6.0) + wheel_angle * i + PI / 2) * V_yaw; // [rpm]
    }

    // 3つの車輪の速度と角度を計算
    for (int i = 0; i < 3; i++) {
        target_speed[i] = sqrt(pow(Vx_list[i], 2) + pow(Vy_list[i], 2)); // [rpm]
        target_angle[i] = atan2(Vy_list[i], Vx_list[i]) * (180 / PI); // [degree]
    }

    for (int i = 0; i < 3; i++) {
        float error = target_angle[i] - as5600_angle[i];
        // 0と360の境目をまたぐときに正負が逆になるので，ある程度のところで予め補正する
        if (error < -270) {
            target_angle[i] += 360;
        } else if (error > 270) {
            target_angle[i] -= 360;
        }
        // 目標角度に対して時計回りと反時計回りのどちらで回転するかを決定
        if (error < -90) {
            target_angle[i] += 180;
            target_speed[i] *= -1;
        } else if (error > 90) {
            target_angle[i] -= 180;
            target_speed[i] *= -1;
        }
    }

    /* --- コントローラ --- */
    dt = (millis() - prev_time) / 1000.0; // [s]
    prev_time = millis();
    // PID制御(角度)
    pid1.set_gain(Kp1, Ki1, Kd1);
    for (int i = 0; i < 3; i++) {
        rotate_speed[i] = (float)pid1.calculate(target_angle[i], as5600_angle[i], dt);
        rotate_speed[i] = constrain(rotate_speed[i], -MAX_YAW_RPM, MAX_YAW_RPM);
    }
    Serial.println(String(pid1.calculate(target_angle[0], as5600_angle[0], dt)));

    // 目標回転数を計算
    for (int i = 0; i < 3; i++) {
        target_rpm[0 + (i * 2)] = target_speed[i] - rotate_speed[i];
        target_rpm[1 + (i * 2)] = target_speed[i] + rotate_speed[i];
    }

    // PID制御(速度)
    pid2.set_gain(Kp2, Ki2, Kd2);
    for (int i = 0; i < 3; i++) {
        calc_current_data[0 + (i * 2)] = (int16_t)pid2.calculate(target_rpm[0 + (i * 2)], m_rpm[0 + (i * 2)], dt);
        calc_current_data[1 + (i * 2)] = (int16_t)pid2.calculate(target_rpm[1 + (i * 2)], m_rpm[1 + (i * 2)], dt);
        calc_current_data[0 + (i * 2)] = constrain(calc_current_data[0 + (i * 2)], -MAX_AMPERE, MAX_AMPERE);
        calc_current_data[1 + (i * 2)] = constrain(calc_current_data[1 + (i * 2)], -MAX_AMPERE, MAX_AMPERE);
    }
    /* ------------------ */

    /* --- プラント ------ */
    make_current_data(calc_current_data, send_current_data1, send_current_data2);
    can_wrapper.sendMessage(0x200, send_current_data1);
    can_wrapper.sendMessage(0x1FF, send_current_data2);
    /* ----------------- */

    delay(10); // 1/5 時定数
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


void recv_cb(AsyncUDPPacket packet) {
    String str = String((char*)packet.data());
    // スペース区切りで受信データを分割
    for (int i = 0; i < 3; i++) { // 受信データの数だけ繰り返す
        recv_data[i] = str.substring(0, str.indexOf(" ")).toFloat();
        str = str.substring(str.indexOf(" ") + 1);
    }
}

void task1(void *args) {
    while (1) {
        can_wrapper.update();
        as5600_tca9548a_get_current_angle(as5600_angle, init_offset, live_offset);
        delay(1);
    }
}

void task2(void *args) {
    while (1) {
        Serial.println(">target_rpm[0]: " + String(target_rpm[0]));
        Serial.println(">m_rpm[0]: " + String(m_rpm[0]));
        Serial.println(">calc_current_data[0]: " + String(calc_current_data[0]));
        Serial.println(">as5600_angle[0]: " + String(as5600_angle[0]));

        Serial.println(">rotate_speed[0]: " + String(rotate_speed[0]));
        Serial.println(">target_speed[0]: " + String(target_speed[0]));
        Serial.println(">target_angle[0]: " + String(target_angle[0]));

        Serial.println(">recv_data[0]: " + String(recv_data[0]));
        Serial.println(">recv_data[1]: " + String(recv_data[1]));
        Serial.println(">recv_data[2]: " + String(recv_data[2]));
        delay(100);
    }
}