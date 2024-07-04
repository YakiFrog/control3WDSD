#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
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

// WiFi
#define WiFi_ON
#define HOME_WiFi
// #define ANNEX_WiFi

// DEBUG
// #define DEBUG

#ifdef ANNEX_WiFi
const char* ssid = "Buffalo-G-8360";
const char* password = "tn3krc7dabknc";
IPAddress server_ip(192, 168, 11, 60);
uint16_t server_port = 1113;
uint16_t local_port = 12345;
#endif

#ifdef HOME_WiFi
const char* ssid = "Buffalo-G-2EC8";
const char* password = "exkfnthn7x7k5";
IPAddress server_ip(192, 168, 11, 2);
uint16_t server_port = 1113;
uint16_t local_port = 12345;
#endif

// 受信したいデータ例
// [Type 0] V, V_dir, V_yaw, 
// [Type 1] LED_R, LED_G, LED_B, 光るパターン, 光る時間
// [Type 2] Kp1, Ki1, Kd1, Kp2, Ki2, Kd2

// UDP
float recv_data[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // [Type 1: V, V_dir, V_yaw]
float data[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // recv_dataのコピー

// UDP受信時間
unsigned long recv_time = 0;

// AS5600
float init_offset[3] = {0, 0, 0};
float live_offset[3] = {0, 0, 0};
float as5600_angle[3] = {0, 0, 0};

TaskHandle_t thp[3]; // 3つのタスクを作成
WiFiUDP udp; // WiFiUDP object
CANWrapper can_wrapper;
LEDController led(LED_PIN, NUMPIXELS);

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

// 誤差<10%程度
float Kp1 = 12; // 12
float Ki1 = 0; // 0.1:BEST
float Kd1 = 0;

float Kp2 = Kp1 * 3;
float Ki2 = Ki1 * 3;
float Kd2 = Kd1 * 3;

PIDController pid1(Kp1, Ki1, Kd1); // PID object
PIDController pid2(Kp2, Ki2, Kd2); // PID object

// 目標値
float target_speed[3] = {0.0, 0.0, 0.0}; // [rpm]
float target_angle[3] = {0.0, 0.0, 0.0}; // [degree]

float rotate_speed[3] = {0.0, 0.0, 0.0}; // [rpm]
float target_rpm[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // [rpm]
float prev_target_rpm[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // [rpm]

// pose
float x = 0.0;
float y = 0.0;
float qz = 0.0;
float qw = 0.0;

// twist
float linear_x = 0.0;
float linear_y = 0.0;
float angular_z = 0.0;
float world_angle = 0.0;

// モータの計算モデルのパラメタ
// ゲイン調整でうまくいかなかったら．これをいじる
int K = 1.00; // ゲイン 0.75 (実際の値が入力値に対してどれだけの割合で反応するか)
int T = 0.1; // 時定数  0.1 (0より大きいほど遅れる)

int16_t calc_current_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // ID: 1-8
int8_t send_current_data1[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // ID: 1-8
int8_t send_current_data2[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // ID: 1-8

void recv_msg();
void task1(void *args);
void task2(void *args);
void task3(void *args);
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
        led.set_led_cw_ccw(255, 0, 0, true, 900);
    }
    Serial.println("Connected to WiFi");
    Serial.println("IP address: " + WiFi.localIP().toString());
    led.set_all_led(0, 255, 0);

    // WiFiUDP server
    udp.begin(local_port);

    #endif

    // CAN
    while (!can_wrapper.begin()) {
        Serial.println("Failed to start CAN");
        led.set_led_cw_ccw(255, 0, 255, true, 900);
    }
    Serial.println("CAN started");
    led.set_all_led(0, 0, 255);

    // AS5600
    as5600_tca9548a_init();
    as5600_tca9548a_get_offset(init_offset);
    Serial.println(String(init_offset[0]) + " " + String(init_offset[1]) + " " + String(init_offset[2]));
    init_offset[0] = 12.22;
    init_offset[1] = 26.89;
    init_offset[2] = 113.91;

    // Task
    xTaskCreatePinnedToCore(task1, "task1", 4096, NULL, 1, &thp[0], 1); // CAN, AS5600
    xTaskCreatePinnedToCore(task2, "task2", 4096, NULL, 1, &thp[1], 1); // Serial
    xTaskCreatePinnedToCore(task3, "task3", 4096, NULL, 1, &thp[2], 1); // LED
}

void loop() {
    recv_msg();
    // 1秒以上受信データがない場合は，データを初期化
    if (millis() - recv_time > 1000) {
        data[0] = 0.01; // 0だと角度保持できないので，0.00001にする
        data[1] = 0;
        data[2] = 0;
    }

    V = (data[0] * 30) / (PI * wheel_radius); // [rpm]
    V_dir = data[1] * (PI / 180); // [rad]
    V_yaw = (data[2] * 30) / (PI * wheel_radius); // [rpm]

    // 車体速度(twist)
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
        target_angle[i] = atan2(Vy_list[i], Vx_list[i]) * (180 / PI); // [degree] (-180 to 180)
    }

    for (int i = 0; i < 3; i++) {
        float error = target_angle[i] - as5600_angle[i]; // [degree] (-360 to 360)
        // 0と360の境目をまたぐときに正負が逆になるので，ある程度のところで予め補正する, ふらつくなら消す．
        // if (error < -(270 + 45)) {
        //     target_angle[i] += 360;
        // } else if (error > 270 + 45) {
        //     target_angle[i] -= 360;
        // }
        // 目標角度に対して時計回りと反時計回りのどちらで回転するかを決定
        if (error < -90) {
            target_angle[i] += 180;
            target_speed[i] *= -1;
        } else if (error > 90) {
            target_angle[i] -= 180;
            target_speed[i] *= -1;
        }
    }

    // もし，目標角度と現在角度の差が45度以上なら，すべての車輪の速度を0にする
    // TODO: スピードを出している時に，角度が大きく変わると0になるので，ぎくしゃくする．あんまり良くない
    for (int i = 0; i < 3; i++) {
        if (abs(target_angle[i] - as5600_angle[i]) > 45) {
            for (int j = 0; j < 3; j++) {
                target_speed[j] = 0;
            }
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

    // 目標回転数を計算
    for (int i = 0; i < 3; i++) {
        target_rpm[0 + (i * 2)] = target_speed[i] - rotate_speed[i];
        target_rpm[1 + (i * 2)] = target_speed[i] + rotate_speed[i];
    }

    // target_rpmを1次遅れに通す
    for (int i = 0; i < 3; i++) {
        target_rpm[0 + (i * 2)] = K * (1 - std::exp(-dt / T)) * target_rpm[0 + (i * 2)] + prev_target_rpm[0 + (i * 2)] * std::exp(-dt / T);
        target_rpm[1 + (i * 2)] = K * (1 - std::exp(-dt / T)) * target_rpm[1 + (i * 2)] + prev_target_rpm[1 + (i * 2)] * std::exp(-dt / T);
        prev_target_rpm[0 + (i * 2)] = target_rpm[0 + (i * 2)];
        prev_target_rpm[1 + (i * 2)] = target_rpm[1 + (i * 2)];
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

    /* --- ROS2 --- */

    // twist (linear, angular)
    // rpm -> m/s
    linear_x = Vx * (PI * wheel_radius / 30); // [m/s]
    linear_y = Vy * (PI * wheel_radius / 30); // [m/s]
    angular_z = V_yaw * (PI / 30); // [rad/s]
    world_angle += angular_z * dt;

    // angular_z分，x,y を回転, dtもつかう
    x += (linear_x * cos(world_angle) - linear_y * sin(world_angle)) * dt;
    y += (linear_x * sin(world_angle) + linear_y * cos(world_angle)) * dt;
    qz = sin(world_angle / 2);
    qw = cos(world_angle / 2);

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

void recv_msg() {
    int packet_size = udp.parsePacket();
    if (packet_size > 0) {
        recv_time = millis();
        String str = "";
        for (int i = 0; i < packet_size; i++) {
            str += (char)udp.read();
        }
        
        // スペース区切りで受信データを分割
        for (int i = 0; i < 3; i++) { // 受信データの数だけ繰り返す
            recv_data[i] = str.substring(0, str.indexOf(" ")).toFloat();
            str = str.substring(str.indexOf(" ") + 1);
        }
        // 受信データをコピー
        if (recv_data[0] == 0) {
            data[0] = 0.00001;
        } else {
            data[0] = recv_data[0];
            data[1] = recv_data[1];
        }
        data[2] = recv_data[2];
    }
}

void task1(void *args) {
    while (1) {
        can_wrapper.update();
        as5600_tca9548a_get_current_angle(as5600_angle, init_offset, live_offset);
        delay(1);
    }
}

// シリアル通信，UDP通信
void task2(void *args) {
    while (1) {
        #ifdef DEBUG
        // Serial.println(">target_rpm[0]: " + String(target_rpm[0]));
        // Serial.println(">m_rpm[0]: " + String(m_rpm[0]));
        // Serial.println(">calc_current_data[0]: " + String(calc_current_data[0]));
        // Serial.println(">as5600_angle[0]: " + String(as5600_angle[0]));

        // Serial.println(">rotate_speed[0]: " + String(rotate_speed[0]));
        // Serial.println(">target_speed[0]: " + String(target_speed[0]));
        // Serial.println(">target_angle[0]: " + String(target_angle[0]));

        // Serial.println(">recv_data[0]: " + String(recv_data[0]));
        // Serial.println(">recv_data[1]: " + String(recv_data[1]));
        // Serial.println(">recv_data[2]: " + String(recv_data[2]));
        #else
        udp.beginPacket(server_ip, server_port);
        String send_data = "";
        // for (int i = 0; i < 6; i++) {
        //     // send_data += String(target_rpm[i]) + " ";
        //     send_data += String(m_rpm[i]) + " ";
        // }
        send_data += String(x) + " " + String(y) + " ";
        send_data += String(qz) + " " + String(qw) + " ";
        send_data += String(linear_x) + " " + String(linear_y) + " " + String(angular_z);
        udp.print(send_data);
        udp.endPacket();
        #endif
        delay(100);
    }
}

// LED制御 
void task3(void *args) {
    while (1) {
        if (data[2] > 0.15) {
            led.set_led_cw_ccw(0, 255, 255, true, 900);
        } else if (data[2] < -0.15) {
            led.set_led_cw_ccw(0, 255, 255, false, 900);
        } else {
            led.set_all_led(0, 255, 255);
        }
    }
}