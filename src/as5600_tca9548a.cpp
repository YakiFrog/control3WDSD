#include <Arduino.h>
#include <Wire.h>
#include <TCA9548A.h>
#include <AS5600.h>
#include "as5600_tca9548a.h"

static AS5600 as5600_1;
static AS5600 as5600_2;
static AS5600 as5600_3;
static AS5600 as5600[3] = {as5600_1, as5600_2, as5600_3};

static TCA9548A I2CMux;

static float gear_ratio = 1.0; // Differential Swerve Driveのギア比

// setup()で呼び出す
void as5600_tca9548a_init() {
  try {
    Wire.setPins(GPIO_NUM_5, GPIO_NUM_6); // ESP32 users, use setPins(sda, scl) if customised, *before* passing Wire to the library (the line below).
    I2CMux.begin(Wire); // TCA9548Aの初期化
    I2CMux.closeAll(); // 全てのチャンネルを閉じる
    for (int i = 0; i < 3; i++) {
      as5600[i].setDirection(AS5600_CLOCK_WISE);
    }
  } catch (const std::exception& e) {
    Serial.println("Error: " + String(e.what()));
  }
}

// offsetを取得する
void as5600_tca9548a_get_offset(float offset1[3]) {
  try {
    for (int i = 0; i < 3; i++) {
      I2CMux.openChannel((i + 2) % 3 + 1); // i=0 -> 3, i=1 -> 2, i=2 -> 1
      offset1[i] = as5600[i].getCumulativePosition() * AS5600_RAW_TO_DEGREES / gear_ratio;
      I2CMux.closeChannel((i + 2) % 3 + 1);
    }
  } catch (const std::exception& e) {
    Serial.println("Error: " + String(e.what()));
  }
}

// 現在の角度を取得する(0 to 360)
void as5600_tca9548a_get_current_angle(float current_angle[3], float offset1[3], float offset2[3]) {
  try {
    for (int i = 0; i < 3; i++) {
      I2CMux.openChannel((i + 2) % 3 + 1);
      as5600[i].getCumulativePosition(); // これがないと，getCumulativePosition()の値がおかしくなる
      // -360 to 0 to 360
      current_angle[i] = (as5600[i].getCumulativePosition() * AS5600_RAW_TO_DEGREES / gear_ratio) - offset1[i] - offset2[i];
      I2CMux.closeChannel((i + 2) % 3 + 1);

      // 回転時の補正（これが曲者）
      if (current_angle[i] >= 360) { // 360度以上,時計回りしたらリセット
        offset2[i] += 360 - (as5600[i].resetPosition() * AS5600_RAW_TO_DEGREES / gear_ratio);
      } else if (current_angle[i] <= -360) { // -360度以上,反時計回りしたらリセット
        offset2[i] += -360 - (as5600[i].resetPosition() * AS5600_RAW_TO_DEGREES / gear_ratio);
      }

      // -360 to 0 -> 0 to 360: 反時計回りしたとき，マイナスになるので補正(例：-1 -> 359)
      if (current_angle[i] < 0) {
        current_angle[i] = 360 + current_angle[i];
      }

      // -180 to 180
      if (current_angle[i] > 180) {
        current_angle[i] -= 360;
      }
    }
  } catch (const std::exception& e) {
    Serial.println("Error: " + String(e.what()));
  }
}