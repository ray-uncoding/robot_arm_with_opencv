#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// I2C 位址 0x40（已用 scanner 確認）
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

const float SERVO_FREQ = 50.0;  // 伺服常用 50Hz

// 4 顆馬達使用的 PCA9685 通道
const uint8_t SERVO_CHANNELS[4] = {0, 4, 8, 12};

// 伺服角度對應的脈寬範圍（可依實際馬達微調）
const int SERVO_MIN_US = 800;   // 0 度對應脈寬
const int SERVO_MAX_US = 2200;  // 180 度對應脈寬

// 用於暫存接收到的命令
String inputString = "";
boolean stringComplete = false;

// 將角度（0-180度）轉換成微秒脈寬
int angleToMicroseconds(float angle) {
  // 限制角度範圍
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  
  // 線性插值計算脈寬
  return (int)(SERVO_MIN_US + (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US));
}

// 將「微秒」轉成 PCA9685 的 0~4095 計數值
void setServoPulseUS(uint8_t channel, int us) {
  float period_us = 1000000.0 / SERVO_FREQ;        // 1 秒 / 頻率 = 週期 (秒) → 轉 μs
  uint16_t ticks = (uint16_t)(us * 4096.0 / period_us);

  if (ticks > 4095) ticks = 4095;
  // 第二個參數是 on tick，通常從 0 開始就好
  pwm.setPWM(channel, 0, ticks);
}

// 設定馬達角度
void setServoAngle(uint8_t motorIndex, float angle) {
  if (motorIndex >= 4) return;  // 防止超出範圍
  
  int pulse = angleToMicroseconds(angle);
  uint8_t channel = SERVO_CHANNELS[motorIndex];
  setServoPulseUS(channel, pulse);
  
  Serial.print("Motor ");
  Serial.print(motorIndex);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.print(" degrees (");
  Serial.print(pulse);
  Serial.println(" us)");
}

// 解析來自 Python 的命令
void parseCommand(String command) {
  // 移除前後空白
  command.trim();
  
  // 預期格式: "angle1,angle2,angle3,angle4"
  float angles[4] = {90, 90, 90, 90}; // 預設角度
  int angleIndex = 0;
  int startIndex = 0;
  
  Serial.println("Received: " + command);
  
  // 解析逗號分隔的角度值
  for (int i = 0; i <= command.length() && angleIndex < 4; i++) {
    if (i == command.length() || command.charAt(i) == ',') {
      if (i > startIndex) {
        String angleStr = command.substring(startIndex, i);
        angles[angleIndex] = angleStr.toFloat();
        angleIndex++;
      }
      startIndex = i + 1;
    }
  }
  
  // 設定四個馬達角度
  for (int i = 0; i < 4; i++) {
    setServoAngle(i, angles[i]);
  }
  
  Serial.println("Command executed successfully!");
}

void printMenu() {
  Serial.println("=== Robot Arm Serial Control ===");
  Serial.println("Send command format: angle1,angle2,angle3,angle4");
  Serial.println("Example: 90,45,120,0");
  Serial.println("Angles range: 0-180 degrees");
  Serial.println("Motors: 0,1,2,3 (channels: 0,4,8,12)");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== Robot Arm with OpenCV Control ===");

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(500);   // 給晶片一點時間穩定

  // 初始化字串緩衝區
  inputString.reserve(200);
  
  // 先把所有馬達轉到中間位置 (90度)
  for (int i = 0; i < 4; i++) {
    setServoAngle(i, 90);
  }

  Serial.println("All motors initialized to 90 degrees");
  printMenu();
  Serial.println("Ready to receive commands from Python...");
}

void loop() {
  // 持續讀取 Serial 輸入
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // 如果收到換行符，表示命令完成
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        stringComplete = true;
      }
    } else {
      // 累積字符到字串中
      inputString += inChar;
    }
  }
  
  // 如果接收到完整命令，就處理它
  if (stringComplete) {
    parseCommand(inputString);
    
    // 清空緩衝區，準備下一個命令
    inputString = "";
    stringComplete = false;
  }
}
