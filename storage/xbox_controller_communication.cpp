#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_0_MIN 100
#define SERVO_0_MAX 500

#define SERVO_1_MIN 250
#define SERVO_1_MAX 500

#define SERVO_X 0   // channel for X axis servo
#define SERVO_Y 1   // channel for Y axis servo

void setup() {
  Serial.begin(9600);      // Debug via USB
  Serial1.begin(115200);   // Communication with Pi
  pwm.begin();
  pwm.setPWMFreq(60);      // 60 Hz for servos
}

void loop() {
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');  // e.g. "90,120"
    int commaIndex = data.indexOf(',');

    if (commaIndex > 0) {
      int xl = data.substring(0, commaIndex).toInt();
      int yl = data.substring(commaIndex + 1).toInt();
      int xr = data.substring(commaIndex + 2).toInt();
      int yr = data.substring(commaIndex + 3).toInt();


      // Map 0–180 to pulse length
      int xPulse = map(xl, 0, 1, SERVO_0_MIN, SERVO_0_MAX);
      int yPulse = map(yl, 0, 1, SERVO_1_MIN, SERVO_1_MAX);

      // Send to servos
      pwm.setPWM(SERVO_X, 0, xPulse);
      pwm.setPWM(SERVO_Y, 0, yPulse);

      Serial.print("Received X: ");
      Serial.print(xl);
      Serial.print(" Y: ");
      Serial.println(yl);
    }
  }
}
