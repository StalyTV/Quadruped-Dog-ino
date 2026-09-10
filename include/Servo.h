#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_MIN 150
#define SERVO_MAX 600

class Servo {
private:
  int channel;
  int currentPosition;
  int minPosition;
  int maxPosition;
  Adafruit_PWMServoDriver* pwmDriver;

public:
  Servo();
  void init(Adafruit_PWMServoDriver* driver, int servoChannel, int minPos = SERVO_MIN, int maxPos = SERVO_MAX);
  void setPosition(int position);
  void setAngle(float angle);
  int getCurrentPosition();
  float getCurrentAngle();
  void moveToPosition(int position, int delayMs = 20);
  void moveToAngle(float angle, int delayMs = 20);
  bool isInRange(int position);
  void calibrate(int minPos, int maxPos);
};

#endif
