#include "Servo.h"

Servo::Servo() {
  channel = 0;
  currentPosition = 0;
  minPosition = SERVO_MIN;
  maxPosition = SERVO_MAX;
  pwmDriver = nullptr;
}

void Servo::init(Adafruit_PWMServoDriver* driver, int servoChannel, int minPos, int maxPos) {
  pwmDriver = driver;
  channel = servoChannel;
  minPosition = minPos;
  maxPosition = maxPos;
  currentPosition = (minPos + maxPos) / 2; // Start at middle position
}

void Servo::setPosition(int position) {
  if (isInRange(position) && pwmDriver != nullptr) {
    pwmDriver->setPWM(channel, 0, position);
    currentPosition = position;
  }
}

void Servo::setAngle(float angle) {
  if (angle >= 0 && angle <= 180) {
    int position = map(angle, 0, 180, minPosition, maxPosition);
    setPosition(position);
  }
}

int Servo::getCurrentPosition() {
  return currentPosition;
}

float Servo::getCurrentAngle() {
  return map(currentPosition, minPosition, maxPosition, 0, 180);
}

void Servo::moveToPosition(int position, int delayMs) {
  if (!isInRange(position)) return;
  
  int step = (position > currentPosition) ? 1 : -1;
  while (currentPosition != position) {
    currentPosition += step;
    setPosition(currentPosition);
    delay(delayMs);
  }
}

void Servo::moveToAngle(float angle, int delayMs) {
  int targetPosition = map(angle, 0, 180, minPosition, maxPosition);
  moveToPosition(targetPosition, delayMs);
}

bool Servo::isInRange(int position) {
  return (position >= minPosition && position <= maxPosition);
}

void Servo::calibrate(int minPos, int maxPos) {
  minPosition = minPos;
  maxPosition = maxPos;
}
