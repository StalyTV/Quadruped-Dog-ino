#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

class SensorManager {
private:
  bool hasIMU;
  bool hasUltrasonic;
  bool hasCamera;
  
public:
  SensorManager();
  void init();
  
  // IMU functions
  void readIMU(float& pitch, float& roll, float& yaw);
  bool isUpsideDown();
  
  // Distance sensing
  float getDistanceForward();
  float getDistanceLeft();
  float getDistanceRight();
  
  // Environmental awareness
  bool detectObstacle();
  float getGroundDistance();
  
  void update();
};

#endif
