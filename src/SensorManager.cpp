#include "SensorManager.h"

SensorManager::SensorManager() {
  hasIMU = false;
  hasUltrasonic = false;
  hasCamera = false;
}

void SensorManager::init() {
  Serial.println("Initializing sensors...");
  
  // Initialize IMU if available
  // hasIMU = initIMU();
  
  // Initialize ultrasonic sensors if available
  // hasUltrasonic = initUltrasonic();
  
  // Initialize camera if available
  // hasCamera = initCamera();
  
  Serial.print("Sensors initialized - IMU: ");
  Serial.print(hasIMU ? "Yes" : "No");
  Serial.print(", Ultrasonic: ");
  Serial.print(hasUltrasonic ? "Yes" : "No");
  Serial.print(", Camera: ");
  Serial.println(hasCamera ? "Yes" : "No");
}

void SensorManager::readIMU(float& pitch, float& roll, float& yaw) {
  if (!hasIMU) {
    pitch = 0;
    roll = 0;
    yaw = 0;
    return;
  }
  
  // Read IMU data
  // Implementation depends on specific IMU sensor
  pitch = 0; // Replace with actual IMU reading
  roll = 0;  // Replace with actual IMU reading
  yaw = 0;   // Replace with actual IMU reading
}

bool SensorManager::isUpsideDown() {
  if (!hasIMU) return false;
  
  float pitch, roll, yaw;
  readIMU(pitch, roll, yaw);
  
  // Check if robot is upside down (roll > 90 degrees)
  return (abs(roll) > 90);
}

float SensorManager::getDistanceForward() {
  if (!hasUltrasonic) return -1; // No sensor available
  
  // Read forward-facing ultrasonic sensor
  // Implementation depends on specific sensor
  return 100; // Placeholder - replace with actual reading
}

float SensorManager::getDistanceLeft() {
  if (!hasUltrasonic) return -1;
  
  // Read left-facing ultrasonic sensor
  return 100; // Placeholder
}

float SensorManager::getDistanceRight() {
  if (!hasUltrasonic) return -1;
  
  // Read right-facing ultrasonic sensor
  return 100; // Placeholder
}

bool SensorManager::detectObstacle() {
  if (!hasUltrasonic) return false;
  
  float forwardDistance = getDistanceForward();
  
  // Obstacle detected if something is closer than 30cm
  return (forwardDistance < 30 && forwardDistance > 0);
}

float SensorManager::getGroundDistance() {
  if (!hasUltrasonic) return -1;
  
  // Read downward-facing ultrasonic sensor
  return 15; // Placeholder - typical ground clearance
}

void SensorManager::update() {
  // Update sensor readings
  // This would be called regularly to update sensor data
  
  if (hasIMU) {
    // Update IMU readings
  }
  
  if (hasUltrasonic) {
    // Update ultrasonic readings
  }
  
  if (hasCamera) {
    // Process camera data
  }
}
