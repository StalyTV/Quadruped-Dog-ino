#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

#include "Leg.h"
#include "Gait.h"
#include <Adafruit_PWMServoDriver.h>

#define NUM_LEGS 4
#define SERVOS_PER_LEG 3
#define PWM_FREQUENCY 200

// Movement states
enum MovementState {
  IDLE,
  WALKING,
  TURNING_LEFT,
  TURNING_RIGHT,
  SITTING,
  STANDING
};

class QuadrupedRobot {
private:
  Adafruit_PWMServoDriver pwm;
  Leg legs[NUM_LEGS];
  Gait gait;
  MovementState currentState;
  
  // Robot body parameters
  float bodyHeight;
  float bodyWidth;
  float bodyLength;
  
  // Movement parameters
  float speed;
  float direction;  // 0-360 degrees
  float turnRate;
  
  // Balance and stability
  float centerOfGravityX;
  float centerOfGravityY;
  bool isBalanced;

public:
  QuadrupedRobot();
  
  // Initialization
  void init();
  void calibrateAllServos();
  void setBodyDimensions(float height, float width, float length);
  
  // Basic movement commands
  void stand();
  void sit();
  void walkForward(float speed = 1.0);
  void walkBackward(float speed = 1.0);
  void turnLeft(float rate = 1.0);
  void turnRight(float rate = 1.0);
  void stop();
  
  // Advanced movement
  void walkDirection(float angle, float speed = 1.0);
  void strafe(float direction, float speed = 1.0);
  void setGaitType(GaitType gait);
  
  // Body control
  void adjustBodyHeight(float height);
  void adjustBodyPitch(float angle);
  void adjustBodyRoll(float angle);
  void adjustBodyYaw(float angle);
  
  // Balance and stability
  void updateBalance();
  bool checkStability();
  void correctBalance();
  
  // Leg coordination
  void updateLegPositions();
  void coordinated_step();
  void emergencyStop();
  
  // State management
  void setState(MovementState state);
  MovementState getState();
  void update();  // Main update loop
  
  // Utility functions
  void printStatus();
  void printLegPositions();
  bool isMoving();
  
  // Safety functions
  bool selfTest();
  void enableSafetyLimits(bool enabled);
  bool checkServoHealth();
};

#endif
