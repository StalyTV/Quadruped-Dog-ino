#ifndef LEG_H
#define LEG_H

#include "Servo.h"

// Servo indices for each joint type
#define SHOULDER_SERVO 0  // Hip rotation (left/right)
#define UPPER_LEG_SERVO 1 // Upper leg (forward/back)
#define LOWER_LEG_SERVO 2 // Lower leg/foot (up/down)

// Leg positions
enum LegPosition {
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  REAR_LEFT = 2,
  REAR_RIGHT = 3
};

class Leg {
private:
  Servo shoulder;    // Hip rotation servo
  Servo upperLeg;    // Upper leg servo
  Servo lowerLeg;    // Lower leg/foot servo
  LegPosition position;
  bool isGrounded;
  
  // Current leg coordinates (relative to body)
  float x, y, z;
  
  // Leg geometry parameters
  float shoulderLength;
  float upperLegLength;
  float lowerLegLength;

public:
  Leg();
  void init(Adafruit_PWMServoDriver* driver, LegPosition legPos, 
            int shoulderChannel, int upperChannel, int lowerChannel);
  
  // Basic servo control
  void setShoulderAngle(float angle);
  void setUpperLegAngle(float angle);
  void setLowerLegAngle(float angle);
  
  // Coordinate-based control
  void setPosition(float x, float y, float z);
  void getPosition(float& x, float& y, float& z);
  
  // Predefined poses
  void standingPose();
  void sittingPose();
  void liftedPose();
  void neutralPose();
  
  // Movement functions
  void stepForward(float stepSize);
  void stepBackward(float stepSize);
  void stepSideways(float stepSize);
  void liftLeg(float height);
  void lowLeg();
  
  // State management
  void setGrounded(bool grounded);
  bool getGrounded();
  LegPosition getLegPosition();
  
  // Calibration and setup
  void calibrateServos();
  void setLegGeometry(float shoulder, float upper, float lower);
  
  // Inverse kinematics
  bool calculateAngles(float x, float y, float z, float& shoulderAngle, float& upperAngle, float& lowerAngle);
  
  // Forward kinematics
  void calculatePosition(float shoulderAngle, float upperAngle, float lowerAngle, float& x, float& y, float& z);
};

#endif
