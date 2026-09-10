#include "Leg.h"

Leg::Leg() {
  position = FRONT_LEFT;
  isGrounded = true;
  x = 0;
  y = 0;
  z = 0;
  shoulderLength = 50;   // Default values - adjust based on your robot
  upperLegLength = 100;
  lowerLegLength = 100;
}


void Leg::init(Adafruit_PWMServoDriver* driver, LegPosition legPos, 
               int shoulderChannel, int upperChannel, int lowerChannel) {
  position = legPos;
  shoulder.init(driver, shoulderChannel);
  upperLeg.init(driver, upperChannel);
  lowerLeg.init(driver, lowerChannel);
  
  // Set to neutral pose initially
  neutralPose();
}

void Leg::setShoulderAngle(float angle) {
  shoulder.setAngle(angle);
}

void Leg::setUpperLegAngle(float angle) {
  upperLeg.setAngle(angle);
}

void Leg::setLowerLegAngle(float angle) {
  lowerLeg.setAngle(angle);
}

void Leg::setPosition(float newX, float newY, float newZ) {
  x = newX;
  y = newY;
  z = newZ;
  
  // Calculate required servo angles using inverse kinematics
  float shoulderAngle, upperAngle, lowerAngle;
  if (calculateAngles(x, y, z, shoulderAngle, upperAngle, lowerAngle)) {
    setShoulderAngle(shoulderAngle);
    setUpperLegAngle(upperAngle);
    setLowerLegAngle(lowerAngle);
  }
}

void Leg::getPosition(float& outX, float& outY, float& outZ) {
  outX = x;
  outY = y;
  outZ = z;
}

void Leg::standingPose() {
  // Default standing position - adjust angles based on your robot's geometry
  setShoulderAngle(90);   // Neutral shoulder position
  setUpperLegAngle(45);   // Upper leg angled down
  setLowerLegAngle(135);  // Lower leg supporting weight
  isGrounded = true;
}

void Leg::sittingPose() {
  // Sitting position - legs folded
  setShoulderAngle(90);
  setUpperLegAngle(90);   // Upper leg more vertical
  setLowerLegAngle(45);   // Lower leg folded under
  isGrounded = true;
}

void Leg::liftedPose() {
  // Leg lifted for stepping
  setShoulderAngle(90);
  setUpperLegAngle(30);   // Upper leg lifted
  setLowerLegAngle(90);   // Lower leg neutral
  isGrounded = false;
}

void Leg::neutralPose() {
  // Safe neutral position
  setShoulderAngle(90);
  setUpperLegAngle(90);
  setLowerLegAngle(90);
  isGrounded = false;
}

void Leg::stepForward(float stepSize) {
  // Implement forward step motion
  // This would involve lifting the leg, moving forward, then placing down
  // For now, just a simple implementation
  liftLeg(20);
  delay(100);
  // Move shoulder forward (adjust based on leg position)
  float currentAngle = shoulder.getCurrentAngle();
  setShoulderAngle(currentAngle + stepSize);
  delay(100);
  lowLeg();
}

void Leg::stepBackward(float stepSize) {
  // Implement backward step motion
  liftLeg(20);
  delay(100);
  float currentAngle = shoulder.getCurrentAngle();
  setShoulderAngle(currentAngle - stepSize);
  delay(100);
  lowLeg();
}

void Leg::stepSideways(float stepSize) {
  // Implement sideways step motion
  // Implementation depends on robot design
}

void Leg::liftLeg(float height) {
  // Lift leg to specified height
  setUpperLegAngle(30);   // Lift upper leg
  setLowerLegAngle(60);   // Adjust lower leg
  isGrounded = false;
}

void Leg::lowLeg() {
  // Lower leg to ground
  standingPose();
  isGrounded = true;
}

void Leg::setGrounded(bool grounded) {
  isGrounded = grounded;
}

bool Leg::getGrounded() {
  return isGrounded;
}

LegPosition Leg::getLegPosition() {
  return position;
}

void Leg::calibrateServos() {
  // Calibrate all servos for this leg
  // Move through range of motion and adjust limits
  shoulder.calibrate(SERVO_MIN, SERVO_MAX);
  upperLeg.calibrate(SERVO_MIN, SERVO_MAX);
  lowerLeg.calibrate(SERVO_MIN, SERVO_MAX);
}

void Leg::setLegGeometry(float shoulder, float upper, float lower) {
  shoulderLength = shoulder;
  upperLegLength = upper;
  lowerLegLength = lower;
}

bool Leg::calculateAngles(float x, float y, float z, float& shoulderAngle, float& upperAngle, float& lowerAngle) {
  // Simplified inverse kinematics - you would implement proper 3D IK here
  // This is a basic 2D approximation
  
  // Calculate shoulder angle (rotation around vertical axis)
  shoulderAngle = atan2(y, x) * 180.0 / PI + 90; // Convert to degrees and offset
  
  // Calculate distance from shoulder to target
  float horizontalDist = sqrt(x*x + y*y) - shoulderLength;
  float totalDist = sqrt(horizontalDist*horizontalDist + z*z);
  
  // Check if target is reachable
  if (totalDist > (upperLegLength + lowerLegLength)) {
    return false; // Target too far
  }
  
  // Calculate upper and lower leg angles using law of cosines
  float upperLegAngleRad = acos((upperLegLength*upperLegLength + totalDist*totalDist - lowerLegLength*lowerLegLength) / 
                               (2 * upperLegLength * totalDist));
  float elevationAngle = atan2(z, horizontalDist);
  
  upperAngle = (upperLegAngleRad + elevationAngle) * 180.0 / PI;
  
  float lowerLegAngleRad = acos((upperLegLength*upperLegLength + lowerLegLength*lowerLegLength - totalDist*totalDist) / 
                               (2 * upperLegLength * lowerLegLength));
  lowerAngle = 180 - (lowerLegAngleRad * 180.0 / PI);
  
  return true;
}

void Leg::calculatePosition(float shoulderAngle, float upperAngle, float lowerAngle, float& x, float& y, float& z) {
  // Forward kinematics - calculate position from angles
  float shoulderRad = (shoulderAngle - 90) * PI / 180.0;
  float upperRad = upperAngle * PI / 180.0;
  float lowerRad = lowerAngle * PI / 180.0;
  
  // Calculate end effector position
  float shoulderX = shoulderLength * cos(shoulderRad);
  float shoulderY = shoulderLength * sin(shoulderRad);
  
  float upperEndX = shoulderX + upperLegLength * cos(upperRad);
  float upperEndZ = upperLegLength * sin(upperRad);
  
  x = upperEndX + lowerLegLength * cos(upperRad + lowerRad - PI);
  y = shoulderY;
  z = upperEndZ + lowerLegLength * sin(upperRad + lowerRad - PI);
}
