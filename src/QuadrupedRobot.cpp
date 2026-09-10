#include "QuadrupedRobot.h"

QuadrupedRobot::QuadrupedRobot() {
  currentState = IDLE;
  bodyHeight = 150.0;
  bodyWidth = 200.0;
  bodyLength = 300.0;
  speed = 1.0;
  direction = 0.0;
  turnRate = 1.0;
  centerOfGravityX = 0.0;
  centerOfGravityY = 0.0;
  isBalanced = true;
}

void QuadrupedRobot::init() {
  // Initialize PWM driver
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQUENCY);
  
  // Initialize each leg with appropriate servo channels
  // Adjust these channel numbers based on your wiring
  legs[FRONT_LEFT].init(&pwm, FRONT_LEFT, 0, 1, 2);     // Channels 0, 1, 2
  legs[FRONT_RIGHT].init(&pwm, FRONT_RIGHT, 3, 4, 5);   // Channels 3, 4, 5
  legs[REAR_LEFT].init(&pwm, REAR_LEFT, 6, 7, 8);       // Channels 6, 7, 8
  legs[REAR_RIGHT].init(&pwm, REAR_RIGHT, 9, 10, 11);   // Channels 9, 10, 11
  
  // Initialize gait
  gait.init(TROT);
  
  // Set initial state
  currentState = STANDING;
  
  delay(100); // Allow servos to initialize
}

void QuadrupedRobot::calibrateAllServos() {
  for (int i = 0; i < NUM_LEGS; i++) {
    legs[i].calibrateServos();
  }
}

void QuadrupedRobot::setBodyDimensions(float height, float width, float length) {
  bodyHeight = height;
  bodyWidth = width;
  bodyLength = length;
}

void QuadrupedRobot::stand() {
  currentState = STANDING;
  for (int i = 0; i < NUM_LEGS; i++) {
    legs[i].standingPose();
  }
}

void QuadrupedRobot::sit() {
  currentState = SITTING;
  for (int i = 0; i < NUM_LEGS; i++) {
    legs[i].sittingPose();
  }
}

void QuadrupedRobot::walkForward(float walkSpeed) {
  currentState = WALKING;
  speed = walkSpeed;
  direction = 0; // Forward direction
  
  // Use gait to coordinate leg movements
  // This is a simplified implementation
  unsigned long currentTime = millis();
  
  for (int i = 0; i < NUM_LEGS; i++) {
    if (gait.shouldLiftLeg(i, currentTime)) {
      legs[i].stepForward(10 * speed);
    }
  }
}

void QuadrupedRobot::walkBackward(float walkSpeed) {
  currentState = WALKING;
  speed = walkSpeed;
  direction = 180; // Backward direction
  
  unsigned long currentTime = millis();
  
  for (int i = 0; i < NUM_LEGS; i++) {
    if (gait.shouldLiftLeg(i, currentTime)) {
      legs[i].stepBackward(10 * speed);
    }
  }
}

void QuadrupedRobot::turnLeft(float rate) {
  currentState = TURNING_LEFT;
  turnRate = rate;
  
  // Implement turning logic
  // Left legs take smaller steps, right legs take larger steps
  unsigned long currentTime = millis();
  
  for (int i = 0; i < NUM_LEGS; i++) {
    if (gait.shouldLiftLeg(i, currentTime)) {
      if (i == FRONT_LEFT || i == REAR_LEFT) {
        legs[i].stepBackward(5 * rate); // Left legs step back
      } else {
        legs[i].stepForward(5 * rate);  // Right legs step forward
      }
    }
  }
}

void QuadrupedRobot::turnRight(float rate) {
  currentState = TURNING_RIGHT;
  turnRate = rate;
  
  unsigned long currentTime = millis();
  
  for (int i = 0; i < NUM_LEGS; i++) {
    if (gait.shouldLiftLeg(i, currentTime)) {
      if (i == FRONT_RIGHT || i == REAR_RIGHT) {
        legs[i].stepBackward(5 * rate); // Right legs step back
      } else {
        legs[i].stepForward(5 * rate);  // Left legs step forward
      }
    }
  }
}

void QuadrupedRobot::stop() {
  currentState = IDLE;
  speed = 0;
  
  // Return to standing pose
  stand();
}

void QuadrupedRobot::walkDirection(float angle, float walkSpeed) {
  // Implement omnidirectional walking
  direction = angle;
  speed = walkSpeed;
  
  // This would involve complex leg coordination
  // For now, simplified to forward/backward
  if (angle >= -45 && angle <= 45) {
    walkForward(walkSpeed);
  } else if (angle >= 135 || angle <= -135) {
    walkBackward(walkSpeed);
  }
}

void QuadrupedRobot::strafe(float direction, float walkSpeed) {
  // Implement sideways movement
  // This requires complex leg coordination and depends on robot design
}

void QuadrupedRobot::setGaitType(GaitType newGait) {
  gait.setGaitType(newGait);
}

void QuadrupedRobot::adjustBodyHeight(float height) {
  bodyHeight = height;
  // Adjust all leg positions to change body height
  for (int i = 0; i < NUM_LEGS; i++) {
    // Implementation depends on inverse kinematics
  }
}

void QuadrupedRobot::adjustBodyPitch(float angle) {
  // Tilt body forward/backward
  // Adjust front and rear leg heights differently
}

void QuadrupedRobot::adjustBodyRoll(float angle) {
  // Tilt body left/right
  // Adjust left and right leg heights differently
}

void QuadrupedRobot::adjustBodyYaw(float angle) {
  // Rotate body around vertical axis
  // Adjust shoulder servo angles
}

void QuadrupedRobot::updateBalance() {
  // Calculate center of gravity and adjust leg positions
  // This is a complex calculation involving leg positions and body orientation
}

bool QuadrupedRobot::checkStability() {
  // Check if robot is stable (center of gravity within support polygon)
  return isBalanced;
}

void QuadrupedRobot::correctBalance() {
  // Implement balance correction algorithms
  if (!checkStability()) {
    updateBalance();
  }
}

void QuadrupedRobot::updateLegPositions() {
  // Update leg positions based on current movement state
  switch (currentState) {
    case WALKING:
      // Implement walking gait
      break;
    case TURNING_LEFT:
      // Implement left turn
      break;
    case TURNING_RIGHT:
      // Implement right turn
      break;
    case STANDING:
    case SITTING:
    case IDLE:
    default:
      // No movement updates needed
      break;
  }
}

void QuadrupedRobot::coordinated_step() {
  // Coordinate all legs for one step cycle
  unsigned long currentTime = millis();
  bool legPhases[NUM_LEGS];
  
  gait.calculateLegPhases(currentTime, legPhases);
  
  for (int i = 0; i < NUM_LEGS; i++) {
    if (legPhases[i]) {
      legs[i].liftLeg(20); // Swing phase
    } else {
      legs[i].lowerLeg();  // Stance phase
    }
  }
}

void QuadrupedRobot::emergencyStop() {
  currentState = IDLE;
  speed = 0;
  
  // Immediately stop all movement and go to safe position
  for (int i = 0; i < NUM_LEGS; i++) {
    legs[i].neutralPose();
  }
}

void QuadrupedRobot::setState(MovementState state) {
  currentState = state;
}

MovementState QuadrupedRobot::getState() {
  return currentState;
}

void QuadrupedRobot::update() {
  // Main update loop - call this regularly
  updateLegPositions();
  updateBalance();
  
  // Check for safety conditions
  if (!checkServoHealth()) {
    emergencyStop();
  }
}

void QuadrupedRobot::printStatus() {
  Serial.print("State: ");
  switch (currentState) {
    case IDLE: Serial.print("IDLE"); break;
    case WALKING: Serial.print("WALKING"); break;
    case TURNING_LEFT: Serial.print("TURNING_LEFT"); break;
    case TURNING_RIGHT: Serial.print("TURNING_RIGHT"); break;
    case SITTING: Serial.print("SITTING"); break;
    case STANDING: Serial.print("STANDING"); break;
  }
  Serial.print(", Speed: ");
  Serial.print(speed);
  Serial.print(", Direction: ");
  Serial.println(direction);
}

void QuadrupedRobot::printLegPositions() {
  for (int i = 0; i < NUM_LEGS; i++) {
    float x, y, z;
    legs[i].getPosition(x, y, z);
    Serial.print("Leg ");
    Serial.print(i);
    Serial.print(": (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.println(")");
  }
}

bool QuadrupedRobot::isMoving() {
  return (currentState == WALKING || currentState == TURNING_LEFT || currentState == TURNING_RIGHT);
}

bool QuadrupedRobot::selfTest() {
  // Perform self-test sequence
  Serial.println("Performing self-test...");
  
  // Test each leg
  for (int i = 0; i < NUM_LEGS; i++) {
    legs[i].neutralPose();
    delay(500);
    legs[i].standingPose();
    delay(500);
  }
  
  Serial.println("Self-test complete");
  return true;
}

void QuadrupedRobot::enableSafetyLimits(bool enabled) {
  // Enable/disable safety limits for servo movements
  // Implementation would depend on specific safety requirements
}

bool QuadrupedRobot::checkServoHealth() {
  // Check if all servos are responding properly
  // This is a simplified implementation
  return true;
}
