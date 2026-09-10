#include "MovementController.h"

MovementController::MovementController() {
  robot = nullptr;
  lastUpdateTime = 0;
  currentSequenceStep = 0;
  sequenceActive = false;
}

void MovementController::init(QuadrupedRobot* robotInstance) {
  robot = robotInstance;
  lastUpdateTime = millis();
}

void MovementController::performStartupSequence() {
  if (robot == nullptr) return;
  
  Serial.println("Performing startup sequence...");
  
  // Self-test
  robot->selfTest();
  delay(500);
  
  // Move to standing position
  robot->stand();
  delay(1000);
  
  // Quick movement test
  robot->adjustBodyHeight(120);
  delay(500);
  robot->adjustBodyHeight(150);
  delay(500);
  
  Serial.println("Startup sequence complete!");
}

void MovementController::performShutdownSequence() {
  if (robot == nullptr) return;
  
  Serial.println("Performing shutdown sequence...");
  
  // Stop any movement
  robot->stop();
  delay(500);
  
  // Move to sitting position
  robot->sit();
  delay(1000);
  
  Serial.println("Shutdown sequence complete!");
}

void MovementController::performStretchSequence() {
  if (robot == nullptr) return;
  
  Serial.println("Performing stretch sequence...");
  
  // Cycle through different poses
  robot->stand();
  delay(1000);
  
  robot->sit();
  delay(1000);
  
  robot->stand();
  delay(500);
  
  Serial.println("Stretch sequence complete!");
}

void MovementController::performDanceSequence() {
  if (robot == nullptr) return;
  
  Serial.println("Dancing!");
  
  // Simple dance routine
  for (int i = 0; i < 3; i++) {
    robot->turnLeft(0.5);
    delay(1000);
    robot->turnRight(0.5);
    delay(1000);
  }
  
  robot->stop();
  Serial.println("Dance complete!");
}

void MovementController::respondToCommand(char command) {
  if (robot == nullptr) return;
  
  Serial.print("Command received: ");
  Serial.println(command);
  
  switch (command) {
    case 'w':
    case 'W':
      robot->walkForward(1.0);
      Serial.println("Walking forward");
      break;
      
    case 's':
    case 'S':
      robot->walkBackward(1.0);
      Serial.println("Walking backward");
      break;
      
    case 'a':
    case 'A':
      robot->turnLeft(1.0);
      Serial.println("Turning left");
      break;
      
    case 'd':
    case 'D':
      robot->turnRight(1.0);
      Serial.println("Turning right");
      break;
      
    case 'q':
    case 'Q':
      robot->sit();
      Serial.println("Sitting");
      break;
      
    case 'e':
    case 'E':
      robot->stand();
      Serial.println("Standing");
      break;
      
    case 'r':
    case 'R':
      robot->stop();
      Serial.println("Stopping");
      break;
      
    case 't':
    case 'T':
      performStretchSequence();
      break;
      
    case 'x':
    case 'X':
      performDanceSequence();
      break;
      
    case 'z':
    case 'Z':
      robot->setGaitType(TROT);
      Serial.println("Gait set to TROT");
      break;
      
    case 'c':
    case 'C':
      robot->setGaitType(WALK);
      Serial.println("Gait set to WALK");
      break;
      
    case 'p':
    case 'P':
      robot->printStatus();
      break;
      
    case 'l':
    case 'L':
      robot->printLegPositions();
      break;
      
    case '!':
      emergencyShutdown();
      break;
      
    default:
      Serial.println("Unknown command!");
      Serial.println("Available commands:");
      Serial.println("w/W - Walk forward");
      Serial.println("s/S - Walk backward");
      Serial.println("a/A - Turn left");
      Serial.println("d/D - Turn right");
      Serial.println("q/Q - Sit");
      Serial.println("e/E - Stand");
      Serial.println("r/R - Stop");
      Serial.println("t/T - Stretch");
      Serial.println("x/X - Dance");
      Serial.println("z/Z - Trot gait");
      Serial.println("c/C - Walk gait");
      Serial.println("p/P - Print status");
      Serial.println("l/L - Print leg positions");
      Serial.println("! - Emergency stop");
      break;
  }
}

void MovementController::autonomousWander() {
  if (robot == nullptr) return;
  
  // Simple autonomous behavior
  static unsigned long lastDirectionChange = 0;
  static int currentDirection = 0; // 0=forward, 1=left, 2=right
  
  unsigned long currentTime = millis();
  
  // Change direction every 3 seconds
  if (currentTime - lastDirectionChange > 3000) {
    currentDirection = random(0, 3);
    lastDirectionChange = currentTime;
    
    switch (currentDirection) {
      case 0:
        robot->walkForward(0.8);
        Serial.println("Auto: Walking forward");
        break;
      case 1:
        robot->turnLeft(0.6);
        Serial.println("Auto: Turning left");
        break;
      case 2:
        robot->turnRight(0.6);
        Serial.println("Auto: Turning right");
        break;
    }
  }
}

void MovementController::followPath(float waypoints[][2], int numWaypoints) {
  // Implementation for following a predefined path
  // This would involve calculating directions and distances to waypoints
}

void MovementController::handleObstacle() {
  if (robot == nullptr) return;
  
  Serial.println("Obstacle detected! Taking evasive action...");
  
  // Simple obstacle avoidance
  robot->stop();
  delay(500);
  robot->walkBackward(0.5);
  delay(1000);
  robot->turnRight(1.0);
  delay(1500);
  robot->walkForward(0.8);
}

void MovementController::recoverFromFall() {
  if (robot == nullptr) return;
  
  Serial.println("Fall detected! Attempting recovery...");
  
  // Recovery sequence
  robot->emergencyStop();
  delay(1000);
  
  // Try to get back up
  performStartupSequence();
}

void MovementController::emergencyShutdown() {
  if (robot == nullptr) return;
  
  Serial.println("EMERGENCY SHUTDOWN ACTIVATED!");
  
  robot->emergencyStop();
  sequenceActive = false;
  
  Serial.println("Robot stopped. Send 'e' to restart.");
}

void MovementController::update() {
  if (robot == nullptr) return;
  
  unsigned long currentTime = millis();
  
  // Update timing
  lastUpdateTime = currentTime;
  
  // Handle any ongoing sequences
  if (sequenceActive) {
    // Implementation for multi-step sequences
  }
  
  // Could add autonomous behaviors here
  // autonomousWander();
}
