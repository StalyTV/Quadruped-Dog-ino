#include "Gait.h"

Gait::Gait() {
  currentGait = TROT;
  stepSize = 10.0;
  stepHeight = 20.0;
  stepDuration = 1000; // milliseconds
  
  // Initialize phase offsets
  for (int i = 0; i < NUM_LEGS; i++) {
    phaseOffset[i] = 0;
  }
}

void Gait::init(GaitType gait) {
  setGaitType(gait);
}

void Gait::setGaitType(GaitType gait) {
  currentGait = gait;
  
  // Set phase offsets based on gait type
  switch (gait) {
    case TROT:
      // Diagonal legs move together
      phaseOffset[0] = 0;    // Front Left
      phaseOffset[1] = 500;  // Front Right (opposite phase)
      phaseOffset[2] = 500;  // Rear Left (opposite phase)
      phaseOffset[3] = 0;    // Rear Right
      break;
      
    case WALK:
      // Sequential leg movement
      phaseOffset[0] = 0;    // Front Left
      phaseOffset[1] = 250;  // Front Right
      phaseOffset[2] = 500;  // Rear Left
      phaseOffset[3] = 750;  // Rear Right
      break;
      
    case BOUND:
      // Front legs together, rear legs together
      phaseOffset[0] = 0;    // Front Left
      phaseOffset[1] = 0;    // Front Right
      phaseOffset[2] = 500;  // Rear Left
      phaseOffset[3] = 500;  // Rear Right
      break;
      
    case PACE:
      // Same-side legs move together
      phaseOffset[0] = 0;    // Front Left
      phaseOffset[1] = 500;  // Front Right
      phaseOffset[2] = 0;    // Rear Left
      phaseOffset[3] = 500;  // Rear Right
      break;
  }
}

void Gait::setStepParameters(float size, float height, int duration) {
  stepSize = size;
  stepHeight = height;
  stepDuration = duration;
}

void Gait::calculateLegPhases(int currentTime, bool legPhases[NUM_LEGS]) {
  for (int i = 0; i < NUM_LEGS; i++) {
    int adjustedTime = (currentTime + phaseOffset[i]) % stepDuration;
    legPhases[i] = (adjustedTime < stepDuration / 2); // First half = swing phase
  }
}

bool Gait::shouldLiftLeg(int legIndex, int currentTime) {
  if (legIndex < 0 || legIndex >= NUM_LEGS) return false;
  
  int adjustedTime = (currentTime + phaseOffset[legIndex]) % stepDuration;
  return (adjustedTime < stepDuration / 2); // Lift during first half of cycle
}

float Gait::getStepProgress(int legIndex, int currentTime) {
  if (legIndex < 0 || legIndex >= NUM_LEGS) return 0.0;
  
  int adjustedTime = (currentTime + phaseOffset[legIndex]) % stepDuration;
  return (float)adjustedTime / stepDuration;
}

void Gait::setStepSize(float size) {
  stepSize = size;
}

void Gait::setStepHeight(float height) {
  stepHeight = height;
}

GaitType Gait::getCurrentGait() {
  return currentGait;
}
