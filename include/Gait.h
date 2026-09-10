#ifndef GAIT_H
#define GAIT_H

#include <Arduino.h>

#define NUM_LEGS 4

// Gait types
enum GaitType {
  TROT,
  WALK,
  BOUND,
  PACE
};

class Gait {
private:
  GaitType currentGait;
  float stepSize;
  float stepHeight;
  int stepDuration;
  int phaseOffset[NUM_LEGS];
  
public:
  Gait();
  void init(GaitType gait = TROT);
  void setGaitType(GaitType gait);
  void setStepParameters(float size, float height, int duration);
  void calculateLegPhases(int currentTime, bool legPhases[NUM_LEGS]);
  bool shouldLiftLeg(int legIndex, int currentTime);
  float getStepProgress(int legIndex, int currentTime);
  void setStepSize(float size);
  void setStepHeight(float height);
  GaitType getCurrentGait();
};

#endif
