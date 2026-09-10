#ifndef MOVEMENT_CONTROLLER_H
#define MOVEMENT_CONTROLLER_H

#include "QuadrupedRobot.h"

class MovementController {
private:
  QuadrupedRobot* robot;
  unsigned long lastUpdateTime;
  int currentSequenceStep;
  bool sequenceActive;

public:
  MovementController();
  void init(QuadrupedRobot* robotInstance);
  
  // Predefined movement sequences
  void performStartupSequence();
  void performShutdownSequence();
  void performStretchSequence();
  void performDanceSequence();
  
  // Interactive behaviors
  void respondToCommand(char command);
  void autonomousWander();
  void followPath(float waypoints[][2], int numWaypoints);
  
  // Safety behaviors
  void handleObstacle();
  void recoverFromFall();
  void emergencyShutdown();
  
  void update();
};

#endif
