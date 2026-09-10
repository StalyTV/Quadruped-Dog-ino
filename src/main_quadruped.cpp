#include <Arduino.h>
#include "QuadrupedRobot.h"
#include "MovementController.h"
#include "SensorManager.h"

QuadrupedRobot robot;
MovementController controller;
SensorManager sensors;

void setup() {
  Serial.begin(9600);
  Serial.println("Quadruped Robot Initializing...");
  
  // Initialize robot systems
  robot.init();
  controller.init(&robot);
  sensors.init();
  
  // Perform startup sequence
  controller.performStartupSequence();
  
  Serial.println("Robot ready for commands!");
  Serial.println("Available commands:");
  Serial.println("w/W=forward, s/S=backward, a/A=left, d/D=right");
  Serial.println("q/Q=sit, e/E=stand, r/R=stop, t/T=stretch, x/X=dance");
  Serial.println("z/Z=trot gait, c/C=walk gait, p/P=status, l/L=leg positions");
  Serial.println("!=emergency stop");
}

void loop() {
  // Check for user commands
  if (Serial.available() > 0) {
    char command = Serial.read();
    controller.respondToCommand(command);
  }
  
  // Update robot systems
  robot.update();
  controller.update();
  sensors.update();
  
  // Small delay for stability
  delay(10);
}
