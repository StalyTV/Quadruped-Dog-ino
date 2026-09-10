#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Default I²C address for PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150  // Minimum pulse count (approx 0°)
#define SERVOMAX  600  // Maximum pulse count (approx 180°) 
#define FRQ       60  // Servo refresh rate (Hz) - Higher for smoother movement
#define SERVO_FR_1  0    // Servo on channel 0
#define SERVO_FR_2  1    // Servo on channel 1
#define SERVO_FR_3  2    // Servo on channel 2

// Variable to control servo position
int servoPosition = 200;  // Starting position

// Function prototypes
void updateServoPositions();
void printCurrentPosition();

void setup() {
  Serial.begin(9600);
  Serial.println("PCA9685 Servo Control with User Input");
  Serial.println("Commands:");
  Serial.println("'w' or 'W' = Increase position by 100");
  Serial.println("'d' or 'D' = Increase position by 10 (fine adjustment)");
  Serial.println("'s' or 'S' = Decrease position by 100");
  Serial.println("'a' or 'A' = Decrease position by 10 (fine adjustment)");
  Serial.println("'r' or 'R' = Reset to starting position (200)");
  Serial.println("Send commands through Serial Monitor...");

  pwm.begin();
  pwm.setPWMFreq(FRQ);
  
  // Set initial position
  updateServoPositions();
  printCurrentPosition();
}

void loop() {
  // Check for user input
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case 'w':
      case 'W':
        servoPosition += 100;
        if (servoPosition > SERVOMAX) {
          servoPosition = SERVOMAX;
          Serial.println("Maximum position reached!");
        }
        updateServoPositions();
        printCurrentPosition();
        break;

      case 'd':
      case 'D':
        servoPosition += 10;
        if (servoPosition > SERVOMAX) {
          servoPosition = SERVOMAX;
          Serial.println("Maximum position reached!");
        }
        updateServoPositions();
        printCurrentPosition();
        break;  
        
      case 's':
      case 'S':
        servoPosition -= 100;
        if (servoPosition < SERVOMIN) {
          servoPosition = SERVOMIN;
          Serial.println("Minimum position reached!");
        }
        updateServoPositions();
        printCurrentPosition();
        break;
       
      case 'a':
      case 'A':
        servoPosition -= 10;
        if (servoPosition < SERVOMIN) {
          servoPosition = SERVOMIN;
          Serial.println("Minimum position reached!");
        }
        updateServoPositions();
        printCurrentPosition();
        break;  


      case 'r':
      case 'R':
        servoPosition = 200;
        updateServoPositions();
        Serial.println("Position reset to starting value");
        printCurrentPosition();
        break;
        
      default:
        Serial.println("Unknown command. Use 'w'/'d' (up), 's'/'a' (down), or 'r' (reset)");
        break;
    }
  }
}

// Function to update all servo positions
void updateServoPositions() {
  pwm.setPWM(SERVO_FR_1, 0, servoPosition);
  pwm.setPWM(SERVO_FR_2, 0, servoPosition);
  pwm.setPWM(SERVO_FR_3, 0, servoPosition);
}

// Function to print current position
void printCurrentPosition() {
  Serial.print("Current servo position: ");
  Serial.print(servoPosition);
  
  // Calculate approximate angle (rough estimation)
  int angle = map(servoPosition, SERVOMIN, SERVOMAX, 0, 180);
  Serial.print(" (≈");
  Serial.print(angle);
  Serial.println("°)");
}