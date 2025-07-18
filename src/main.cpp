#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Function declarations
void moveServoToPulse(int channel, int pulse, int* currentPulse);
void initializeMPU6050();
void readMPU6050AndControlServos();

// Create PCA9685 object (uses hardware I2C on pins 20, 21)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Servo configuration
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Servo channels on PCA9685
#define SERVO_CHANNEL_0 0
#define SERVO_CHANNEL_1 1

// Servo pulse length limits (starting at 100)
#define SERVOMIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // This is the 'maximum' pulse length count (out of 4096)

// Current servo positions
int currentPulse0 = 100;  // Servo on channel 0
int currentPulse1 = 100;  // Servo on channel 1

// MPU6050 variables
unsigned long lastMPUUpdate = 0;
const unsigned long MPU_UPDATE_INTERVAL = 10; // Update every 10ms for very smooth servo control
unsigned long lastSerialUpdate = 0;
const unsigned long SERIAL_UPDATE_INTERVAL = 500; // Print to serial every 500ms to avoid lag

// Smoothing variables for reducing servo jitter
const int SMOOTHING_SAMPLES = 10; // Increased from 5 to 10 for more smoothing
float pitchHistory[SMOOTHING_SAMPLES];
float rollHistory[SMOOTHING_SAMPLES];
int historyIndex = 0;
bool historyFilled = false;

// Additional smoothing variables
float lastSmoothedPitch = 0;
float lastSmoothedRoll = 0;
const float ALPHA = 0.7; // Exponential smoothing factor (0.0 = no change, 1.0 = no smoothing)

// Function to initialize MPU6050
void initializeMPU6050() {
  Serial.println("Initializing MPU6050...");
  
  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  // Configure accelerometer (±2g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  // Configure gyroscope (±250°/s)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  delay(100);
  Serial.println("MPU6050 initialized!");
}

// Function to smooth values using moving average + exponential smoothing
float smoothValue(float newValue, float* history, bool isRoll = false) {
  // Add new value to history
  if (isRoll) {
    rollHistory[historyIndex] = newValue;
  } else {
    pitchHistory[historyIndex] = newValue;
  }
  
  // Calculate moving average
  float sum = 0;
  int count = historyFilled ? SMOOTHING_SAMPLES : (historyIndex + 1);
  
  for (int i = 0; i < count; i++) {
    if (isRoll) {
      sum += rollHistory[i];
    } else {
      sum += pitchHistory[i];
    }
  }
  
  float movingAverage = sum / count;
  
  // Apply exponential smoothing on top of moving average
  float* lastValue = isRoll ? &lastSmoothedRoll : &lastSmoothedPitch;
  
  if (count == 1) {
    // First reading, use the raw value
    *lastValue = movingAverage;
    return movingAverage;
  } else {
    // Apply exponential smoothing: new = alpha * current + (1-alpha) * previous
    float smoothedValue = ALPHA * movingAverage + (1.0 - ALPHA) * (*lastValue);
    *lastValue = smoothedValue;
    return smoothedValue;
  }
}

// Function to move servo to specific pulse value
void moveServoToPulse(int channel, int pulse, int* currentPulse) {
  // Constrain pulse to valid range
  pulse = constrain(pulse, SERVOMIN, SERVOMAX);
  *currentPulse = pulse;
  
  pwm.setPWM(channel, 0, pulse);
  
//   Serial.print("Servo CH");
//   Serial.print(channel);
//   Serial.print(" moved to pulse: ");
//   Serial.print(pulse);
//   Serial.print(" (Range: ");
//   Serial.print(SERVOMIN);
//   Serial.print("-");
//   Serial.print(SERVOMAX);
//   Serial.println(")");
}

// Function to read MPU6050 and control servos automatically
void readMPU6050AndControlServos() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // Start at ACCEL_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);  // Read 6 registers (accelerometer only)
  
  if (Wire.available() >= 6) {
    // Accelerometer data (registers 0x3B-0x40)
    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();
    int16_t AcZ = Wire.read() << 8 | Wire.read();
    
    // Convert to g-force (±2g range, sensitivity: 16384 LSB/g)
    float accelX = AcX / 16384.0;
    float accelY = AcY / 16384.0;
    float accelZ = AcZ / 16384.0;
    
    // Calculate tilt angles in degrees
    float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
    float roll = atan2(accelY, accelZ) * 180.0 / PI;
    
    // Apply smoothing to reduce jitter
    float smoothedPitch = smoothValue(pitch, pitchHistory, false);
    float smoothedRoll = smoothValue(roll, rollHistory, true);
    
    // Apply deadband to prevent micro-movements when nearly level
    const float DEADBAND = 1.0; // degrees
    if (abs(smoothedPitch) < DEADBAND) smoothedPitch = 0;
    if (abs(smoothedRoll) < DEADBAND) smoothedRoll = 0;
    
    // Update history index
    historyIndex = (historyIndex + 1) % SMOOTHING_SAMPLES;
    if (historyIndex == 0) historyFilled = true;
    
    // Constrain smoothed angles to ±90 degrees
    smoothedPitch = constrain(smoothedPitch, -90, 90);
    smoothedRoll = constrain(smoothedRoll, -90, 90);
    
    // Map Roll to Servo 0 (channel 0): -90° to 90° mapped to pulse 100-500
    int servo0Pulse = map(smoothedRoll, -90, 90, SERVOMIN, SERVOMAX);
    
    // Map Pitch to Servo 1 (channel 1): -90° to 90° mapped to pulse 100-500
    int servo1Pulse = map(smoothedPitch, -90, 90, 250, SERVOMAX);
    
    // Only update servos if the change is significant enough (reduces micro-jitter)
    const int MIN_CHANGE = 1; // Reduced from 2 to 1 for smoother transitions
    
    if (abs(servo0Pulse - currentPulse0) >= MIN_CHANGE) {
      moveServoToPulse(SERVO_CHANNEL_0, servo0Pulse, &currentPulse0);
    }
    
    if (abs(servo1Pulse - currentPulse1) >= MIN_CHANGE) {
      moveServoToPulse(SERVO_CHANNEL_1, servo1Pulse, &currentPulse1);
    }
  }
}


void setup() {
  Serial.begin(9600);
  delay(1000);  // Give serial time to initialize
  
  Serial.println("MPU6050 Automatic Servo Control");
  Serial.println("Connect servo 0 to channel 0 and servo 1 to channel 1 on PCA9685");
  Serial.println("MPU6050 connected to I2C pins 20 (SDA) and 21 (SCL)");
  Serial.println("Roll controls Servo 0, Pitch controls Servo 1");

  // Initialize hardware I2C for both PCA9685 and MPU6050 (pins 20, 21 on Arduino Mega)
  Wire.begin();
  Wire.setClock(400000); // Set I2C to fast mode (400kHz) for faster communication
  
  // Initialize MPU6050 first
  initializeMPU6050();
  
  // Initialize smoothing arrays
  for (int i = 0; i < SMOOTHING_SAMPLES; i++) {
    pitchHistory[i] = 0;
    rollHistory[i] = 0;
  }
  
  // Initialize exponential smoothing variables
  lastSmoothedPitch = 0;
  lastSmoothedRoll = 0;
  
  // Initialize PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  delay(100);  // Delay for PCA9685 initialization
  
  // Initialize both servos to center position (pulse 300 = middle of 100-500 range)
  Serial.println("Initializing servos to center position...");
  moveServoToPulse(SERVO_CHANNEL_0, 300, &currentPulse0);
  moveServoToPulse(SERVO_CHANNEL_1, 300, &currentPulse1);
  delay(1000);
  
  Serial.println("Setup complete! Automatic servo control active.");
  Serial.println("Tilt the MPU6050 to control the servos.");
  Serial.println("");
}

void loop() {
  // Update servos based on MPU6050 readings as fast as possible
  unsigned long currentTime = millis();
  if (currentTime - lastMPUUpdate >= MPU_UPDATE_INTERVAL) {
    readMPU6050AndControlServos();
    lastMPUUpdate = currentTime;
  }
  
  // No delay - run as fast as possible for maximum responsiveness
}