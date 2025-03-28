#include <Arduino.h>
#include <Wire.h>

// Define sensor pins (adjust these according to your wiring)
#define LEFT_SENSOR_PIN 25
#define RIGHT_SENSOR_PIN 3

// Update interval in milliseconds (industry standard ~10Hz)
#define UPDATE_INTERVAL 1000

// I2C device address (change this as needed)
#define I2C_DEV_ADDR 0x55  // Example address (you need to set the address for your device)

// Variables for left wheel measurement
volatile unsigned long leftLastPulseTime = 0;
volatile unsigned long leftPulseInterval = 0;

// Variables for right wheel measurement
volatile unsigned long rightLastPulseTime = 0;
volatile unsigned long rightPulseInterval = 0;

static unsigned long lastLeftPulseCheck = 0;
static unsigned long lastRightPulseCheck = 0;


void IRAM_ATTR leftSensorISR() {
  unsigned long now = micros();
  if (leftLastPulseTime > 0) {
    leftPulseInterval = now - leftLastPulseTime;
  }
  leftLastPulseTime = now;
  lastLeftPulseCheck = millis();  // Update timestamp ONLY when pulse occurs
}

void IRAM_ATTR rightSensorISR() {
  unsigned long now = micros();
  if (rightLastPulseTime > 0) {
    rightPulseInterval = now - rightLastPulseTime;
  }
  rightLastPulseTime = now;
  lastRightPulseCheck = millis();  // Update timestamp ONLY when pulse occurs
}

// Helper function to calculate RPM from pulse interval (in microseconds)
// RPM = (1 / (pulseInterval in seconds)) * 60
float calculateRPM(unsigned long pulseInterval) {
  if (pulseInterval == 0) return 0.0;
  float timePerRevolution = pulseInterval / 1000000.0; // Convert microseconds to seconds
  float rpm = (1.0 / timePerRevolution) * 60.0;
  return rpm;
}

// Function to send RPM values over I2C
void sendI2C(float leftRPM, float rightRPM) {
  // Send the RPM values (convert them to two 4-byte integers for each)
  int left = (int)leftRPM;
  int right = (int)rightRPM;

  // Start I2C transmission
  Wire.beginTransmission(I2C_DEV_ADDR);  // Address of the I2C device
  Wire.write((byte)(left >> 24));  // Send left RPM high byte
  Wire.write((byte)(left >> 16));  // Send left RPM next byte
  Wire.write((byte)(left >> 8));   // Send left RPM next byte
  Wire.write((byte)(left & 0xFF)); // Send left RPM low byte

  Wire.write((byte)(right >> 24));  // Send right RPM high byte
  Wire.write((byte)(right >> 16));  // Send right RPM next byte
  Wire.write((byte)(right >> 8));   // Send right RPM next byte
  Wire.write((byte)(right & 0xFF)); // Send right RPM low byte
  Wire.endTransmission();  // End I2C transmission

  Serial.print("I2C Sent - LEFT: ");
  Serial.print(left);
  Serial.print(" RIGHT: ");
  Serial.println(right);
}

void setup() {
  Serial.begin(115200);

  // Initialize sensor pins with pull-up resistors
  pinMode(LEFT_SENSOR_PIN, INPUT_PULLUP);
  pinMode(RIGHT_SENSOR_PIN, INPUT_PULLUP);

  // Attach interrupts to sensor pins (trigger on falling edge)
  attachInterrupt(digitalPinToInterrupt(LEFT_SENSOR_PIN), leftSensorISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SENSOR_PIN), rightSensorISR, FALLING);

  // Initialize I2C communication
  Wire.begin();  // Start the I2C bus (this will be the master)
  Serial.println("I2C bus started");
}

void loop() {
  static unsigned long lastUpdateTime = 0;
  const unsigned long timeout = 2 * UPDATE_INTERVAL; // Allow enough time for slow pulses

  // Check if it is time to update (every UPDATE_INTERVAL)
  if (millis() - lastUpdateTime >= UPDATE_INTERVAL) {
    lastUpdateTime = millis();

    // If no new pulses were detected in the timeout period, reset the interval
    if ((millis() - lastLeftPulseCheck) >= timeout) {
      leftPulseInterval = 0;  // No recent pulse, force RPM to zero
    }
    if ((millis() - lastRightPulseCheck) >= timeout) {
      rightPulseInterval = 0;  // No recent pulse, force RPM to zero
    }

    // Calculate RPM from the most recent pulse intervals
    float leftRPM = calculateRPM(leftPulseInterval);
    float rightRPM = calculateRPM(rightPulseInterval);

    // Transmit the RPM values over I2C
    sendI2C(leftRPM, rightRPM);
  }
}
