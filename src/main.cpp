#include <Arduino.h>
#include <CAN.h>

//CODE FOR SENDING JOYSTICK DATA

// const int analogPin = 33;  // ADC-capable pin
// int analogValue = 0;
// char analogStr[5];     // Buffer to hold 4-digit string + null terminator
// unsigned long lastSendTime = 0;
// const unsigned long sendInterval = 50;  //change as per requirements
// void setup() {
//   Serial.begin(115200);
//   while (!Serial);
//   Serial.println("CAN Sender");
//   // Start the CAN bus at 500 kbps (500E3) OR 1 Mbps (1000E3)
//   if (!CAN.begin(500E3)) {
//     Serial.println("Starting CAN failed!");
//     while (1);
//   }
// }
// void loop() {
//   unsigned long currentMillis = millis();
//   analogValue = analogRead(analogPin);  // Read analog value (0-4095)
//    // Only send data if the value has changed significantly or 50ms has passed
//   // if (abs(analogValue - prevAnalogValue) > 15 || currentMillis - lastSendTime >= sendInterval) {
//   //   prevAnalogValue = analogValue;
//   //   lastSendTime = currentMillis;
//   sprintf(analogStr, "%04d", analogValue);  // Convert to 4-char string (e.g., "0345")
//   Serial.print("Sending Analog Value: ");
//   Serial.println(analogStr);  // Print value before sending
//   CAN.beginPacket(0x12);  // Send packet with ID 0x12
//   for (int i = 0; i < 4; i++) {
//     CAN.write(analogStr[i]);  // Send each digit as a byte
//   }
//   CAN.endPacket();
//   delay(200);  // Delay for receiver loop. should match. 
// }


//CODE FOR SEND ALL (look at test.cpp for most recent code)
// #include <Arduino.h>
// #include <CAN.h>
// #include <Wire.h>
// #include <Adafruit_LIS3DH.h>
// #include <Adafruit_Sensor.h>

// // Analog Sensor Setup
// const int analogPin = 33;  // ADC-capable pin
// int analogValue = 0;
// char analogStr[5];  // Buffer to hold 4-digit string + null terminator
// unsigned long lastAnalogSendTime = 0;
// const unsigned long analogSendInterval = 50;  // Send interval for analog sensor

// // Accelerometer Setup
// Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// // RPM Sensor Setup
// #define LEFT_SENSOR_PIN 25
// #define RIGHT_SENSOR_PIN 3
// volatile unsigned long leftLastPulseTime = 0;
// volatile unsigned long leftPulseInterval = 0;
// volatile unsigned long rightLastPulseTime = 0;
// volatile unsigned long rightPulseInterval = 0;
// unsigned long lastLeftPulseCheck = 0;
// unsigned long lastRightPulseCheck = 0;
// const unsigned long rpmUpdateInterval = 1000;  // Update interval for RPM sensors

// // Function Declarations
// void sendAnalogData();
// void sendAccelerometerData();
// void sendRPMData();
// void leftSensorISR();
// void rightSensorISR();
// float calculateRPM(unsigned long pulseInterval);

// // Setup Function
// void setup() {
//   Serial.begin(115200);
//   while (!Serial);

//   // Initialize CAN bus at 500 kbps
//   if (!CAN.begin(500E3)) {
//     Serial.println("Starting CAN failed!");
//     while (1);
//   }
//   Serial.println("CAN started!");

//   // Initialize Accelerometer
//   if (!lis.begin(0x18)) {  // Change to 0x19 for alternative I2C address
//     Serial.println("Could not start LIS3DH!");
//     while (1);
//   }
//   Serial.println("LIS3DH found!");

//   // Initialize RPM Sensors
//   pinMode(LEFT_SENSOR_PIN, INPUT_PULLUP);
//   pinMode(RIGHT_SENSOR_PIN, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(LEFT_SENSOR_PIN), leftSensorISR, FALLING);
//   attachInterrupt(digitalPinToInterrupt(RIGHT_SENSOR_PIN), rightSensorISR, FALLING);
// }

// // Loop Function
// void loop() {
//   unsigned long currentMillis = millis();

//   // Send Analog Sensor Data
//   if (currentMillis - lastAnalogSendTime >= analogSendInterval) {
//     sendAnalogData();
//     lastAnalogSendTime = currentMillis;
//   }

//   // Send Accelerometer Data
//   sendAccelerometerData();

//   // Send RPM Data
//   static unsigned long lastRPMUpdateTime = 0;
//   if (currentMillis - lastRPMUpdateTime >= rpmUpdateInterval) {
//     sendRPMData();
//     lastRPMUpdateTime = currentMillis;
//   }
// }

// // Function to Send Analog Sensor Data
// void sendAnalogData() {
//   analogValue = analogRead(analogPin);  // Read analog value (0-4095)
//   sprintf(analogStr, "%04d", analogValue);  // Convert to 4-char string (e.g., "0345")
//   Serial.print("Sending Analog Value: ");
//   Serial.println(analogStr);

//   CAN.beginPacket(0x12);  // Send packet with ID 0x12
//   for (int i = 0; i < 4; i++) {
//     CAN.write(analogStr[i]);  // Send each digit as a byte
//   }
//   CAN.endPacket();
// }

// // Function to Send Accelerometer Data
// void sendAccelerometerData() {
//   sensors_event_t event;
//   lis.getEvent(&event);

//   int16_t x = (int16_t)(event.acceleration.x * 100);  // Scale by 100 to preserve 2 decimal places
//   int16_t y = (int16_t)(event.acceleration.y * 100);
//   int16_t z = (int16_t)(event.acceleration.z * 100);

//   Serial.print("Sending XYZ Accel - X: ");
//   Serial.print(x);
//   Serial.print(" Y: ");
//   Serial.print(y);
//   Serial.print(" Z: ");
//   Serial.println(z);

//   CAN.beginPacket(0x18);  // Send packet with ID 0x18
//   CAN.write(x >> 8);  // High byte of X
//   CAN.write(x & 0xFF);  // Low byte of X
//   CAN.write(y >> 8);  // High byte of Y
//   CAN.write(y & 0xFF);  // Low byte of Y
//   CAN.write(z >> 8);  // High byte of Z
//   CAN.write(z & 0xFF);  // Low byte of Z
//   CAN.endPacket();
// }

// // Function to Send RPM Data
// void sendRPMData() {
//   const unsigned long timeout = 2 * rpmUpdateInterval;  // Allow enough time for slow pulses

//   // If no new pulses were detected in the timeout period, reset the interval
//   if ((millis() - lastLeftPulseCheck) >= timeout) {
//     leftPulseInterval = 0;  // No recent pulse, force RPM to zero
//   }
//   if ((millis() - lastRightPulseCheck) >= timeout) {
//     rightPulseInterval = 0;  // No recent pulse, force RPM to zero
//   }

//   // Calculate RPM from the most recent pulse intervals
//   float leftRPM = calculateRPM(leftPulseInterval);
//   float rightRPM = calculateRPM(rightPulseInterval);

//   Serial.print("Sending CAN - LEFT: ");
//   Serial.print(leftRPM);
//   Serial.print(" RIGHT: ");
//   Serial.println(rightRPM);

//   CAN.beginPacket(0x15);  // Send packet with ID 0x15
//   CAN.write((int)leftRPM >> 8);
//   CAN.write((int)leftRPM & 0xFF);
//   CAN.write((int)rightRPM >> 8);
//   CAN.write((int)rightRPM & 0xFF);
//   CAN.endPacket();
// }

// // ISR for Left RPM Sensor
// void IRAM_ATTR leftSensorISR() {
//   unsigned long now = micros();
//   if (leftLastPulseTime > 0) {
//     leftPulseInterval = now - leftLastPulseTime;
//   }
//   leftLastPulseTime = now;
//   lastLeftPulseCheck = millis();  // Update timestamp ONLY when pulse occurs
// }

// // ISR for Right RPM Sensor
// void IRAM_ATTR rightSensorISR() {
//   unsigned long now = micros();
//   if (rightLastPulseTime > 0) {
//     rightPulseInterval = now - rightLastPulseTime;
//   }
//   rightLastPulseTime = now;
//   lastRightPulseCheck = millis();  // Update timestamp ONLY when pulse occurs
// }

// // Helper Function to Calculate RPM
// float calculateRPM(unsigned long pulseInterval) {
//   if (pulseInterval == 0) return 0.0;
//   float timePerRevolution = pulseInterval / 1000000.0;  // Convert microseconds to seconds
//   float rpm = (1.0 / timePerRevolution) * 60.0;
//   return rpm;
// }
