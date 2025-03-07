//Function INST: 
void sendCAN(float leftRPM, float rightRPM);  // Function prototype

// #include <CAN.h>

// const int leftSensorPin = 25;   // Left photointerruptor pin
// const int rightSensorPin = 26;  // Right photointerruptor pin

// volatile int leftRevolutions = 0;
// volatile int rightRevolutions = 0;

// unsigned long lastSendTime = 0;
// const unsigned long sendInterval = 100; // Send data every 100ms (10Hz)
// const int timeWindow = 1000;  // 1-second interval for RPM calculation
// unsigned long startTime = 0;

// // Interrupt handlers
// void IRAM_ATTR leftInterrupt() {
//   leftRevolutions++;
// }

// void IRAM_ATTR rightInterrupt() {
//   rightRevolutions++;
// }

// void setup() {
//   Serial.begin(115200);
//   while (!Serial);

//   Serial.println("ESP32 CAN RPM Sender");

//   if (!CAN.begin(500E3)) {  // Initialize CAN at 500kbps
//     Serial.println("CAN Init Failed!");
//     while (1);
//   }

//   // Attach interrupts
//   attachInterrupt(digitalPinToInterrupt(leftSensorPin), leftInterrupt, RISING);
//   attachInterrupt(digitalPinToInterrupt(rightSensorPin), rightInterrupt, RISING);

//   startTime = millis();
// }

// void loop() {
//   unsigned long currentMillis = millis();

//   if (currentMillis - startTime >= timeWindow) {
//     float leftRPM = (leftRevolutions * 60000.0) / timeWindow;  // Calculate RPM
//     float rightRPM = (rightRevolutions * 60000.0) / timeWindow;

//     Serial.print("LEFT RPM: "); Serial.print(leftRPM);
//     Serial.print(" | RIGHT RPM: "); Serial.println(rightRPM);

//     sendCAN(leftRPM, rightRPM);  // Send RPM values via CAN

//     // Reset counters and timestamp
//     leftRevolutions = 0;
//     rightRevolutions = 0;
//     startTime = millis();
//   }
// }

// // Function to send RPM data via CAN bus
// void sendCAN(int leftRPM, int rightRPM) {
//   CAN.beginPacket(0x15);  // Using CAN ID 0x15
//   CAN.write(leftRPM >> 8);   // Send high byte
//   CAN.write(leftRPM & 0xFF); // Send low byte
//   CAN.write(rightRPM >> 8);
//   CAN.write(rightRPM & 0xFF);
//   CAN.endPacket();
// }
// const int digitalPin = 25;  // Change to your desired GPIO pin

// void setup() {
//   Serial.begin(115200);
//   pinMode(digitalPin, INPUT);  // Set the pin as an input
// }

// void loop() {
//   int digitalValue = digitalRead(digitalPin);  // Read the digital state (HIGH or LOW)
//   Serial.print("Digital Value: ");
//   Serial.println(digitalValue);
//   delay(500);  // Delay for readability
// }


// #include <CAN.h>

// const int leftSensorPin = 25;   // Left photointerruptor pin
// const int rightSensorPin = 26;  // Right photointerruptor pin

// volatile unsigned long lastLeftPulseTime = 0;
// volatile unsigned long lastRightPulseTime = 0;
// volatile float leftRPM = 0, rightRPM = 0;

// // Interrupt Service Routine (ISR) for left sensor
// void IRAM_ATTR leftInterrupt() {
//   unsigned long currentTime = micros();  // Get current time in microseconds
//   if (lastLeftPulseTime > 0) {
//     unsigned long timeDiff = currentTime - lastLeftPulseTime;  // Time between pulses
//     if (timeDiff > 0) {
//       leftRPM = (60000000.0 / timeDiff);  // Convert period to RPM
//     }
//   }
//   lastLeftPulseTime = currentTime;
// }

// // ISR for right sensor
// void IRAM_ATTR rightInterrupt() {
//   unsigned long currentTime = micros();
//   if (lastRightPulseTime > 0) {
//     unsigned long timeDiff = currentTime - lastRightPulseTime;
//     if (timeDiff > 0) {
//       rightRPM = (60000000.0 / timeDiff);
//     }
//   }
//   lastRightPulseTime = currentTime;
// }

// void setup() {
//   Serial.begin(115200);
//   while (!Serial);

//   Serial.println("ESP32 Instantaneous RPM via CAN");

//   if (!CAN.begin(500E3)) {
//     Serial.println("CAN Init Failed!");
//     while (1);
//   }

//   // Attach interrupts for both sensors
//   attachInterrupt(digitalPinToInterrupt(leftSensorPin), leftInterrupt, RISING);
//   attachInterrupt(digitalPinToInterrupt(rightSensorPin), rightInterrupt, RISING);
// }

// void loop() {
//   static unsigned long lastSendTime = 0;
//   unsigned long currentMillis = millis();

//   // Send data every 100ms (adjustable)
//   if (currentMillis - lastSendTime >= 100) {
//     Serial.print("LEFT RPM: "); Serial.print(leftRPM);
//     Serial.print(" | RIGHT RPM: "); Serial.println(rightRPM);

//     sendCAN(leftRPM, rightRPM);
//     lastSendTime = currentMillis;
//   }
// }

// // Function to send RPM over CAN
// void sendCAN(float leftRPM, float rightRPM) {
//   int left = (int)leftRPM;
//   int right = (int)rightRPM;

//   CAN.beginPacket(0x15);
//   CAN.write(left >> 8);
//   CAN.write(left & 0xFF);
//   CAN.write(right >> 8);
//   CAN.write(right & 0xFF);
//   CAN.endPacket();
// }

// #4th iter w/ a better time interval and more accurate reset to 0 
// #include <CAN.h>

// const int leftSensorPin = 25;   // Left photointerruptor pin
// const int rightSensorPin = 26;  // Right photointerruptor pin

// volatile unsigned int leftRevolutions = 0, rightRevolutions = 0;
// volatile unsigned long lastLeftPulseTime = 0, lastRightPulseTime = 0;

// unsigned long lastCalcTime = 0;
// float leftRPM = 0, rightRPM = 0;

// void IRAM_ATTR leftInterrupt() {
//   leftRevolutions++;  
//   lastLeftPulseTime = millis();  
// }

// void IRAM_ATTR rightInterrupt() {
//   rightRevolutions++;  
//   lastRightPulseTime = millis();  
// }

// void setup() {
//   Serial.begin(9600);
//   while (!Serial);

//   Serial.println("ESP32 RPM Calculation with Reset");

//   if (!CAN.begin(500E3)) {
//     Serial.println("CAN Init Failed!");
//     while (1);
//   }

//   attachInterrupt(digitalPinToInterrupt(leftSensorPin), leftInterrupt, RISING);
//   attachInterrupt(digitalPinToInterrupt(rightSensorPin), rightInterrupt, RISING);
// }

// void loop() {
//   unsigned long currentMillis = millis();

//   // Compute RPM every 1 second
//   if (currentMillis - lastCalcTime >= 1000) {
//     noInterrupts();  // Prevent interrupts while calculating RPM

//     float timeElapsed = (currentMillis - lastCalcTime) / 1000.0; // Time in seconds

//     if (timeElapsed > 0) {
//       leftRPM = (leftRevolutions / timeElapsed) * 60.0;
//       rightRPM = (rightRevolutions / timeElapsed) * 60.0;
//     } else {
//       leftRPM = 0;
//       rightRPM = 0;
//     }

//     leftRevolutions = 0;
//     rightRevolutions = 0;
//     lastCalcTime = currentMillis;
//     interrupts();  // Re-enable interrupts

//     // Ensure RPM doesn't reset too quickly
//     if (currentMillis - lastLeftPulseTime > 2000) leftRPM = 0;   // Increased timeout
//     if (currentMillis - lastRightPulseTime > 2000) rightRPM = 0; 

//     Serial.print("LEFT RPM: "); Serial.print(leftRPM);
//     Serial.print(" | RIGHT RPM: "); Serial.println(rightRPM);

//     sendCAN(leftRPM, rightRPM);
//   }
// }

// // Send RPM over CAN
// void sendCAN(float leftRPM, float rightRPM) {
//   int left = (int)leftRPM;
//   int right = (int)rightRPM;

//   Serial.print("Sending CAN - LEFT: "); Serial.print(left);
//   Serial.print(" RIGHT: "); Serial.println(right);

//   CAN.beginPacket(0x15);
//   CAN.write(left >> 8);
//   CAN.write(left & 0xFF);
//   CAN.write(right >> 8);
//   CAN.write(right & 0xFF);
//   CAN.endPacket();
// }


// volatile unsigned long lastLeftChangeTime = 0;
// volatile unsigned long leftPeriod = 0;
// volatile bool lastState = LOW;
// float leftRPM = 0;

// void IRAM_ATTR leftInterrupt() {
//   unsigned long currentTime = millis();
  
//   // Only process state changes
//   bool currentState = digitalRead(25);
//   if (currentState != lastState) {  
//     if (lastLeftChangeTime > 0) {  // Ignore first pulse
//       leftPeriod = currentTime - lastLeftChangeTime;
//     }
//     lastLeftChangeTime = currentTime;
//     lastState = currentState;
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   pinMode(25, INPUT_PULLUP);  // Adjust to correct pin
//   attachInterrupt(digitalPinToInterrupt(25), leftInterrupt, CHANGE);
// }

// void loop() {
//   // Calculate RPM only if a valid period has been recorded
//   if (leftPeriod > 0) {
//     leftRPM = 60000.0 / (leftPeriod * 2);  // RPM Calculation
//   }

//   // Reset RPM to 0 if no recent pulses
//   if (millis() - lastLeftChangeTime > 200) {
//     leftRPM = 0;
//   }

//   Serial.print("LEFT RPM: "); 
//   Serial.println(leftRPM);
  
//   delay(200);  // Small delay for readability
// }

#include <Arduino.h>
#include <CAN.h>

// Define sensor pins (adjust these according to your wiring)
#define LEFT_SENSOR_PIN 25
#define RIGHT_SENSOR_PIN 3

// Update interval in milliseconds (industry standard ~10Hz)
#define UPDATE_INTERVAL 1000

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



// Provided sendCAN function that sends two RPM values over CAN
void sendCAN(float leftRPM, float rightRPM) {
  int left = (int)leftRPM;
  int right = (int)rightRPM;

  Serial.print("Sending CAN - LEFT: ");
  Serial.print(left);
  Serial.print(" RIGHT: ");
  Serial.println(right);

  CAN.beginPacket(0x15);
  CAN.write(left >> 8);
  CAN.write(left & 0xFF);
  CAN.write(right >> 8);
  CAN.write(right & 0xFF);
  CAN.endPacket();
}

// Helper function to calculate RPM from pulse interval (in microseconds)
// RPM = (1 / (pulseInterval in seconds)) * 60
float calculateRPM(unsigned long pulseInterval) {
  if (pulseInterval == 0) return 0.0;
  float timePerRevolution = pulseInterval / 1000000.0; // Convert microseconds to seconds
  float rpm = (1.0 / timePerRevolution) * 60.0;
  return rpm;
}

void setup() {
  Serial.begin(115200);

  // Initialize sensor pins with pull-up resistors
  pinMode(LEFT_SENSOR_PIN, INPUT_PULLUP);
  pinMode(RIGHT_SENSOR_PIN, INPUT_PULLUP);

  // Attach interrupts to sensor pins (trigger on falling edge)
  attachInterrupt(digitalPinToInterrupt(LEFT_SENSOR_PIN), leftSensorISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SENSOR_PIN), rightSensorISR, FALLING);

  // Initialize CAN bus at 500 kbps
  // (If CAN.begin returns false, it means initialization failed)
  while (!CAN.begin(500E3)) {  // 500 kbps
    Serial.println("CAN bus initialization failed, retrying...");
    delay(1000);
  }
  Serial.println("CAN bus started");
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

    // Transmit the RPM values over CAN
    sendCAN(leftRPM, rightRPM);
  }
}


// #include <Arduino.h>
// #include <CAN.h>

// #define LEFT_SENSOR_PIN 4   // GPIO for left wheel sensor
// #define RIGHT_SENSOR_PIN 5  // GPIO for right wheel sensor
// #define INTERVAL_MS 100     // Send data every 100ms (10Hz)

// // Variables for RPM calculation
// volatile unsigned long lastLeftPulseTime = 0;
// volatile unsigned long lastRightPulseTime = 0;
// volatile unsigned long leftPulseInterval = 0;
// volatile unsigned long rightPulseInterval = 0;

// // Interrupt service routines for both sensors
// void IRAM_ATTR leftWheelISR() {
//     unsigned long now = micros();
//     if (lastLeftPulseTime > 0) {
//         leftPulseInterval = now - lastLeftPulseTime;
//     }
//     lastLeftPulseTime = now;
// }

// void IRAM_ATTR rightWheelISR() {
//     unsigned long now = micros();
//     if (lastRightPulseTime > 0) {
//         rightPulseInterval = now - lastRightPulseTime;
//     }
//     lastRightPulseTime = now;
// }

// // Function to calculate RPM from pulse interval
// float calculateRPM(unsigned long pulseInterval) {
//     if (pulseInterval == 0) return 0;  // Avoid division by zero
//     float timePerRevolution = pulseInterval / 1e6;  // Convert microseconds to seconds
//     return (1.0 / timePerRevolution) * 60.0;  // Convert to RPM
// }

// void setup() {
//     Serial.begin(9600);

//     // Initialize sensors with interrupts
//     pinMode(LEFT_SENSOR_PIN, INPUT_PULLUP);
//     pinMode(RIGHT_SENSOR_PIN, INPUT_PULLUP);
//     attachInterrupt(digitalPinToInterrupt(LEFT_SENSOR_PIN), leftWheelISR, FALLING);
//     attachInterrupt(digitalPinToInterrupt(RIGHT_SENSOR_PIN), rightWheelISR, FALLING);

//     // Initialize CAN Bus
//     if (!CAN.begin(500E3)) {  // 500 kbps standard automotive speed
//         Serial.println("CAN BUS Initialization Failed!");
//         while (1);
//     }
//     Serial.println("CAN BUS Initialized");
// }

// void loop() {
//     static unsigned long lastUpdate = 0;

//     if (millis() - lastUpdate >= INTERVAL_MS) {
//         lastUpdate = millis();

//         // Calculate RPM for both wheels
//         float leftRPM = calculateRPM(leftPulseInterval);
//         float rightRPM = calculateRPM(rightPulseInterval);

//         // Send via CAN
//         sendCAN(leftRPM, rightRPM);
//     }
// }

// // Your existing sendCAN function
// void sendCAN(float leftRPM, float rightRPM) {
//     int left = (int)leftRPM;
//     int right = (int)rightRPM;

//     Serial.print("Sending CAN - LEFT: "); Serial.print(left);
//     Serial.print(" RIGHT: "); Serial.println(right);

//     CAN.beginPacket(0x15);
//     CAN.write(left >> 8);
//     CAN.write(left & 0xFF);
//     CAN.write(right >> 8);
//     CAN.write(right & 0xFF);
//     CAN.endPacket();
// }


