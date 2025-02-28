#include <Arduino.h>
#include <CAN.h>


const int analogPin = 33;  // ADC-capable pin
int analogValue = 0;
char analogStr[5];     // Buffer to hold 4-digit string + null terminator
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 50;  //change as per requirements
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("CAN Sender");
  // Start the CAN bus at 500 kbps (500E3) OR 1 Mbps (1000E3)
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}
void loop() {
  unsigned long currentMillis = millis();
  analogValue = analogRead(analogPin);  // Read analog value (0-4095)
   // Only send data if the value has changed significantly or 50ms has passed
  // if (abs(analogValue - prevAnalogValue) > 15 || currentMillis - lastSendTime >= sendInterval) {
  //   prevAnalogValue = analogValue;
  //   lastSendTime = currentMillis;
  sprintf(analogStr, "%04d", analogValue);  // Convert to 4-char string (e.g., "0345")
  Serial.print("Sending Analog Value: ");
  Serial.println(analogStr);  // Print value before sending
  CAN.beginPacket(0x12);  // Send packet with ID 0x12
  for (int i = 0; i < 4; i++) {
    CAN.write(analogStr[i]);  // Send each digit as a byte
  }
  CAN.endPacket();
  delay(500);  // Delay based on A&A requirements.
}
