// Include the external utilities file.
#include "utils.ino"

void setup() {
  // Initialize serial communication at 9600 baud:
  Serial.begin(9600);
}

void loop() {
  // Call the custom utility function from utils.ino
  int sensorValue = readSensor();

  // Print the sensor value to the serial monitor:
  Serial.println(sensorValue);

  // Pause for a moment
  delay(1000);
}

