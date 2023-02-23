#include <Wire.h>

#define SLAVE_ADDRESS 0x04 // Set the I2C address of the slave board

void setup() {
  Wire.begin(); // Initialize I2C bus
  Serial.begin(9600); // Initialize serial communication
  while (!Serial); // Wait for serial port to connect
}

void loop() {
  Wire.beginTransmission(SLAVE_ADDRESS); // Begin I2C transmission to slave board
  Wire.write(0x00); // Send register address to slave board
  Wire.write(0x55); // Send data byte to slave board
  Wire.endTransmission(); // End I2C transmission
  delay(1000); // Wait 1 second before sending another message
}
