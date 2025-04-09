#include <Wire.h>
#include <Adafruit_BMP085.h>

// Create an instance of the sensor
Adafruit_BMP085 bmp;

void setup() {
  Serial.begin(9600);
  
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {} // Stop the program if sensor isn't found
  }
}

void loop() {
  // Read sensor values
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  float altitude_std = bmp.readAltitude(); // Using standard sea-level pressure
  float seaLevelPressure = bmp.readSealevelPressure(7.0); // Chennai altitude ~7 meters
  float realAltitude = bmp.readAltitude(seaLevelPressure); // More accurate altitude using calculated sea-level pressure

  // Print all values
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" Pa");

  Serial.print("Altitude (standard) = ");
  Serial.print(altitude_std);
  Serial.println(" meters");

  Serial.print("Pressure at sea level (calculated from 7m) = ");
  Serial.print(seaLevelPressure);
  Serial.println(" Pa");

  Serial.print("Real altitude (using calibrated sea-level pressure) = ");
  Serial.print(realAltitude);
  Serial.println(" meters");

  Serial.println();
  delay(2000); // Wait 2 seconds between readings
}
