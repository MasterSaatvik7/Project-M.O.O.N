//#include <Wire.h>
//#include <Adafruit_BMP085.h>
//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
//
//// Create sensor objects
//Adafruit_BMP085 bmp;
//Adafruit_MPU6050 mpu;
//
//void setup() {
//  Serial.begin(115200);
//  while (!Serial)
//    delay(10); // Wait for serial to be ready (for boards like Leonardo)
//
//  // BMP085 setup
//  Serial.println("Initializing BMP085...");
//  if (!bmp.begin()) {
//    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
//    while (1);
//  }
//  Serial.println("BMP085 Found!");
//
//  // MPU6050 setup
//  Serial.println("Initializing MPU6050...");
//  if (!mpu.begin()) {
//    Serial.println("Failed to find MPU6050 chip");
//    while (1);
//  }
//  Serial.println("MPU6050 Found!");
//
//  // Accelerometer range
//  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//  Serial.print("Accelerometer range set to: +-");
//  Serial.print(8);
//  Serial.println("G");
//
//  // Gyroscope range
//  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//  Serial.print("Gyro range set to: +-");
//  Serial.print(500);
//  Serial.println(" deg/s");
//
//  // Filter bandwidth
//  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
//  Serial.println("Filter bandwidth set to: 21 Hz");
//
//  Serial.println();
//  delay(100);
//}
//
//void loop() {
//  // -------------------- BMP085 Readings --------------------
//  float temperature = bmp.readTemperature();
//  float pressure = bmp.readPressure();
//  float altitude_std = bmp.readAltitude(); // Standard sea-level pressure (101325 Pa)
//  float seaLevelPressure = bmp.readSealevelPressure(7.0); // Chennai approx altitude
//  float realAltitude = bmp.readAltitude(seaLevelPressure); // Adjusted altitude
//
//  Serial.println("---- BMP085 Data ----");
//  Serial.print("Temperature = ");
//  Serial.print(temperature);
//  Serial.println(" *C");
//
//  Serial.print("Pressure = ");
//  Serial.print(pressure);
//  Serial.println(" Pa");
//
//  Serial.print("Altitude (standard) = ");
//  Serial.print(altitude_std);
//  Serial.println(" meters");
//
//  Serial.print("Pressure at sea level (from 7m) = ");
//  Serial.print(seaLevelPressure);
//  Serial.println(" Pa");
//
//  Serial.print("Real altitude = ");
//  Serial.print(realAltitude);
//  Serial.println(" meters");
//
//  // -------------------- MPU6050 Readings --------------------
//  sensors_event_t a, g, temp;
//  mpu.getEvent(&a, &g, &temp);
//
//  Serial.println("---- MPU6050 Data ----");
//  Serial.print("Acceleration X: ");
//  Serial.print(a.acceleration.x);
//  Serial.print(", Y: ");
//  Serial.print(a.acceleration.y);
//  Serial.print(", Z: ");
//  Serial.print(a.acceleration.z);
//  Serial.println(" m/s^2");
//
//  Serial.print("Rotation X: ");
//  Serial.print(g.gyro.x);
//  Serial.print(", Y: ");
//  Serial.print(g.gyro.y);
//  Serial.print(", Z: ");
//  Serial.print(g.gyro.z);
//  Serial.println(" rad/s");
//
//  Serial.print("MPU6050 Temperature: ");
//  Serial.print(temp.temperature);
//  Serial.println(" *C");
//
//  Serial.println("---------------------------\n");
//  delay(2000); // Delay before next reading
//}


//#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_MPU6050.h>
//#include <Adafruit_HMC5883_U.h>
//#include <Adafruit_BMP085.h>
//
//// Sensor instances
//Adafruit_MPU6050 mpu;
//Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
//Adafruit_BMP085 bmp;
//
//// Function to display HMC5883 sensor info
//void displayMagSensorDetails() {
//  sensor_t sensor;
//  mag.getSensor(&sensor);
//  Serial.println("------------------------------------");
//  Serial.print("Sensor:       "); Serial.println(sensor.name);
//  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
//  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
//  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
//  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
//  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
//  Serial.println("------------------------------------\n");
//  delay(500);
//}
//
//void setup() {
//  Serial.begin(115200);
//  while (!Serial) delay(10);
//
//  Serial.println("Initializing MPU6050, HMC5883, and BMP085...\n");
//
//  // MPU6050 setup
//  if (!mpu.begin()) {
//    Serial.println("Failed to find MPU6050 chip");
//    while (1);
//  }
//  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
//  Serial.println("MPU6050 initialized.");
//
//  // HMC5883 setup
//  if (!mag.begin()) {
//    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
//    while (1);
//  }
//  displayMagSensorDetails();
//  Serial.println("HMC5883 initialized.");
//
//  // BMP085 setup
//  if (!bmp.begin()) {
//    Serial.println("Could not find BMP085 sensor at address 0x77");
//    while (1);
//  }
//  Serial.println("BMP085 initialized.\n");
//}
//
//void loop() {
//  // MPU6050 readings
//  sensors_event_t accel, gyro, temp;
//  mpu.getEvent(&accel, &gyro, &temp);
//
//  Serial.println("== MPU6050 Readings ==");
//  Serial.print("Acceleration (m/s^2): X=");
//  Serial.print(accel.acceleration.x); Serial.print(" Y=");
//  Serial.print(accel.acceleration.y); Serial.print(" Z=");
//  Serial.println(accel.acceleration.z);
//
//  Serial.print("Gyroscope (rad/s): X=");
//  Serial.print(gyro.gyro.x); Serial.print(" Y=");
//  Serial.print(gyro.gyro.y); Serial.print(" Z=");
//  Serial.println(gyro.gyro.z);
//
//  Serial.print("Temperature (°C) from MPU6050: ");
//  Serial.println(temp.temperature);
//
//  // HMC5883 readings
//  sensors_event_t magEvent;
//  mag.getEvent(&magEvent);
//
//  Serial.println("== HMC5883 Readings ==");
//  Serial.print("Magnetic Field (uT): X=");
//  Serial.print(magEvent.magnetic.x); Serial.print(" Y=");
//  Serial.print(magEvent.magnetic.y); Serial.print(" Z=");
//  Serial.println(magEvent.magnetic.z);
//
//  float heading = atan2(magEvent.magnetic.y, magEvent.magnetic.x);
//  float declinationAngle = 0.22; // Adjust based on your location
//  heading += declinationAngle;
//  if (heading < 0) heading += 2 * PI;
//  if (heading > 2 * PI) heading -= 2 * PI;
//  float headingDegrees = heading * 180 / M_PI;
//
//  Serial.print("Heading (degrees): ");
//  Serial.println(headingDegrees);
//
//  // BMP085 readings
//  Serial.println("== BMP085 Readings ==");
//  Serial.print("Temperature (°C): ");
//  Serial.println(bmp.readTemperature());
//
//  Serial.print("Pressure (Pa): ");
//  Serial.println(bmp.readPressure());
//
//  Serial.print("Altitude (m): ");
//  Serial.println(bmp.readAltitude());
//
//  Serial.print("Pressure at sealevel (Pa): ");
//  Serial.println(bmp.readSealevelPressure());
//
//  Serial.println("\n-----------------------------\n");
//
//  delay(1000);
//}

/***************************************************************************
  Combined code for HMC5883L magnetometer, BMP085 barometric sensor,
  and MPU6050 accelerometer/gyroscope.
  
  Libraries required:
  - Adafruit_Sensor
  - Adafruit_HMC5883_U
  - Adafruit_BMP085
  - Adafruit_MPU6050
  - Wire
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>

// Initialize sensors
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;

// Configuration settings
const float DECLINATION_ANGLE = -1.12; // Set this to your local declination angle
const int DELAY_MS = 500;             // Delay between readings

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // For Leonardo/Micro
  
  Serial.println("Multi-sensor Test");
  
  // Initialize HMC5883L magnetometer
  if (!mag.begin()) {
    Serial.println("No HMC5883 detected. Check wiring!");
    while(1);
  }
  
  // Initialize BMP085 barometer
  if (!bmp.begin()) {
    Serial.println("No BMP085 detected. Check wiring!");
    while(1);
  }
  
  // Initialize MPU6050 accelerometer/gyroscope
  if (!mpu.begin()) {
    Serial.println("No MPU6050 detected. Check wiring!");
    while(1);
  }
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("All sensors initialized successfully!");
  delay(100);
}

void loop() {
  // Read and display magnetometer data
  readMagnetometer();
  
  // Read and display barometer data
  readBarometer();
  
  // Read and display accelerometer/gyroscope data
  readAccelGyro();
  
  Serial.println("\n-------------------------------------\n");
  delay(DELAY_MS);
}

void readMagnetometer() {
  Serial.println("=== MAGNETOMETER (HMC5883L) ===");
  
  // Get magnetometer reading
  sensors_event_t event;
  mag.getEvent(&event);
  
  // Display magnetic vector values (micro-Tesla)
  Serial.print("Magnetic field - X: ");
  Serial.print(event.magnetic.x);
  Serial.print(" Y: ");
  Serial.print(event.magnetic.y);
  Serial.print(" Z: ");
  Serial.print(event.magnetic.z);
  Serial.println(" uT");
  
  // Calculate heading
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Add declination angle
  heading += DECLINATION_ANGLE;
  
  // Correct for sign reversals
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
    
  // Convert to degrees
  float headingDegrees = heading * 180 / M_PI;
  Serial.print("Heading: ");
  Serial.print(headingDegrees);
  Serial.println(" degrees");
}

void readBarometer() {
  Serial.println("=== BAROMETER/TEMPERATURE (BMP085) ===");
  
  // Read temperature
  float temperature = bmp.readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  
  // Read pressure
  int32_t pressure = bmp.readPressure();
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");
  
  // Calculate altitude (standard barometric pressure)
  float altitude = bmp.readAltitude();
  Serial.print("Default Altitude: ");
  Serial.print(altitude);
  Serial.println(" meters");
  
  // Chennai calibration (7m above sea level)
  float altitude_m = 7.0;
  float calibratedSeaLevelPressure = pressure / pow(1.0 - (altitude_m / 44330.0), 5.255);
  Serial.print("Calibrated Sea Level Pressure = ");
  Serial.print(calibratedSeaLevelPressure);
  Serial.println(" Pa");
  Serial.print("Calibrated Altitude (should be close to 7m) = ");
  Serial.print(bmp.readAltitude(calibratedSeaLevelPressure));
  Serial.println(" meters");
  
  // Standard sea level pressure calculation
  int32_t standardSeaLevelPressure = bmp.readSealevelPressure();
  Serial.print("Standard Sea level pressure: ");
  Serial.print(standardSeaLevelPressure);
  Serial.println(" Pa");
}

void readAccelGyro() {
  Serial.println("=== ACCELEROMETER/GYROSCOPE (MPU6050) ===");
  
  // Get new readings
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  // Display acceleration values
  Serial.print("Acceleration - X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" Y: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" Z: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s²");
  
  // Display gyroscope values
  Serial.print("Rotation - X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" Y: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" Z: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" rad/s");
  
  // Display temperature from MPU6050
  Serial.print("MPU Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");
}
