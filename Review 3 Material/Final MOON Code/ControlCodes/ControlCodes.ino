#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <DHT11.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>

// Pin Definitions
#define BT_RX 1   // Connect to TX of Bluetooth module
#define BT_TX 0   // Connect to RX of Bluetooth module
#define GPS_RX 4
#define GPS_TX 3
#define DHT_PIN 6

// Motor Pins from ControlCodes.ino
// Left Wheel
#define in1 A0
#define in2 A1
// Right Wheel
#define in3 A2
#define in4 A3

// Bluetooth and GPS Serial
SoftwareSerial bluetooth(BT_RX, BT_TX);
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// Initialize sensors
TinyGPS gps;
DHT11 dht11(DHT_PIN);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;

// Configuration
const float DECLINATION_ANGLE = -1.12; // Local declination angle
const unsigned long TRANSMIT_INTERVAL = 1000; // 1 second between transmissions
unsigned long lastTransmitTime = 0;

// Sensor data structure - Using smaller data types where possible
struct SensorData {
  // DHT11
  int8_t temperature;
  uint8_t humidity;
  
  // GPS
  float latitude;
  float longitude;
  uint8_t satellites;
  uint16_t hdop;
  
  // MPU6050
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  
  // HMC5883L
  float heading;
  
  // BMP085
  int32_t pressure;
  float altitude;
};

SensorData sensorData;

// Program strings stored in flash memory to save RAM
const char PSTR_INIT[] PROGMEM = "Initializing Integrated Sensor System...";
const char PSTR_SETUP[] PROGMEM = "Setup complete!";
const char PSTR_MPU_INIT[] PROGMEM = "Initializing MPU6050... ";
const char PSTR_HMC_INIT[] PROGMEM = "Initializing HMC5883L... ";
const char PSTR_BMP_INIT[] PROGMEM = "Initializing BMP085... ";
const char PSTR_SUCCESS[] PROGMEM = "Success!";
const char PSTR_FAILED[] PROGMEM = "Failed!";
const char PSTR_DHT_READY[] PROGMEM = "DHT11 ready on pin ";
const char PSTR_NO_SIGNAL[] PROGMEM = "No signal";
const char PSTR_RCV_CMD[] PROGMEM = "Received command: ";
const char PSTR_ERR_CMD[] PROGMEM = "Error: Unknown command!";
const char PSTR_VALID_CMD[] PROGMEM = "Valid commands are:";

// Helper function to print flash strings to save RAM
void printProgString(const char* str, Stream &output) {
  char c;
  for (uint8_t i = 0; (c = pgm_read_byte(str + i)) != 0; i++) {
    output.print(c);
  }
}

void setup() {
  // Initialize motor pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Initialize serial communications
  Serial.begin(9600);
  bluetooth.begin(9600);
  gpsSerial.begin(4800);
  
  printProgString(PSTR_INIT, Serial);
  Serial.println();
  printProgString(PSTR_INIT, bluetooth);
  bluetooth.println();
  
  // Initialize I2C sensors
  Wire.begin();
  initializeSensors();
  
  // Explicitly stop motors at initialization
  stopMotors();
  
  Serial.println("DC Motor Control with Integrated Sensors");
  bluetooth.println("DC Motor Control with Integrated Sensors");
  
  printProgString(PSTR_SETUP, Serial);
  Serial.println();
  
  printProgString(PSTR_SETUP, bluetooth);
  bluetooth.println();
  
  delay(1000);
}

void loop() {
  // Check for commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.println("Received via Serial: " + command);
    
    handleCommand(command);
  }
  
  if (bluetooth.available() > 0) {
    String command = bluetooth.readStringUntil('\n');
    command.trim();
    Serial.println("Received via BT: " + command);
    
    handleCommand(command);
  }
  
  // Check if it's time to read sensors and transmit data
  unsigned long currentTime = millis();
  if (currentTime - lastTransmitTime >= TRANSMIT_INTERVAL) {
    // Read all sensor data
    readDHT11();
    readGPS();
    readMPU6050();
    readHMC5883L();
    readBMP085();
    
    // Transmit data in specified format
    transmitData();
    
    lastTransmitTime = currentTime;
  }
}

// Initialize all sensors
void initializeSensors() {
  // Initialize MPU6050
  printProgString(PSTR_MPU_INIT, Serial);
  if (!mpu.begin()) {
    printProgString(PSTR_FAILED, Serial);
    Serial.println();
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    printProgString(PSTR_SUCCESS, Serial);
    Serial.println();
  }
  
  // Initialize HMC5883L
  printProgString(PSTR_HMC_INIT, Serial);
  if (!mag.begin()) {
    printProgString(PSTR_FAILED, Serial);
    Serial.println();
  } else {
    printProgString(PSTR_SUCCESS, Serial);
    Serial.println();
  }
  
  // Initialize BMP085
  printProgString(PSTR_BMP_INIT, Serial);
  if (!bmp.begin()) {
    printProgString(PSTR_FAILED, Serial);
    Serial.println();
  } else {
    printProgString(PSTR_SUCCESS, Serial);
    Serial.println();
  }
  
  // DHT11 doesn't need explicit initialization
  printProgString(PSTR_DHT_READY, Serial);
  Serial.println(DHT_PIN);
  
  // Initialize default values for GPS
  sensorData.latitude = NAN;
  sensorData.longitude = NAN;
  sensorData.satellites = 0;
  sensorData.hdop = 9999;
}

// Read DHT11 temperature and humidity
void readDHT11() {
  int temperature = 0;
  int humidity = 0;
  
  int result = dht11.readTemperatureHumidity(temperature, humidity);
  
  if (result == 0) {
    // Reading successful
    sensorData.temperature = temperature;
    sensorData.humidity = humidity;
  }
}

// Read GPS data - Optimized to use less memory
void readGPS() {
  bool newData = false;
  unsigned long startTime = millis();
  
  // Try to get GPS data for up to 300ms (reduced from 500ms)
  while (millis() - startTime < 300) {
    if (gpsSerial.available()) {
      char c = gpsSerial.read();
      if (gps.encode(c)) {
        newData = true;
      }
    }
  }
  
  // Try to get position if new data is available
  if (newData) {
    float latitude, longitude;
    unsigned long age;
    
    gps.f_get_position(&latitude, &longitude, &age);
      
    // Check if the position is valid
    if (latitude != TinyGPS::GPS_INVALID_F_ANGLE && 
        longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
      sensorData.latitude = latitude;
      sensorData.longitude = longitude;
    } else {
      // Use default coordinates if invalid
      sensorData.latitude = NAN;
      sensorData.longitude = NAN;
    }
      
    // Get satellites and precision data
    sensorData.satellites = gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites();
    sensorData.hdop = gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 9999 : gps.hdop();
  }
}

// Read MPU6050 accelerometer and gyroscope
void readMPU6050() {
  sensors_event_t accel, gyro, temp;
  
  mpu.getEvent(&accel, &gyro, &temp);
  
  // Store acceleration values
  sensorData.accelX = accel.acceleration.x;
  sensorData.accelY = accel.acceleration.y;
  sensorData.accelZ = accel.acceleration.z;
  
  // Store gyroscope values
  sensorData.gyroX = gyro.gyro.x;
  sensorData.gyroY = gyro.gyro.y;
  sensorData.gyroZ = gyro.gyro.z;
}

// Read HMC5883L magnetometer
void readHMC5883L() {
  sensors_event_t event;
  
  mag.getEvent(&event);
  
  // Calculate heading directly without storing individual components
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  heading += DECLINATION_ANGLE;
  
  // Normalize heading
  if (heading < 0) heading += 2 * PI;
  if (heading > 2 * PI) heading -= 2 * PI;
  
  sensorData.heading = heading * 180 / M_PI; // Convert to degrees
}

// Read BMP085 barometer
void readBMP085() {
  sensorData.pressure = bmp.readPressure();
  sensorData.altitude = bmp.readAltitude();
}

// Motor control functions from ControlCodes.ino
void moveBackward(int duration) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  Serial.println("Moving forward for " + String(duration/1000) + " seconds");
  bluetooth.println("Moving forward for " + String(duration/1000) + " seconds");
  
  delay(duration);
  stopMotors();
}

void moveForward(int duration) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  Serial.println("Moving backward for " + String(duration/1000) + " seconds");
  bluetooth.println("Moving backward for " + String(duration/1000) + " seconds");
  
  delay(duration);
  stopMotors();
}

void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  // Left wheel stops
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);  // Right wheel forward
  
  Serial.println("Turning left");
  bluetooth.println("Turning left");
  
  delay(1000);
  stopMotors();
}

void turnRight() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);  // Left wheel forward
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);  // Right wheel stops
  
  Serial.println("Turning right");
  bluetooth.println("Turning right");
  
  delay(1000);
  stopMotors();
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  Serial.println("Motors stopped");
  bluetooth.println("Motors stopped");
}

void runSquare() {
  Serial.println("Running square pattern");
  bluetooth.println("Running square pattern");
  
  for (int i = 0; i < 4; i++) {
    moveForward(2000);
    turnLeft();
  }
}

void handleCommand(String command) {
  command.toLowerCase();
  int duration = 3000; // Default duration
  int spaceIndex = command.indexOf(' ');
  String mainCmd = command;
  
  // If there's a space, separate command and value
  if (spaceIndex > 0) {
    mainCmd = command.substring(0, spaceIndex);
    String timeStr = command.substring(spaceIndex + 1);
    int seconds = timeStr.toInt();
    if (seconds > 0) {
      duration = seconds * 1000;
    }
  }
  
  if (mainCmd == "forward") {
    moveForward(duration);
  } else if (mainCmd == "back" || mainCmd == "backward") {
    moveBackward(duration);
  } else if (command == "left") {
    turnLeft();
  } else if (command == "right") {
    turnRight();
  } else if (command == "stop") {
    stopMotors();
  } else if (command == "square") {
    runSquare();
  } else {
    Serial.println("Error: Unknown command!");
    Serial.println("Valid commands are:");
    Serial.println("- 'forward [seconds]': Move forward");
    Serial.println("- 'back [seconds]' or 'backward [seconds]': Move backward");
    Serial.println("- 'left': Turn left");
    Serial.println("- 'right': Turn right");
    Serial.println("- 'stop': Stop motors");
    Serial.println("- 'square': Run in square pattern");
    
    bluetooth.println("Error: Unknown command!");
    bluetooth.println("Valid commands are:");
    bluetooth.println("- 'forward [seconds]': Move forward");
    bluetooth.println("- 'back [seconds]' or 'backward [seconds]': Move backward");
    bluetooth.println("- 'left': Turn left");
    bluetooth.println("- 'right': Turn right");
    bluetooth.println("- 'stop': Stop motors");
    bluetooth.println("- 'square': Run in square pattern");
  }
}

// Transmit all sensor data over Bluetooth and Serial in the specified format
void transmitData() {
  // Format: "temperature,humidity,LAT,LONG,No of SAT,HDOP,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Heading,Pressure,Altitude"
  
  // DHT11
  Serial.print(sensorData.temperature);
  Serial.print(F(","));
  Serial.print(sensorData.humidity);
  Serial.print(F(","));
  
  bluetooth.print(sensorData.temperature);
  bluetooth.print(F(","));
  bluetooth.print(sensorData.humidity);
  bluetooth.print(F(","));

  // GPS
  if (isnan(sensorData.latitude) || isnan(sensorData.longitude)) {
    printProgString(PSTR_NO_SIGNAL, Serial);
    Serial.print(F(","));
    printProgString(PSTR_NO_SIGNAL, Serial);
    Serial.print(F(","));
    
    printProgString(PSTR_NO_SIGNAL, bluetooth);
    bluetooth.print(F(","));
    printProgString(PSTR_NO_SIGNAL, bluetooth);
    bluetooth.print(F(","));
  } else {
    Serial.print(sensorData.latitude, 6);
    Serial.print(F(","));
    Serial.print(sensorData.longitude, 6);
    Serial.print(F(","));
    
    bluetooth.print(sensorData.latitude, 6);
    bluetooth.print(F(","));
    bluetooth.print(sensorData.longitude, 6);
    bluetooth.print(F(","));
  }

  Serial.print(sensorData.satellites);
  Serial.print(F(","));
  Serial.print(sensorData.hdop);
  Serial.print(F(","));
  
  bluetooth.print(sensorData.satellites);
  bluetooth.print(F(","));
  bluetooth.print(sensorData.hdop);
  bluetooth.print(F(","));

  // MPU6050
  Serial.print(sensorData.accelX);
  Serial.print(F(","));
  Serial.print(sensorData.accelY);
  Serial.print(F(","));
  Serial.print(sensorData.accelZ);
  Serial.print(F(","));
  Serial.print(sensorData.gyroX);
  Serial.print(F(","));
  Serial.print(sensorData.gyroY);
  Serial.print(F(","));
  Serial.print(sensorData.gyroZ);
  Serial.print(F(","));

  bluetooth.print(sensorData.accelX);
  bluetooth.print(F(","));
  bluetooth.print(sensorData.accelY);
  bluetooth.print(F(","));
  bluetooth.print(sensorData.accelZ);
  bluetooth.print(F(","));
  bluetooth.print(sensorData.gyroX);
  bluetooth.print(F(","));
  bluetooth.print(sensorData.gyroY);
  bluetooth.print(F(","));
  bluetooth.print(sensorData.gyroZ);
  bluetooth.print(F(","));

  // HMC5883L - Only transmit heading
  Serial.print(sensorData.heading);
  Serial.print(F(","));
  
  bluetooth.print(sensorData.heading);
  bluetooth.print(F(","));

  // BMP085
  Serial.print(sensorData.pressure);
  Serial.print(F(","));
  Serial.println(sensorData.altitude);

  bluetooth.print(sensorData.pressure);
  bluetooth.print(F(","));
  bluetooth.println(sensorData.altitude);
}
