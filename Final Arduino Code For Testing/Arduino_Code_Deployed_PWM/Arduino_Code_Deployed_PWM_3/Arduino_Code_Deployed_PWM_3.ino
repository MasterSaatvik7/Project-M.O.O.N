#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <DHT11.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Servo.h>

// Pin Definitions
#define BT_RX 1
#define BT_TX 0
#define GPS_RX 4
#define GPS_TX 3
#define DHT_PIN 6

// Motor Pins
#define MOTOR1_PIN 5  // Left motor
#define MOTOR2_PIN 9  // Right motor

// Motor Control Constants
#define TURN_DELAY 1500    // 1.5 seconds 
#define NORMAL_SPEED 1500  // 50% speed for normal movement

// Default GPS coordinates (Chennai)
//#define DEFAULT_LAT 12.8437
//#define DEFAULT_LON 80.1545

// Bluetooth and GPS Serial
SoftwareSerial bluetooth(BT_RX, BT_TX);
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// Initialize sensors
TinyGPS gps;
DHT11 dht11(DHT_PIN);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;

// Initialize motors
Servo motor1;
Servo motor2;

// Configuration
const float DECLINATION_ANGLE = -1.12; // Set to your local declination angle
const unsigned long TRANSMIT_INTERVAL = 1000; // 1 second between transmissions
unsigned long lastTransmitTime = 0;

// Sensor data structure
struct SensorData {
  // DHT11
  int temperature;
  int humidity;
  
  // GPS
  float latitude;
  float longitude;
  unsigned long gpsAge;
  int satellites;
  int hdop;
  
  // MPU6050
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  
  // HMC5883L
  float magX, magY, magZ;
  float heading;
  
  // BMP085
  float pressure;
  float altitude;
};

SensorData sensorData;

void setup() {
  // Initialize serial communications
  Serial.begin(115200);
  bluetooth.begin(9600);
  gpsSerial.begin(4800);
  
  Serial.println("Initializing Integrated Sensor System...");
  bluetooth.println("Initializing Integrated Sensor System...");
  
  // Initialize I2C sensors
  Wire.begin();
  initializeSensors();
  
  // Initialize motors
  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);
  
  // Initialize motors to stopped position
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  
  Serial.println("Setup complete!");
  bluetooth.println("Setup complete!");
  delay(1000);
}

void loop() {
  // Check for Bluetooth commands
  checkBluetoothCommands();
  
  // Check if it's time to read sensors and transmit data
  unsigned long currentTime = millis();
  if (currentTime - lastTransmitTime >= TRANSMIT_INTERVAL) {
    // Read all sensor data
    readDHT11();
    readGPS();
    readMPU6050();
    readHMC5883L();
    readBMP085();
    
    // Transmit data
    transmitData(currentTime);
    
    lastTransmitTime = currentTime;
  }
}

// Initialize all sensors
void initializeSensors() {
  // Initialize MPU6050
  Serial.print("Initializing MPU6050... ");
  if (!mpu.begin()) {
    Serial.println("Failed!");
    Serial.println("Could not find MPU6050 sensor. Check wiring!");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("Success!");
  }
  
  // Initialize HMC5883L
  Serial.print("Initializing HMC5883L... ");
  if (!mag.begin()) {
    Serial.println("Failed!");
    Serial.println("Could not find HMC5883L sensor. Check wiring!");
  } else {
    Serial.println("Success!");
  }
  
  // Initialize BMP085
  Serial.print("Initializing BMP085... ");
  if (!bmp.begin()) {
    Serial.println("Failed!");
    Serial.println("Could not find BMP085 sensor. Check wiring!");
  } else {
    Serial.println("Success!");
  }
  
  // DHT11 doesn't need explicit initialization
  Serial.println("DHT11 ready on pin " + String(DHT_PIN));
  
  // Initialize default values for GPS
  sensorData.latitude = NAN;
  sensorData.longitude = NAN;
  sensorData.satellites = 0;
  sensorData.hdop = 9999;
  sensorData.gpsAge = 0;
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
  } else {
    // Error reading, keep previous values
    Serial.println("DHT11 Error: " + String(DHT11::getErrorString(result)));
  }
}

// Read GPS data
void readGPS() {
  bool newData = false;
  unsigned long startTime = millis();
  
  // Try to get GPS data for up to 500ms
  while (millis() - startTime < 500) {
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
        sensorData.gpsAge = age;
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
  
  // Temperature measurement removed as requested
}

// Read HMC5883L magnetometer
void readHMC5883L() {
  sensors_event_t event;
  
  mag.getEvent(&event);
  
  // Store magnetometer values
  sensorData.magX = event.magnetic.x;
  sensorData.magY = event.magnetic.y;
  sensorData.magZ = event.magnetic.z;
  
  // Calculate heading
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  heading += DECLINATION_ANGLE;
  
  // Normalize heading
  if (heading < 0) heading += 2 * PI;
  if (heading > 2 * PI) heading -= 2 * PI;
  
  sensorData.heading = heading * 180 / M_PI; // Convert to degrees
}

// Read BMP085 barometer
void readBMP085() {
  // Temperature measurement removed as requested
  sensorData.pressure = bmp.readPressure();
  sensorData.altitude = bmp.readAltitude();
}

// Check for Bluetooth commands
void checkBluetoothCommands() {
  if (bluetooth.available() > 0) {
    String command = bluetooth.readStringUntil('\n');
    command.trim();
    
    // Log received command
    Serial.println("Received command: " + command);
    
    // Process command for motor control
    processCommand(command);
  }
  
  // Also check Serial for commands (for debugging)
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    Serial.println("Received: " + command);
    bluetooth.println("Received: " + command);
    
    processCommand(command);
  }
}

// Process commands for motor control
void processCommand(String command) {
    int leftSpeed = 1000, rightSpeed = 1000; // Default: motors off
    bool validCommand = false;  // Flag to track if command was valid

    // Check text commands first, before numeric checks
    if (command.equalsIgnoreCase("stop")) {
        leftSpeed = 1000;  // 0% speed
        rightSpeed = 1000; // 0% speed
        
        Serial.println("Stopping Motors");
        bluetooth.println("Stopping Motors");
        
        Serial.println("Both motors stopped");
        bluetooth.println("Both motors stopped");
        validCommand = true;
    }
    else if (command.equalsIgnoreCase("left")) {
        // Step 1: Stop both motors
        motor1.writeMicroseconds(1000);
        motor2.writeMicroseconds(1000);
        Serial.println("Stopping before turn");
        bluetooth.println("Stopping before turn");
        delay(TURN_DELAY);
        
        // Step 2: Execute turn
        motor1.writeMicroseconds(1000);  // 0% speed
        motor2.writeMicroseconds(1700);  // 70% speed
        Serial.println("Turning Left");
        bluetooth.println("Turning Left");
        Serial.println("Left motor speed: 30%");
        bluetooth.println("Left motor speed: 30%");
        Serial.println("Right motor speed: 70%");
        bluetooth.println("Right motor speed: 70%");
        delay(TURN_DELAY);
        
        // Step 3: Resume normal speed
        leftSpeed = NORMAL_SPEED;
        rightSpeed = NORMAL_SPEED;
        Serial.println("Resuming normal speed");
        bluetooth.println("Resuming normal speed");
        Serial.println("Both motors speed: 50%");
        bluetooth.println("Both motors speed: 50%");
        validCommand = true;
    } 
    else if (command.equalsIgnoreCase("right")) {
        // Step 1: Stop both motors
        motor1.writeMicroseconds(1000);
        motor2.writeMicroseconds(1000);
        Serial.println("Stopping before turn");
        bluetooth.println("Stopping before turn");
        delay(TURN_DELAY);
        
        // Step 2: Execute turn
        motor1.writeMicroseconds(1700);  // 70% speed
        motor2.writeMicroseconds(1000);  // 30% speed
        Serial.println("Turning Right");
        bluetooth.println("Turning Right");
        Serial.println("Left motor speed: 70%");
        bluetooth.println("Left motor speed: 70%");
        Serial.println("Right motor speed: 30%");
        bluetooth.println("Right motor speed: 30%");
        delay(TURN_DELAY);
        
        // Step 3: Resume normal speed
        leftSpeed = NORMAL_SPEED;
        rightSpeed = NORMAL_SPEED;
        Serial.println("Resuming normal speed");
        bluetooth.println("Resuming normal speed");
        Serial.println("Both motors speed: 50%");
        bluetooth.println("Both motors speed: 50%");
        validCommand = true;
    }
    else if (command.startsWith("L") && command.indexOf("R") > 0) {
        int lIndex = command.indexOf("L");
        int rIndex = command.indexOf("R");
        int leftValue = command.substring(lIndex + 1, rIndex).toInt();
        int rightValue = command.substring(rIndex + 1).toInt();

        if (leftValue >= 0 && leftValue <= 100 && rightValue >= 0 && rightValue <= 100) {
            leftSpeed = 1000 + (leftValue * 10);
            rightSpeed = 1000 + (rightValue * 10);
            Serial.println("Left motor speed: " + String(leftValue) + "%");
            bluetooth.println("Left motor speed: " + String(leftValue) + "%");
            Serial.println("Right motor speed: " + String(rightValue) + "%");
            bluetooth.println("Right motor speed: " + String(rightValue) + "%");
            validCommand = true;
        }
    } 
    else if (command.startsWith("L")) {
        int leftValue = command.substring(1).toInt();
        if (leftValue >= 0 && leftValue <= 100) {
            leftSpeed = 1000 + (leftValue * 10);
            Serial.println("Left motor speed: " + String(leftValue) + "%");
            bluetooth.println("Left motor speed: " + String(leftValue) + "%");
            validCommand = true;
        }
    } 
    else if (command.startsWith("R")) {
        int rightValue = command.substring(1).toInt();
        if (rightValue >= 0 && rightValue <= 100) {
            rightSpeed = 1000 + (rightValue * 10);
            Serial.println("Right motor speed: " + String(rightValue) + "%");
            bluetooth.println("Right motor speed: " + String(rightValue) + "%");
            validCommand = true;
        }
    }
    else if (isNumber(command) && command.toInt() >= 0 && command.toInt() <= 100) {
        // Single-Speed Command: Both motors same speed
        int speedValue = command.toInt();
        leftSpeed = rightSpeed = 1000 + (speedValue * 10);
        Serial.println("Both motors speed: " + String(speedValue) + "%");
        bluetooth.println("Both motors speed: " + String(speedValue) + "%");
        validCommand = true;
    }

    if (!validCommand) {
        // This is original sensor data or an invalid command
        // Return without applying anything to motors
        if (!command.startsWith("Received:")) {  // Avoid showing error for confirmation messages
            Serial.println("Error: Invalid Command!");
            Serial.println("Valid commands are:");
            Serial.println("- 'stop': Stops both motors");
            Serial.println("- 'left': Turn left");
            Serial.println("- 'right': Turn right");
            Serial.println("- 'L[0-100]': Set left motor speed (0-100%)");
            Serial.println("- 'R[0-100]': Set right motor speed (0-100%)");
            Serial.println("- 'L[0-100]R[0-100]': Set both motor speeds");
            Serial.println("- '[0-100]': Set both motors to same speed");
            
            bluetooth.println("Error: Invalid Command!");
            bluetooth.println("Valid commands are:");
            bluetooth.println("- 'stop': Stops both motors");
            bluetooth.println("- 'left': Turn left");
            bluetooth.println("- 'right': Turn right");
            bluetooth.println("- 'L[0-100]': Set left motor speed (0-100%)");
            bluetooth.println("- 'R[0-100]': Set right motor speed (0-100%)");
            bluetooth.println("- 'L[0-100]R[0-100]': Set both motor speeds");
            bluetooth.println("- '[0-100]': Set both motors to same speed");
        }
        return;
    }

    // Apply speeds to motors
    motor1.writeMicroseconds(leftSpeed);
    motor2.writeMicroseconds(rightSpeed);
}

// Helper function to check if a string is a valid number
bool isNumber(String str) {
    for(unsigned int i = 0; i < str.length(); i++) {
        if(i == 0 && str.charAt(i) == '-') continue; // Handle negative numbers
        if(!isDigit(str.charAt(i))) return false;
    }
    return true;
}

// Transmit all sensor data over Bluetooth
//void transmitData(unsigned long timestamp) {
//  // Format updated to remove temperature readings from MPU6050 and BMP085
//  String dataString = String(timestamp) + ",";
//  
//  // DHT11 data
//  dataString += String(sensorData.temperature) + "," + String(sensorData.humidity) + ",";
//  
//  // GPS data
//  if (isnan(sensorData.latitude) || isnan(sensorData.longitude)) {
//    dataString += "No signal,No signal,";
//  } else {
//    dataString += String(sensorData.latitude, 6) + "," + String(sensorData.longitude, 6) + ",";
//  }
//  dataString += String(sensorData.satellites) + "," + String(sensorData.hdop) + ",";
//  
//  // MPU6050 data (temp removed)
//  dataString += String(sensorData.accelX) + "," + String(sensorData.accelY) + "," + String(sensorData.accelZ) + ",";
//  dataString += String(sensorData.gyroX) + "," + String(sensorData.gyroY) + "," + String(sensorData.gyroZ) + ",";
//  
//  // HMC5883L data
//  dataString += String(sensorData.magX) + "," + String(sensorData.magY) + "," + String(sensorData.magZ) + ",";
//  dataString += String(sensorData.heading) + ",";
//  
//  // BMP085 data (temp removed)
//  dataString += String(sensorData.pressure) + "," + String(sensorData.altitude);
//  
//  // Send data over Bluetooth
//  Serial.println(dataString);
//  bluetooth.println(dataString);
//  
//  // Also print to Serial for debugging
//  Serial.println("Sent: " + dataString);
//}

void transmitData(unsigned long timestamp) {
  // Timestamp
  Serial.print(timestamp);
  Serial.print(F(","));
  bluetooth.print(timestamp);
  bluetooth.print(F(","));

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
    Serial.print(F("No signal,No signal,"));
    bluetooth.print(F("No signal,No signal,"));
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

  // MPU6050 (temp removed)
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

  // HMC5883L
  Serial.print(sensorData.magX);
  Serial.print(F(","));
  Serial.print(sensorData.magY);
  Serial.print(F(","));
  Serial.print(sensorData.magZ);
  Serial.print(F(","));
  Serial.print(sensorData.heading);
  Serial.print(F(","));

  bluetooth.print(sensorData.magX);
  bluetooth.print(F(","));
  bluetooth.print(sensorData.magY);
  bluetooth.print(F(","));
  bluetooth.print(sensorData.magZ);
  bluetooth.print(F(","));
  bluetooth.print(sensorData.heading);
  bluetooth.print(F(","));

  // BMP085 (temp removed)
  Serial.print(sensorData.pressure);
  Serial.print(F(","));
  Serial.println(sensorData.altitude); // End of line

  bluetooth.print(sensorData.pressure);
  bluetooth.print(F(","));
  bluetooth.println(sensorData.altitude); // End of line

  // Optional debug
  Serial.println(F("Data transmitted successfully."));
}
