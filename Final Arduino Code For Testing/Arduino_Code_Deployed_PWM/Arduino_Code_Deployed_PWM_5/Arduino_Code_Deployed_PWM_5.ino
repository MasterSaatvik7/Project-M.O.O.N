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
#define NORMAL_SPEED 1100  // 50% speed for normal movement

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

// Configuration - Use progmem for string constants to save RAM
const float DECLINATION_ANGLE = -1.12; // Local declination angle
const unsigned long TRANSMIT_INTERVAL = 1000; // 1 second between transmissions
unsigned long lastTransmitTime = 0;

// System state
bool systemStarted = false;

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
const char PSTR_START_INST[] PROGMEM = "Send 'start' command to activate motors";
const char PSTR_MPU_INIT[] PROGMEM = "Initializing MPU6050... ";
const char PSTR_HMC_INIT[] PROGMEM = "Initializing HMC5883L... ";
const char PSTR_BMP_INIT[] PROGMEM = "Initializing BMP085... ";
const char PSTR_SUCCESS[] PROGMEM = "Success!";
const char PSTR_FAILED[] PROGMEM = "Failed!";
const char PSTR_DHT_READY[] PROGMEM = "DHT11 ready on pin ";
const char PSTR_SYS_ACT[] PROGMEM = "System activated";
const char PSTR_MOT_READY[] PROGMEM = "Motors ready for control";
const char PSTR_STOP_MOT[] PROGMEM = "Stopping Motors";
const char PSTR_BOTH_STOP[] PROGMEM = "Both motors stopped";
const char PSTR_STOP_TURN[] PROGMEM = "Stopping before turn";
const char PSTR_TURN_LEFT[] PROGMEM = "Turning Left";
const char PSTR_TURN_RIGHT[] PROGMEM = "Turning Right";
const char PSTR_LEFT_SPEED[] PROGMEM = "Left motor speed: ";
const char PSTR_RIGHT_SPEED[] PROGMEM = "Right motor speed: ";
const char PSTR_NORMAL_SPEED[] PROGMEM = "Resuming normal speed";
const char PSTR_BOTH_SPEED[] PROGMEM = "Both motors speed: ";
const char PSTR_NOT_ACTIVE[] PROGMEM = "System not activated. Send 'start' command first";
const char PSTR_ERR_CMD[] PROGMEM = "Error: Invalid Command!";
const char PSTR_VALID_CMD[] PROGMEM = "Valid commands are:";
const char PSTR_CMD_START[] PROGMEM = "- 'start': Activates the system";
const char PSTR_CMD_STOP[] PROGMEM = "- 'stop': Stops both motors";
const char PSTR_CMD_LEFT[] PROGMEM = "- 'left': Turn left";
const char PSTR_CMD_RIGHT[] PROGMEM = "- 'right': Turn right";
const char PSTR_CMD_L[] PROGMEM = "- 'L[0-100]': Set left motor speed (0-100%)";
const char PSTR_CMD_R[] PROGMEM = "- 'R[0-100]': Set right motor speed (0-100%)";
const char PSTR_CMD_LR[] PROGMEM = "- 'L[0-100]R[0-100]': Set both motor speeds";
const char PSTR_CMD_SPEED[] PROGMEM = "- '[0-100]': Set both motors to same speed";
const char PSTR_RCV_CMD[] PROGMEM = "Received command: ";
const char PSTR_NO_SIGNAL[] PROGMEM = "No signal";

// Helper function to print flash strings to save RAM
void printProgString(const char* str, Stream &output) {
  char c;
  for (uint8_t i = 0; (c = pgm_read_byte(str + i)) != 0; i++) {
    output.print(c);
  }
}

void setup() {
  // Initialize serial communications
  Serial.begin(115200);
  bluetooth.begin(9600);
  gpsSerial.begin(4800);
  
  printProgString(PSTR_INIT, Serial);
  Serial.println();
  printProgString(PSTR_INIT, bluetooth);
  bluetooth.println();
  
  // Initialize I2C sensors
  Wire.begin();
  initializeSensors();
  
  // Initialize motors
  motor1.attach(MOTOR1_PIN);
  motor2.attach(MOTOR2_PIN);
  
  // Explicitly stop motors at initialization
  motor1.writeMicroseconds(1000);  // 1000 = stopped
  motor2.writeMicroseconds(1000);  // 1000 = stopped
  
  // Add clear confirmation messages that motors are stopped
  Serial.println(F("Motors initialized and STOPPED."));
  Serial.println(F("Motors will remain stopped until 'start' command is received."));
  bluetooth.println(F("Motors initialized and STOPPED."));
  bluetooth.println(F("Motors will remain stopped until 'start' command is received."));
  
  printProgString(PSTR_SETUP, Serial);
  Serial.println();
  printProgString(PSTR_START_INST, Serial);
  Serial.println();
  
  printProgString(PSTR_SETUP, bluetooth);
  bluetooth.println();
  printProgString(PSTR_START_INST, bluetooth);
  bluetooth.println();
  
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

// Read HMC5883L magnetometer - Fixed for compatibility
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

// Check for Bluetooth commands
void checkBluetoothCommands() {
  if (bluetooth.available() > 0) {  // Changed from Serial.available() to bluetooth.available()
    String command = bluetooth.readStringUntil('\n');
    command.trim();
    
    // Log received command
    printProgString(PSTR_RCV_CMD, Serial);
    Serial.println(command);
    
    // Process command for motor control
    processCommand(command);
  }
  
  // Also check Serial for commands (for debugging)
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    processCommand(command);
  }
}

// Process commands for motor control
void processCommand(String command) {
    int leftSpeed = 1000, rightSpeed = 1000; // Default: motors off
    bool validCommand = false;  // Flag to track if command was valid
    bool updateMotors = false;  // Only update motors if this flag is set

    // Check for start command first
    if (command.equalsIgnoreCase("start") && !systemStarted) {
        systemStarted = true;
        // Keep motors stopped after system start
        leftSpeed = 1000;  // Motors remain stopped (0% speed)
        rightSpeed = 1000; // Motors remain stopped (0% speed)
        
        printProgString(PSTR_SYS_ACT, Serial);
        Serial.println();
        printProgString(PSTR_MOT_READY, Serial);
        Serial.println();
        Serial.println(F("Motors are ready but still STOPPED. Send speed command to begin movement."));
        
        printProgString(PSTR_SYS_ACT, bluetooth);
        bluetooth.println();
        printProgString(PSTR_MOT_READY, bluetooth);
        bluetooth.println();
        bluetooth.println(F("Motors are ready but still STOPPED. Send speed command to begin movement."));
        
        validCommand = true;
        updateMotors = true;  // Update motors to ensure they're stopped
    }
    // Only process other commands if system is started
    else if (systemStarted) {
        // Check text commands first, before numeric checks
        if (command.equalsIgnoreCase("stop")) {
            leftSpeed = 1000;  // 0% speed
            rightSpeed = 1000; // 0% speed
            
            printProgString(PSTR_STOP_MOT, Serial);
            Serial.println();
            printProgString(PSTR_BOTH_STOP, Serial);
            Serial.println();
            
            printProgString(PSTR_STOP_MOT, bluetooth);
            bluetooth.println();
            printProgString(PSTR_BOTH_STOP, bluetooth);
            bluetooth.println();
            
            validCommand = true;
            updateMotors = true;
        }
        else if (command.equalsIgnoreCase("left")) {
            // Step 1: Stop both motors
            motor1.writeMicroseconds(1000);
            motor2.writeMicroseconds(1000);
            printProgString(PSTR_STOP_TURN, Serial);
            Serial.println();
            printProgString(PSTR_STOP_TURN, bluetooth);
            bluetooth.println();
            delay(TURN_DELAY);
            
            // Step 2: Execute turn
            motor1.writeMicroseconds(1000);  // 0% speed
            motor2.writeMicroseconds(1700);  // 70% speed
            printProgString(PSTR_TURN_LEFT, Serial);
            Serial.println();
            printProgString(PSTR_LEFT_SPEED, Serial);
            Serial.println("0%");
            printProgString(PSTR_RIGHT_SPEED, Serial);
            Serial.println("70%");
            
            printProgString(PSTR_TURN_LEFT, bluetooth);
            bluetooth.println();
            printProgString(PSTR_LEFT_SPEED, bluetooth);
            bluetooth.println("0%");
            printProgString(PSTR_RIGHT_SPEED, bluetooth);
            bluetooth.println("70%");
            
            delay(TURN_DELAY);
            
            // Step 3: Resume normal speed
            leftSpeed = NORMAL_SPEED;
            rightSpeed = NORMAL_SPEED;
            printProgString(PSTR_NORMAL_SPEED, Serial);
            Serial.println();
            printProgString(PSTR_BOTH_SPEED, Serial);
            Serial.println("50%");
            
            printProgString(PSTR_NORMAL_SPEED, bluetooth);
            bluetooth.println();
            printProgString(PSTR_BOTH_SPEED, bluetooth);
            bluetooth.println("50%");
            
            validCommand = true;
            updateMotors = true;
        } 
        else if (command.equalsIgnoreCase("right")) {
            // Step 1: Stop both motors
            motor1.writeMicroseconds(1000);
            motor2.writeMicroseconds(1000);
            printProgString(PSTR_STOP_TURN, Serial);
            Serial.println();
            printProgString(PSTR_STOP_TURN, bluetooth);
            bluetooth.println();
            delay(TURN_DELAY);
            
            // Step 2: Execute turn
            motor1.writeMicroseconds(1700);  // 70% speed
            motor2.writeMicroseconds(1000);  // 0% speed
            printProgString(PSTR_TURN_RIGHT, Serial);
            Serial.println();
            printProgString(PSTR_LEFT_SPEED, Serial);
            Serial.println("70%");
            printProgString(PSTR_RIGHT_SPEED, Serial);
            Serial.println("0%");
            
            printProgString(PSTR_TURN_RIGHT, bluetooth);
            bluetooth.println();
            printProgString(PSTR_LEFT_SPEED, bluetooth);
            bluetooth.println("70%");
            printProgString(PSTR_RIGHT_SPEED, bluetooth);
            bluetooth.println("0%");
            
            delay(TURN_DELAY);
            
            // Step 3: Resume normal speed
            leftSpeed = NORMAL_SPEED;
            rightSpeed = NORMAL_SPEED;
            printProgString(PSTR_NORMAL_SPEED, Serial);
            Serial.println();
            printProgString(PSTR_BOTH_SPEED, Serial);
            Serial.println("50%");
            
            printProgString(PSTR_NORMAL_SPEED, bluetooth);
            bluetooth.println();
            printProgString(PSTR_BOTH_SPEED, bluetooth);
            bluetooth.println("50%");
            
            validCommand = true;
            updateMotors = true;
        }
        else if (command.startsWith("L") && command.indexOf("R") > 0) {
            int lIndex = command.indexOf("L");
            int rIndex = command.indexOf("R");
            int leftValue = command.substring(lIndex + 1, rIndex).toInt();
            int rightValue = command.substring(rIndex + 1).toInt();

            if (leftValue >= 0 && leftValue <= 100 && rightValue >= 0 && rightValue <= 100) {
                leftSpeed = 1000 + (leftValue * 10);
                rightSpeed = 1000 + (rightValue * 10);
                
                printProgString(PSTR_LEFT_SPEED, Serial);
                Serial.println(String(leftValue) + "%");
                printProgString(PSTR_RIGHT_SPEED, Serial);
                Serial.println(String(rightValue) + "%");
                
                printProgString(PSTR_LEFT_SPEED, bluetooth);
                bluetooth.println(String(leftValue) + "%");
                printProgString(PSTR_RIGHT_SPEED, bluetooth);
                bluetooth.println(String(rightValue) + "%");
                
                validCommand = true;
                updateMotors = true;
            }
        } 
        else if (command.startsWith("L")) {
            int leftValue = command.substring(1).toInt();
            if (leftValue >= 0 && leftValue <= 100) {
                leftSpeed = 1000 + (leftValue * 10);
                
                printProgString(PSTR_LEFT_SPEED, Serial);
                Serial.println(String(leftValue) + "%");
                
                printProgString(PSTR_LEFT_SPEED, bluetooth);
                bluetooth.println(String(leftValue) + "%");
                
                validCommand = true;
                updateMotors = true;
            }
        } 
        else if (command.startsWith("R")) {
            int rightValue = command.substring(1).toInt();
            if (rightValue >= 0 && rightValue <= 100) {
                rightSpeed = 1000 + (rightValue * 10);
                
                printProgString(PSTR_RIGHT_SPEED, Serial);
                Serial.println(String(rightValue) + "%");
                
                printProgString(PSTR_RIGHT_SPEED, bluetooth);
                bluetooth.println(String(rightValue) + "%");
                
                validCommand = true;
                updateMotors = true;
            }
        }
        else if (isNumber(command) && command.toInt() >= 0 && command.toInt() <= 100) {
            // Single-Speed Command: Both motors same speed
            int speedValue = command.toInt();
            leftSpeed = rightSpeed = 1000 + (speedValue * 10);
            
            printProgString(PSTR_BOTH_SPEED, Serial);
            Serial.println(String(speedValue) + "%");
            
            printProgString(PSTR_BOTH_SPEED, bluetooth);
            bluetooth.println(String(speedValue) + "%");
            
            validCommand = true;
            updateMotors = true;
        }
    }
    else {
        // System not started yet
        printProgString(PSTR_NOT_ACTIVE, Serial);
        Serial.println();
        Serial.println(F("Motors are STOPPED. Send 'start' command first."));
        
        printProgString(PSTR_NOT_ACTIVE, bluetooth);
        bluetooth.println();
        bluetooth.println(F("Motors are STOPPED. Send 'start' command first."));
        
        validCommand = true; // Avoid showing the error message below
    }

    // Only update motor speeds if we have a valid command that should change motor state
    if (validCommand && updateMotors && systemStarted) {
        motor1.writeMicroseconds(leftSpeed);
        motor2.writeMicroseconds(rightSpeed);
    }
    
    // Show help message for invalid commands
    if (!validCommand) {
        printProgString(PSTR_ERR_CMD, Serial);
        Serial.println();
        printProgString(PSTR_VALID_CMD, Serial);
        Serial.println();
        printProgString(PSTR_CMD_START, Serial);
        Serial.println();
        printProgString(PSTR_CMD_STOP, Serial);
        Serial.println();
        printProgString(PSTR_CMD_LEFT, Serial);
        Serial.println();
        printProgString(PSTR_CMD_RIGHT, Serial);
        Serial.println();
        printProgString(PSTR_CMD_L, Serial);
        Serial.println();
        printProgString(PSTR_CMD_R, Serial);
        Serial.println();
        printProgString(PSTR_CMD_LR, Serial);
        Serial.println();
        printProgString(PSTR_CMD_SPEED, Serial);
        Serial.println();
        
        printProgString(PSTR_ERR_CMD, bluetooth);
        bluetooth.println();
        printProgString(PSTR_VALID_CMD, bluetooth);
        bluetooth.println();
        printProgString(PSTR_CMD_START, bluetooth);
        bluetooth.println();
        printProgString(PSTR_CMD_STOP, bluetooth);
        bluetooth.println();
        printProgString(PSTR_CMD_LEFT, bluetooth);
        bluetooth.println();
        printProgString(PSTR_CMD_RIGHT, bluetooth);
        bluetooth.println();
        printProgString(PSTR_CMD_L, bluetooth);
        bluetooth.println();
        printProgString(PSTR_CMD_R, bluetooth);
        bluetooth.println();
        printProgString(PSTR_CMD_LR, bluetooth);
        bluetooth.println();
        printProgString(PSTR_CMD_SPEED, bluetooth);
        bluetooth.println();
    }
}

// Helper function to check if a string is a valid number
bool isNumber(String str) {
    for(uint8_t i = 0; i < str.length(); i++) {
        if(i == 0 && str.charAt(i) == '-') continue; // Handle negative numbers
        if(!isDigit(str.charAt(i))) return false;
    }
    return true;
}

// Transmit all sensor data over Bluetooth and Serial
void transmitData() {
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
