#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <DHT11.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>

// Pin Definitions
#define BT_RX 1
#define BT_TX 0
#define GPS_RX 4
#define GPS_TX 3
#define DHT_PIN 9

// Default GPS coordinates (Chennai)
#define DEFAULT_LAT 12.8437
#define DEFAULT_LON 80.1545

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
  float mpuTemp;
  
  // HMC5883L
  float magX, magY, magZ;
  float heading;
  
  // BMP085
  float bmpTemp;
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
  sensorData.latitude = DEFAULT_LAT;
  sensorData.longitude = DEFAULT_LON;
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
        sensorData.latitude = DEFAULT_LAT;
        sensorData.longitude = DEFAULT_LON;
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
  
  // Store temperature
  sensorData.mpuTemp = temp.temperature;
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
  sensorData.bmpTemp = bmp.readTemperature();
  sensorData.pressure = bmp.readPressure();
  sensorData.altitude = bmp.readAltitude();
}

// Check for Bluetooth commands
void checkBluetoothCommands() {
  if (Serial.available() > 0) {
    String command = bluetooth.readStringUntil('\n');
    command.trim();
    
    // Log received command
    Serial.println("Received command: " + command);
    
    // Process command (add specific command handling as needed)
    String response = "Received: " + command + " [Time: " + String(millis()) + "ms]";
    Serial.println(response);
    bluetooth.println(response);
  }
}

// Transmit all sensor data over Bluetooth
void transmitData(unsigned long timestamp) {
  // Format: timestamp,DHT11(temp,humidity),GPS(lat,lon,sat,hdop),MPU6050(ax,ay,az,gx,gy,gz,t),HMC5883L(mx,my,mz,heading),BMP085(t,p,alt)
  String dataString = String(timestamp) + ",";
  
  // DHT11 data
  dataString += String(sensorData.temperature) + "," + String(sensorData.humidity) + ",";
  
  // GPS data
  dataString += String(sensorData.latitude, 6) + "," + String(sensorData.longitude, 6) + ",";
  dataString += String(sensorData.satellites) + "," + String(sensorData.hdop) + ",";
  
  // MPU6050 data
  dataString += String(sensorData.accelX) + "," + String(sensorData.accelY) + "," + String(sensorData.accelZ) + ",";
  dataString += String(sensorData.gyroX) + "," + String(sensorData.gyroY) + "," + String(sensorData.gyroZ) + ",";
  dataString += String(sensorData.mpuTemp) + ",";
  
  // HMC5883L data
  dataString += String(sensorData.magX) + "," + String(sensorData.magY) + "," + String(sensorData.magZ) + ",";
  dataString += String(sensorData.heading) + ",";
  
  // BMP085 data
  dataString += String(sensorData.bmpTemp) + "," + String(sensorData.pressure) + "," + String(sensorData.altitude);
  
  // Send data over Bluetooth
  Serial.println(dataString);
  bluetooth.println(dataString);
  
  // Also print to Serial for debugging
  Serial.println("Sent: " + dataString);
}
