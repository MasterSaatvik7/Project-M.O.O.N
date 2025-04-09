////#include <SoftwareSerial.h>
////
////// Bluetooth RX/TX Pins
////#define BT_RX 1
////#define BT_TX 0
////
////// Bluetooth Serial
////SoftwareSerial bluetooth(BT_RX, BT_TX);
////
////void setup() {
////    Serial.begin(9600);
////    bluetooth.begin(9600);
////
////    Serial.println("Setup complete. Sending via Bluetooth...");
////}
////
////void loop() {
////    Serial.println("Hello World");
////    bluetooth.println("Hello World");
////    delay(2000); // Send every 2 seconds
////}
//
//
//#include <SoftwareSerial.h>
//
//#define BT_RX 1
//#define BT_TX 0
//
//SoftwareSerial bluetooth(BT_RX, BT_TX); // HC-05
//
//void setup() {
//  Serial.begin(9600);
//  bluetooth.begin(9600);
//  
//  Serial.println("Bluetooth Echo with Timestamp Ready");
//  bluetooth.println("Bluetooth Echo with Timestamp Ready");
//}
//
//void loop() {
//  if (Serial.available()) {
//    String userInput = Serial.readStringUntil('\n');
//    userInput.trim(); // remove newline, spaces
//    
//    unsigned long timeNow = millis();
//    String response = userInput + " @ " + String(timeNow) + "ms";
//    
//    Serial.println("Received: " + userInput);
//    Serial.println("Sending: " + response);
//    
//    bluetooth.println(response);
//  }
//}

#include <SoftwareSerial.h>

// Bluetooth RX/TX Pins
#define BT_RX 1
#define BT_TX 0

// Bluetooth Serial
SoftwareSerial bluetooth(BT_RX, BT_TX);

// Time of last hello message
unsigned long lastHelloTime = 0;
const unsigned long HELLO_INTERVAL = 5000; // Send hello every 5 seconds

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  
  // Initialize Bluetooth communication
  bluetooth.begin(9600);
  
  Serial.println("Bluetooth Echo With Timestamp");
  
  // Send initial hello message
  bluetooth.println("Hello World");
  lastHelloTime = millis();
}

void loop() {
  // Check if it's time to send a hello message
  if (millis() - lastHelloTime >= HELLO_INTERVAL) {
    bluetooth.println("Hello World");
    Serial.println("Sent: Hello World");
    lastHelloTime = millis();
  }
  
  // Check if there's data from Bluetooth to read
  if (Serial.available() > 0) {
    // Read the incoming command
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Log received command
//    Serial.println("Received Your: " + command);
    
    // Send back with timestamp
    String response = command + " [Time: " + String(millis()) + "ms]";
    Serial.println("Sent: " + response);
    bluetooth.println(response);
  }
  
  // Check if there's data from Serial to send over Bluetooth
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    Serial.println(input);
    bluetooth.println(input);
  }
}
