#include <SoftwareSerial.h>

// Left Wheel
#define in1 A0
#define in2 A1
// Right Wheel
#define in3 A2
#define in4 A3

// Bluetooth RX/TX Pins
#define BT_RX 1   // Connect to TX of Bluetooth module
#define BT_TX 0   // Connect to RX of Bluetooth module

// Bluetooth Serial
SoftwareSerial bluetooth(BT_RX, BT_TX);

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  Serial.begin(9600);
  bluetooth.begin(9600);
  
  Serial.println("DC Motor Control via Bluetooth");
  bluetooth.println("DC Motor Control via Bluetooth");
}

void loop() {
  // Check for serial input (from USB)
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.println("Received via Serial: " + command);
    bluetooth.println("Received via Bluetooth: " + command);
    
    handleCommand(command);
  }
  
  // Check for Bluetooth input
//  if (bluetooth.available() > 0) {
//    String command = bluetooth.readStringUntil('\n');
//    command.trim();
//    Serial.println("Received via BT: " + command);
//    bluetooth.println("Received via BT: " + command);
//    
//    handleCommand(command);
//  }
}

// Helper functions
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
