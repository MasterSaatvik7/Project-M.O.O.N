#include <Servo.h>
#include <SoftwareSerial.h>
#include <DHT11.h>

#define TURN_DELAY 1500    // 1.5 seconds 
#define NORMAL_SPEED 1500  // 50% speed for normal movement

// Motor Pins
#define MOTOR1_PIN 11  // Left motor
#define MOTOR2_PIN 9   // Right motor

// DHT Sensor Pin & Type
#define DHTPIN 3      // DHT11 connected to digital pin 3
DHT11 dht11(DHTPIN);

// Bluetooth RX/TX Pins
#define BT_RX 1
#define BT_TX 0

// Servo Objects for Motors
Servo motor1;
Servo motor2;

// DHT Sensor Object
//DHT dht(DHT_PIN, DHT_TYPE);

// Bluetooth Serial
SoftwareSerial bluetooth(BT_RX, BT_TX);

void setup() {
    Serial.begin(9600);
    bluetooth.begin(9600);
    
    Serial.println("BLDC Motor Control via Bluetooth");
    bluetooth.println("BLDC Motor Control via Bluetooth");

    motor1.attach(MOTOR1_PIN);
    motor2.attach(MOTOR2_PIN);

    // Initialize motors to stopped position
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);

//    dht.begin();
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        Serial.println("Received: " + command);
        bluetooth.println("Received: " + command);
        
        processCommand(command);
    } else {
        sendDHTData();
    }
}

//void processCommand(String command) {
//    int leftSpeed = 1000, rightSpeed = 1000; // Default: motors off
//
//    if (command.startsWith("L") && command.indexOf("R") > 0) {
//        // Independent Motor Control: Lx Rx format
//        int lIndex = command.indexOf("L");
//        int rIndex = command.indexOf("R");
//        int leftValue = command.substring(lIndex + 1, rIndex).toInt();
//        int rightValue = command.substring(rIndex + 1).toInt();
//
//        if (leftValue >= 0 && leftValue <= 100) {
////            leftSpeed = map(leftValue, 0, 100, 1000, 2000);
//            leftSpeed = 1000 + (2000 - 1000) * (leftValue/100);
//            Serial.println("Left motor speed: " + String(leftValue) + "%");
//            bluetooth.println("Left motor speed: " + String(leftValue) + "%");
//        }
//
//        if (rightValue >= 0 && rightValue <= 100) {
////            rightSpeed = map(rightValue, 0, 100, 1000, 2000);
//            rightSpeed = 1000 + (2000 - 1000) * (rightValue/100);
//            Serial.println("Right motor speed: " + String(rightValue) + "%");
//            bluetooth.println("Right motor speed: " + String(rightValue) + "%");
//        }
//    } 
//    else if (command.startsWith("L")) {
//        int leftValue = command.substring(1).toInt();
//        if (leftValue >= 0 && leftValue <= 100) {
//            leftSpeed = 1000 + (2000 - 1000) * (leftValue / 100.0);
//            Serial.println("Left motor speed: " + String(leftValue) + "%");
//            bluetooth.println("Left motor speed: " + String(leftValue) + "%");
//        }
//    } 
//    else if (command.startsWith("R")) {
//        int rightValue = command.substring(1).toInt();
//        if (rightValue >= 0 && rightValue <= 100) {
//            rightSpeed = 1000 + (2000 - 1000) * (rightValue / 100.0);
//            Serial.println("Right motor speed: " + String(rightValue) + "%");
//            bluetooth.println("Right motor speed: " + String(rightValue) + "%");
//        }
//    }
//    else if (command.toInt() >= 0 && command.toInt() <= 100) {
//        // Single-Speed Command: Both motors same speed
//        int speedValue = command.toInt();
////        leftSpeed = rightSpeed = map(speedValue, 0, 100, 1000, 2000);
//        leftSpeed = rightSpeed = 1000 + (2000 - 1000) * (speedValue/100.0);
//        Serial.println("Both motors speed: " + String(speedValue) + "%");
//        bluetooth.println("Both motors speed: " + String(speedValue) + "%");
//    } 
////    else if (command.equalsIgnoreCase("Left")) {
////        // Turning Left: Left 50%, Right 75%
//////        leftSpeed = map(50, 0, 100, 1000, 2000);
////        leftSpeed = 1300; //1000 + (2000 - 1000) * 0.5;
//////        rightSpeed = map(75, 0, 100, 1000, 2000);
////        rightSpeed = 1600;  //1000 + (2000 - 1000) * 0.75;
////        Serial.println("Turning Left");
////        bluetooth.println("Turning Left");
////    } 
////    else if (command.equalsIgnoreCase("Right")) {
////        // Turning Right: Left 75%, Right 50%
//////        leftSpeed = map(75, 0, 100, 1000, 2000);
////        leftSpeed = 1300;  //1000 + (2000 - 1000) * 0.75;
//////        rightSpeed = map(50, 0, 100, 1000, 2000);
////        rightSpeed = 1600;  //1000 + (2000 - 1000) * 0.5;
////        Serial.println("Turning Right");
////        bluetooth.println("Turning Right");
////    } 
//      else if (command.equalsIgnoreCase("left")) {
//        // Turning Left: Left 30%, Right 70%
//        leftSpeed = 1300;  // 30% speed
//        rightSpeed = 1700; // 70% speed
//        
//        Serial.println("Turning Left");
//        bluetooth.println("Turning Left");
//        
//        Serial.println("Left motor speed: 30%");
//        bluetooth.println("Left motor speed: 30%");
//        Serial.println("Right motor speed: 70%");
//        bluetooth.println("Right motor speed: 70%");
//    } 
//    else if (command.equalsIgnoreCase("right")) {
//        // Turning Right: Left 70%, Right 30%
//        leftSpeed = 1700;  // 70% speed
//        rightSpeed = 1300; // 30% speed
//        
//        Serial.println("Turning Right");
//        bluetooth.println("Turning Right");
//        
//        Serial.println("Left motor speed: 70%");
//        bluetooth.println("Left motor speed: 70%");
//        Serial.println("Right motor speed: 30%");
//        bluetooth.println("Right motor speed: 30%");
//    }
//    else {
//        Serial.println("Invalid Command!");
//        bluetooth.println("Invalid Command!");
//        return;
//    }
//
//    // Apply speeds to motors
//    motor1.writeMicroseconds(leftSpeed);
//    motor2.writeMicroseconds(rightSpeed);
//}


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
//    else if (command.equalsIgnoreCase("left")) {
//        leftSpeed = 1300;  // 30% speed
//        rightSpeed = 1700; // 70% speed
//        
//        Serial.println("Turning Left");
//        bluetooth.println("Turning Left");
//        
//        Serial.println("Left motor speed: 30%");
//        bluetooth.println("Left motor speed: 30%");
//        Serial.println("Right motor speed: 70%");
//        bluetooth.println("Right motor speed: 70%");
//        validCommand = true;
//    } 
//    else if (command.equalsIgnoreCase("right")) {
//        leftSpeed = 1700;  // 70% speed
//        rightSpeed = 1300; // 30% speed
//        
//        Serial.println("Turning Right");
//        bluetooth.println("Turning Right");
//        
//        Serial.println("Left motor speed: 70%");
//        bluetooth.println("Left motor speed: 70%");
//        Serial.println("Right motor speed: 30%");
//        bluetooth.println("Right motor speed: 30%");
//        validCommand = true;
//    }
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

void sendDHTData() {
//    float temperature = dht.readTemperature();
//    float humidity = dht.readHumidity();
      int temperature = 0;
      int humidity = 0;
      int result = dht11.readTemperatureHumidity(temperature, humidity);

//    if (!isnan(temperature) && !isnan(humidity)) {
//        Serial.println();
//        bluetooth.print("Temp: ");
//        bluetooth.print(temperature);
//        bluetooth.print("°C, Humidity: ");
//        bluetooth.print(humidity);
//        bluetooth.println("%");
//    } else {
//        Serial.println();
//        bluetooth.println("DHT Read Error!");
//    }
      if (result == 0) {
        // If successful, print the values to the serial monitor and transmit via Bluetooth.
        String data = "Temperature: " + String(temperature) + "°C, Humidity: " + String(humidity) + "%";
        
        // Output to Serial for debugging.
        Serial.println(data);
        
        // Transmit the data over Bluetooth.
        bluetooth.println(data);
    } else {
        // Print error message to Serial and transmit over Bluetooth.
        String errorMessage = DHT11::getErrorString(result);
        Serial.println(errorMessage);
        bluetooth.println(errorMessage);
    }

    delay(2000); // Send data every 2 seconds
}
