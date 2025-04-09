#include <DHT11.h>

// Define pin for the DHT11 sensor
#define DHTPIN 6  // DHT11 connected to digital pin 9

// Create an instance of the DHT11 class
DHT11 dht11(DHTPIN);

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(9600);
    
    // Uncomment the line below to set a custom delay between sensor readings (in milliseconds).
    // dht11.setDelay(500); // Optional
}

void loop() {
    int temperature = 0;
    int humidity = 0;

    // Attempt to read temperature and humidity from DHT11
    int result = dht11.readTemperatureHumidity(temperature, humidity);

    if (result == 0) {
        // If successful, print the values
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print("°C, Humidity: ");
        Serial.print(humidity);
        Serial.println("%");
    } else {
        // Print error message if reading fails
        Serial.println(DHT11::getErrorString(result));
    }

    // Wait before next reading
    delay(2000);
}
