/* Off-grid Air Quality Monitor dissertation project 
   Source code for Prototype 2
   Written by Nicolas Mavrides, academic year 2018 - 19
   Supervised by Prof Hamish Cunningham
   Department of Computer Science

   Parts of the source code borrowed from the COM3505 labs by Hamish Cunningham,
   namely for the WiFi connection functionality
*/

#include <GP2Y1010_DustSensor.h> // Library for the Sharp dust sensor
#include <WiFi.h>
#include <WiFiClient.h>
#include <DHTesp.h>              // Library for temperature / humidity sensor 

GP2Y1010_DustSensor dustsensor;
DHTesp dht; // Declaring the Temperature and Humidity sensor

// Set up sensor pins and connection settings *****************************************
const int DustLEDPin = 21;
const int DustPin = A3;
const int dhtPin = 14;
const int MQgasSensorPin = 32;

// Set up WiFi connection credentials *************************************************
const char *ssid = "NETWORK_NAME";   // Network name redacted. Replace with available network...
const char *password = "PASSWORD"; // Password redacted. Replace with available network password...

// Device MAC address *****************************************************************
uint64_t chipid;                 // init uint for MAC address

// Sensor reading variables ***********************************************************
int gasSensorValue = 0;          // value read from gas sensor
float dustSensorValue = 0;       // value read from dust sensor

int gasSensorValues = 0;         // sum of 10 gas sensor readings to get average
float dustSensorValues = 0;      // sum of 10 dust sensor readings to get average
  
int avGasSensorValues = 0;       // average of gasSensorValues
float avDustSensorValues = 0;    // average of dustSensorValues

int temperature = 0;
int humidity = 0;

  
// this will be used to subtract from MQ-135 reading for calibration purposes
// calibration will set the sensor reading to zero for when CO2 concentration is approx 350ppm
int co2Calibration = 1200;       
int co2MinLevel = 350;
int co2MaxLevel = 5000;   // max CO2 value
int gasSensorValPPM = 0;  // gas sensor value after conversion to PPM


// initialisation entry point *********************************************************
void setup() {
  Serial.begin(9600);             // open the serial port, data rate @ 9600 bps
  pinMode(LED_BUILTIN, OUTPUT);     dht.setup(dhtPin, DHTesp::AM2302);  
  dustsensor.begin(DustLEDPin, DustPin);
  dustsensor.setInputVolts(5);   // Voltage for dust sensor set to 5V
  dustsensor.setADCbit(12);      // ADCbit set to 12 bits as we are using ESP32
// initialize built-in LED as output

// WiFi connection code, taken from the COM3505 lab exercises on WiFi connectivity
  // Wifi Connection temporarily disabled for prototype 2 for testing purposes...
  /* 
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  */
   
  Serial.println("Connection successful!");
  Serial.print("Device IP: ");
  Serial.println(WiFi.localIP());

  Serial.println("ESP32 up and running!");
  chipid=ESP.getEfuseMac();                  // Get MAC address and print tyhe High 2 byes and low 4 bytes
  Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));
  Serial.printf("%08X\n",(uint32_t)chipid);
  
  // Switch on built-in LED
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(13, HIGH);

}


// task loop entry point ***************************************************************
void loop() {
  delay(1000);
  
  for (int x = 0; x < 10; x++) {

    gasSensorValue = analogRead(MQgasSensorPin);  // get current value from gas sensor

    // convert gas sensor value to PPM using calibrated value with mapping function in a realistic range
    gasSensorValPPM = map((gasSensorValue - co2Calibration),0,1023,co2MinLevel,co2MaxLevel);

    dustSensorValue = dustsensor.getDustDensity();     // get current value from dust sensor

    // add read values from both sensors into arrays
    gasSensorValues = gasSensorValues + gasSensorValPPM;
    dustSensorValues = dustSensorValues + dustSensorValue;
    delay(2000); // delay for 2 seconds so to have a sampling rate of
                 // 1 reading every 2 seconds.
  }

  // Get averages
  avGasSensorValues = gasSensorValues/10;
  avDustSensorValues = dustSensorValues/10;

  // Print results
  Serial.print("Air pollution level: ");
  Serial.print(avGasSensorValues);
  Serial.println(" PPM");
  Serial.print("Dust Density: ");
  Serial.print(avDustSensorValues);
  Serial.println(" micrograms/m3");

  // Temperature/Humidity values
  TempAndHumidity newDhtValues = dht.getTempAndHumidity();
  temperature = newDhtValues.temperature;
  humidity = newDhtValues.humidity;

  Serial.print("Temperature: ");
  Serial.println(temperature);

  Serial.print("Humidity: ");
  Serial.println(humidity);

  // Reset sum and average values
  gasSensorValues = 0;
  dustSensorValues = 0;
  avGasSensorValues = 0;
  avDustSensorValues = 0;

  Serial.println("ESP going to sleep...");
  delay(500); // wait half a second
  esp_sleep_enable_timer_wakeup(120 * 1000000); // 10 times 1000000 to convert microseconds -> 10 seconds
  digitalWrite(12, HIGH);

  esp_deep_sleep_start();
  
  // Nothing below this line can be called due to deep-sleep. ESP will reboot and call setup() again.
    
}
