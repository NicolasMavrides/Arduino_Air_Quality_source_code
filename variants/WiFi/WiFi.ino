// Off-grid Air Quality Monitor dissertation project source code
// Written by Nicolas Mavrides, academic year 2018 - 19
// Supervised by Prof Hamish Cunningham
// Department of Computer Science

#include <GP2Y1010_DustSensor.h> // Library for the Sharp dust sensor
#include <WiFi.h>
#include <WiFiClient.h>

GP2Y1010_DustSensor dustsensor;

// Set up WiFi connection credentials *************************************************
const char *ssid = "Kestrel-Hotspot";   // Network name redacted. Replace with available network...
const char *password = "kestrel029"; // Password redacted. Replace with available network password...

// Set up sensor pins and connection settings *****************************************
const int DustLEDPin = 21;
const int DustPin = A12;
const int dhtPin = 14;
const int MQgasSensorPin = A8;   // MQ-135 gas sensor connected to Analog Pin 4.

// Device MAC address *****************************************************************
uint64_t chipid;                 // init uint for MAC address

// Sensor reading variables ***********************************************************
int gasSensorValue = 0;          // value read from gas sensor
float dustSensorValue = 0;       // value read from dust sensor

int gasSensorValues = 0;         // sum of 10 gas sensor readings to get average
float dustSensorValues = 0;      // sum of 10 dust sensor readings to get average
  
int avGasSensorValues = 0;       // average of gasSensorValues
float avDustSensorValues = 0;    // average of dustSensorValues

// this will be used to subtract from MQ-135 reading for calibration purposes
// calibration will set the sensor reading to zero for when CO2 concentration is approx 350ppm
int co2Calibration = 1200;       
int co2MinLevel = 350;
int co2MaxLevel = 5000;   // max CO2 value
int gasSensorValPPM = 0;  // gas sensor value after conversion to PPM


// initialisation entry point *********************************************************
void setup() {
  Serial.begin(9600);             // open the serial port, data rate @ 9600 bps
  pinMode(LED_BUILTIN, OUTPUT);   // initialize built-in LED as output
  
  dustsensor.begin(DustLEDPin, DustPin);
  dustsensor.setInputVolts(5);   // Voltage for dust sensor set to 5V
  dustsensor.setADCbit(12);      // ADCbit set to 12 bits as we are using ESP32

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("...");
  }

  Serial.println("Connection successful!");
  Serial.print("Device IP: ");
  Serial.println(WiFi.localIP());

  Serial.println("ESP32 up and running!");
  chipid=ESP.getEfuseMac();                  // Get MAC address and print tyhe High 2 byes and low 4 bytes
  Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));
  Serial.printf("%08X\n",(uint32_t)chipid);
  delay(2000); // delay 2 seconds
}


// task loop entry point ***************************************************************
void loop() {
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);           // turn the LED on (HIGH is the voltage level)
  delay(100);                                // wait for 100 milliseconds

  for (int x = 0; x < 10; x++) {

    gasSensorValue = analogRead(MQgasSensorPin);  // get current value from gas sensor

    // convert gas sensor value to PPM using calibrated value with mapping function in a realistic range
    gasSensorValPPM = map((gasSensorValue - co2Calibration),0,1023,co2MinLevel,co2MaxLevel);

    dustSensorValue = dustsensor.getDustDensity();     // get current value from dust sensor

    // add read values from both sensors into arrays
    gasSensorValues = gasSensorValues + gasSensorValPPM;
    dustSensorValues = dustSensorValues + dustSensorValue;
    delay(2000);
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
  
  // Reset sum and average values
  gasSensorValues = 0;
  dustSensorValues = 0;
  avGasSensorValues = 0;
  avDustSensorValues = 0;

  digitalWrite(LED_BUILTIN, LOW);            // turn the LED off by making the voltage LOW
  delay(100);                                // wait for 100 milliseconds
}
