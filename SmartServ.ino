#include <SoftwareSerial.h>
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711_ADC.h>
#include <EEPROM.h>

// Pins for water level sensor and LED
const int lvlsensor = 9;
const int ledPin = 50;

//pins for RGB
const int resetbtn = 36 ;
const int full = 40 ;
const int lowfood = 38 ;

// Pins for SoftwareSerial
#define RX_PIN 7  // RX of Arduino to TX of SIM800C
#define TX_PIN 8  // TX of Arduino to RX of SIM800C

// Pins for servo and ultrasonic sensors
#define trigger1 4
#define echo1 3
#define trigger2 6
#define echo2 5
#define servopin 11

// Pins for temperature sensor and relay
const int RELAY_PIN  = A5; // Arduino pin connected to the relay's pin
const int SENSOR_PIN = 2;  // Arduino pin connected to DS18B20 sensor's DATA pin

// Pins for weight sensor and weight recording buttons
const int HX711_dout =  31; // mcu > HX711 dout pin
const int HX711_sck = 30; // mcu > HX711 sck pin
const int TARE_BUTTON_PIN = 12;  // Pin for tare button
const int FULL_BUTTON_PIN = 13;  // Pin for 100% full button

// Weight calibration variables and variables relatedd to weight sensing mechanism
const int calVal_eepromAdress = 0;
const int tareOffsetVal_eepromAdress = 4;
unsigned long t = 0;

double fullWeight = 0; // Variable for 100% filled weight
// long int smsInterval = 600000; // 10 mins in ms
// unsigned long lastSMSTime = 0, SMSTime = 0;
bool isFullbuttonPressed = false;

// Temperature threshold
const float TEMPERATURE_THRESHOLD = 55; // °C
const float TEMP_OFF = 60;

// Global variables
float time1 = 0, distance1 = 0, time2 = 0, distance2 = 0;
float temperature;
int relay_state;
float tempC, tempF;

//boolean values
bool ulsonic = false;
bool waterlvl = false;
bool weightsen = false;

// Objects for libraries
SoftwareSerial sim800(RX_PIN, TX_PIN);
Servo servo;
OneWire oneWire(SENSOR_PIN);
DallasTemperature DS18B20(&oneWire);
DallasTemperature sensor(&oneWire);
LiquidCrystal_I2C lcd(0x27, 16, 4);
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// Function prototypes
void sendATCommand(String command, const int timeout);
void sendSMS(String message);
void smoothServoMove(int targetAngle);

void setup() {
  // Set up water level sensor and LED pins
  pinMode(lvlsensor, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  //set up RGB
  pinMode(resetbtn, OUTPUT);
  pinMode(full, OUTPUT);
  pinMode(lowfood, OUTPUT);

  // Set up Serial communication
  Serial.begin(115200);
  delay(1000);
  sim800.begin(9600);
  delay(1000);

  // Check if the module is ready and set SMS text mode
  sendATCommand("AT", 1000);
  sendATCommand("AT+CMGF=1", 1000);

  // Set up servo and ultrasonic sensor pins
  pinMode(trigger1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trigger2, OUTPUT);
  pinMode(echo2, INPUT);
  servo.attach(servopin);
  servo.write(0);

  // Set up temperature sensor and LCD
  DS18B20.begin();
  lcd.init();
  lcd.backlight();
  pinMode(RELAY_PIN, OUTPUT);
  sensor.begin();

  // Setup weight recording buttons
  pinMode(TARE_BUTTON_PIN, INPUT);
  delay(10);
  pinMode(FULL_BUTTON_PIN, INPUT);

  // Setup loadcells for weight sensing
  LoadCell.begin();
  float calibrationValue;
  // calibrationValue = 12.71; //12.91, 11.27, 11.21

#if defined(ESP8266)|| defined(ESP32)
  EEPROM.begin(512);
#endif

  EEPROM.get(calVal_eepromAdress, calibrationValue); 

  //restore the zero offset value from eeprom:
  long tare_offset = 0;
  EEPROM.get(tareOffsetVal_eepromAdress, tare_offset);
  LoadCell.setTareOffset(tare_offset);
  boolean _tare = false; //set this to false as the value has been resored from eeprom

  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }
}

void loop() {
  // Water level sensor logic
  if (digitalRead(lvlsensor) == LOW) {
    waterlvl = false;
    digitalWrite(ledPin, LOW);
  } else {
    if (waterlvl == false){
      digitalWrite(ledPin, HIGH);
      Serial.println("Water level is low");
      sendSMS("Water level is low");
      waterlvl = true;
    }
  }

  // Ultrasonic sensor logic
  // Measure distance from sensor 1
  digitalWrite(trigger1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger1, LOW);
  time1 = pulseIn(echo1, HIGH);
  distance1 = time1 * 0.034 / 2;

  // Measure distance from sensor 2
  digitalWrite(trigger2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger2, LOW);
  time2 = pulseIn(echo2, HIGH);
  distance2 = time2 * 0.034 / 2;

  // Print distances to Serial Monitor
  Serial.print("Distance from Sensor 1: ");
  Serial.print(distance1);
  Serial.println(" cm");
  Serial.print("Distance from Sensor 2: ");
  Serial.print(distance2);
  Serial.println(" cm");

  // Control the servo based on distance readings
  if (distance1 <= 20 || distance2 <= 20) {
    
    smoothServoMove(0);
  
  } else {
    delay(900);
    smoothServoMove(100);
  }

  // Temperature sensor logic
  sensor.requestTemperatures();
  temperature = sensor.getTempCByIndex(0);
  if (temperature < TEMPERATURE_THRESHOLD)
    relay_state = HIGH;
  else if (temperature > TEMP_OFF)
    relay_state = LOW;

  digitalWrite(RELAY_PIN, relay_state);
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("°C => relay's state: ");
  Serial.println(relay_state);

  // Display temperature readings on the LCD
  DS18B20.requestTemperatures();
  tempC = DS18B20.getTempCByIndex(0);
  tempF = tempC * 9 / 5 + 32;
  lcd.setCursor(0, 0);
  lcd.print("Temp (C): ");
  lcd.print(tempC);
  lcd.print("  ");
  lcd.setCursor(0, 1);
  lcd.print("Temp (F): ");
  lcd.print(tempF);
  lcd.print("  ");

  delay(500); // Delay for 1 second before updating the display again

  // Weight sensor logic
  static boolean newDataReady = 0;
  const int serialPrintInterval = 1000; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(i);

      // Check if value 'i' less than 30% that of full weight
      if (i < fullWeight * 0.3 && isFullbuttonPressed) {
        // lastSMSTime = millis();
        Serial.println("The food on the chafing dish is depleting. Requesting refill!");
        sendSMS("The food on the chafing dish is depleting. Requesting refill!");
        
        digitalWrite(full, LOW);
        digitalWrite(lowfood, HIGH);
        digitalWrite(resetbtn, LOW);

        // SMSTime = lastSMSTime + smsInterval;
        isFullbuttonPressed = false;
      }
      newDataReady = 0;
      t = millis();
    }
    }

  // Check for tare button press
  if (digitalRead(TARE_BUTTON_PIN) == HIGH) {
    delay(50);  // Debounce
    if (digitalRead(TARE_BUTTON_PIN) == HIGH) {
      Serial.println("Tare button pressed");
      refreshOffsetValueAndSaveToEEprom();
      digitalWrite(full, LOW);
      digitalWrite(lowfood, LOW);
      digitalWrite(resetbtn, HIGH);
    }
  }

  // Check for full button press
  if (digitalRead(FULL_BUTTON_PIN) == HIGH) {
    delay(50);  // Debounce
    if (digitalRead(FULL_BUTTON_PIN) == HIGH) {
      Serial.println("Full button pressed");
      recordFullWeight();
      isFullbuttonPressed = true;
      digitalWrite(full, HIGH);
      digitalWrite(lowfood, LOW);
      digitalWrite(resetbtn, LOW);

      
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') refreshOffsetValueAndSaveToEEprom();
  }
}

void sendATCommand(String command, const int timeout) {
  sim800.println(command);
  long int time = millis();
  while ((time + timeout) > millis()) {
    while (sim800.available()) {
      char c = sim800.read();
      Serial.print(c);
    }
  }
}

void sendSMS(String message) {
  sim800.println("AT+CMGS=\"+94768426584\""); 
  delay(1000);
  sim800.println(message); // The message content
  delay(100);
  sim800.write(26); 
  delay(1000);

  while (sim800.available()) {
    Serial.write(sim800.read());
  }
}

void smoothServoMove(int targetAngle) {
  int currentAngle = servo.read();
  if (currentAngle < targetAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      servo.write(angle);
      delay(20);
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      servo.write(angle);
      delay(20);
    }
  }
}

// zero offset value (tare), calculate and save to EEprom:
void refreshOffsetValueAndSaveToEEprom() {
  long _offset = 0;
  Serial.println("Calculating tare offset value...");
  LoadCell.tare(); // calculate the new tare / zero offset value (blocking)
  _offset = LoadCell.getTareOffset(); // get the new tare / zero offset value
  EEPROM.put(tareOffsetVal_eepromAdress, _offset); // save the new tare / zero offset value to EEprom
#if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
#endif
  LoadCell.setTareOffset(_offset); // set value as library parameter (next restart it will be read from EEprom)
  Serial.print("New tare offset value:");
  Serial.print(_offset);
  Serial.print(", saved to EEprom adr:");
  Serial.println(tareOffsetVal_eepromAdress);
}

void recordFullWeight() {
  fullWeight = LoadCell.getData();
  Serial.print("100% full weight recorded: ");
  Serial.println(fullWeight);
}