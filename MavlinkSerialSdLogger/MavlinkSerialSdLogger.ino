#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif

#define BUZZER_PIN 12

#include <GCS_MAVLink.h>

#include <SD.h>

#define TELEMETRY_SPEED 57600
#define LED_PIN 13

#define NO_CARD_DELAY 500
#define REQUEST_DELAY 1000
#define FIND_FILE_DELAY 2000
#define FILE_ERROR_DELAY 100

static boolean      enable_mav_request = 0;
static boolean      waitingMAVBeats = 1;

void request_mavlink_rates();

File dataFile;

String name;

FastSerialPort1(Serial1);

void setup() {
    pinMode(LED_PIN, OUTPUT); 
    pinMode(BUZZER_PIN, OUTPUT); 
  // initialize Serial1:
  Serial1.begin(TELEMETRY_SPEED);
  Serial.begin(TELEMETRY_SPEED);

  mavlink_comm_0_port = &Serial1;

  Serial.print("Initializing SD card...");

  if (!SD.begin()) {
    Serial.println("Card faiLED_PIN, or not present");
      while(true)
      {
          digitalWrite(LED_PIN, HIGH);
          delay(NO_CARD_DELAY);
          digitalWrite(LED_PIN, LOW);
          delay(NO_CARD_DELAY);
      }
    return;
  }
  Serial.println("card initialized.");
  
  
  int i=0;

  do
  {
    name = String("t") + String(i) +String(".bin");
    char buffer[20];
    name.toCharArray(buffer, 20);
    dataFile = SD.open(buffer, O_CREAT | O_EXCL | O_WRITE);
    Serial.print("trying ");
    Serial.println(buffer);
    i ++;

    digitalWrite(LED_PIN, HIGH);
    delay(FIND_FILE_DELAY);
    digitalWrite(LED_PIN, LOW);
    delay(FIND_FILE_DELAY);

  } while (!dataFile);
  
    Serial.println(String("logging to ")+ name);
}

void loop()
{
  read_mavlink();

  if(enable_mav_request == 1){//Request rate control
    for(int n = 0; n < 3; n++){
      
      Serial.println("requesting datastream");
      digitalWrite(LED_PIN, HIGH);
      delay(REQUEST_DELAY);
      digitalWrite(LED_PIN, LOW);
      request_mavlink_rates();//Three times to certify it will be readed
      delay(50);
  }
  enable_mav_request = 0;
  waitingMAVBeats = 0;
  }
}
