/*
 *  By Henry Mellsop (hmellsop) for ENGR40B at Stanford University
 */

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_TCS34725.h"

#define BUFSIZE                        128   // Size of the read buffer for incoming data
#define VERBOSE_MODE                   true  // If set to 'true' enables debug output
#define BLE_READPACKET_TIMEOUT         50   // Timeout in ms waiting to read a response
#define BLUEFRUIT_SWUART_RXD_PIN       5    // Required for software serial!
#define BLUEFRUIT_SWUART_TXD_PIN       6   // Required for software serial!
#define BLUEFRUIT_UART_CTS_PIN         7   // Required for software serial!
#define BLUEFRUIT_UART_RTS_PIN         -1   // Optional, set to -1 if unused
#define BLUEFRUIT_UART_MODE_PIN        -1    // Set to -1 if unused
#define trigPin 3
#define echoPin 2

const int LEFT_MOTOR_PIN = 13;
const int RIGHT_MOTOR_PIN = 12;
const int RED_LED = 10;
const int GREEN_LED = 9;
const int STOPPED = 0;
const int MOVE = 1;
const int FAR_AWAY = 0;
const int TOO_CLOSE = 1;
const long DISTANCE_TOLERANCE = 70000;
const int RED_TOLERANCE = 30;
const int GREEN_TOLERANCE = 30;
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
uint16_t initial_red;
uint16_t initial_green;
uint16_t initial_blue;
int STATE;
int PROX_ALERT;
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
extern uint8_t packetbuffer[];
bool STOPPED_ALREADY;

Servo left_motor;
Servo right_motor;

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);
Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void getRGB(uint16_t* clear, uint16_t* red, uint16_t* green, uint16_t* blue) {
  tcs.setInterrupt(false);      // turn on LED
  delay(60);  // takes 50ms to read
  tcs.getRawData(red, green, blue, clear);
  tcs.setInterrupt(true);  // turn off LED
}

void setup(void)
{
  STATE = MOVE;
  PROX_ALERT = FAR_AWAY;
  STOPPED_ALREADY = false;
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(A0, INPUT); // A0 is the left sensor
  pinMode(A1, INPUT); // A1 is the right sensor
  
  
  // Colour Configuration Begins
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  uint16_t clear;
  getRGB(&initial_red, &initial_green, &initial_blue, &clear);

  // Bluetooth Configuration
  while (!Serial);
  delay(500);
  Serial.print(F("Initialising the Bluefruit LE module: "));
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );
  ble.echo(false);
  Serial.println("Requesting Bluefruit info:");
  ble.info();
  ble.verbose(false);
  while (! ble.isConnected()) {
      delay(500);
  }
  Serial.println("Connected.");
  ble.setMode(BLUEFRUIT_MODE_DATA);

  // Motor Configuration Begins
  left_motor.attach(LEFT_MOTOR_PIN);
  right_motor.attach(RIGHT_MOTOR_PIN);

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);
}

long calculateDistance() {
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * (340/2);
  Serial.print("Distance Calculated: ");
  Serial.print(distance);
  Serial.print("\n");
  return distance;
}

void containRobot(void) {
  while (true) {
    // define behaviour
    int leftSensor = analogRead(A0);
    int rightSensor = analogRead(A1);

    Serial.print("leftSensor: ");
    Serial.println(leftSensor);

    Serial.print("rightSensor: ");
    Serial.println(rightSensor);    

    if (leftSensor > 850) {
      left_motor.write(90);
      right_motor.write(95);
    } else if (rightSensor > 850) {
      left_motor.write(85);
      right_motor.write(90);
    } else {
      left_motor.write(95);
      right_motor.write(85);
    }
  }
}

void followLine(void) {
  int FOLLOW_STATE = 1;
  const int FORWARD = 1;
  const int TURN_LEFT = 2;
  const int TURN_RIGHT = 3;
  
  while (true) {
    // define behaviour
    int leftSensor = analogRead(A0);
    int rightSensor = analogRead(A1);

    Serial.print("leftSensor: ");
    Serial.println(leftSensor);

    Serial.print("rightSensor: ");
    Serial.println(rightSensor);    

    if (leftSensor > 850) {
      left_motor.write(90);
      right_motor.write(85);
    } else if (rightSensor > 850) {
      left_motor.write(95);
      right_motor.write(90);
    } else {
      left_motor.write(95);
      right_motor.write(85);
    }
  }
}

void loop(void)
{
  // Calculate distance in front of the vehicle
  long distance = calculateDistance();

  int leftSensor = analogRead(A0);
  int rightSensor = analogRead(A1);

  Serial.print("Left Sensor: ");
  Serial.println(leftSensor);

  Serial.print("Right Sensor: ");
  Serial.println(rightSensor);

  // Read colours
  uint16_t clear, red, green, blue;
  getRGB(&clear, &red, &green, &blue);
  
  if (distance < DISTANCE_TOLERANCE) {
    PROX_ALERT = TOO_CLOSE;
    if (!STOPPED_ALREADY) {
      left_motor.write(90);
      right_motor.write(90);
    }
    STOPPED_ALREADY = true;
  } else {
    PROX_ALERT = FAR_AWAY;
    STOPPED_ALREADY = false;
  }

  if (red > initial_red + RED_TOLERANCE) {
    STATE = STOPPED;
  } 
  if (green > initial_green + GREEN_TOLERANCE) {
    STATE = MOVE;
  }

  //Serial.print("State: ");
  //Serial.println(STATE);

  if (STATE == STOPPED) {
    left_motor.write(90);
    right_motor.write(90);
    digitalWrite(10, HIGH);
    digitalWrite(9, LOW);
    return;
  } else {
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
  }

  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  // Buttons
    if (packetbuffer[1] == 'B') {
      uint8_t buttnum = 0;
      buttnum = packetbuffer[2] - '0';
      boolean pressed = packetbuffer[3] - '0';
      Serial.print ("Button "); Serial.print(buttnum);
      if (pressed) {
        Serial.println(" pressed");
            if (buttnum == 6) {
              if (STATE == MOVE) {
                left_motor.write(80);
                right_motor.write(100);
              }
            } else if (buttnum == 5) {
              if (STATE == MOVE && PROX_ALERT == FAR_AWAY) {
                left_motor.write(100);
                right_motor.write(80);
              } 
            } else if (buttnum == 8) {
              left_motor.write(100);
              right_motor.write(100);
            } else if (buttnum == 7) {
              left_motor.write(80);
              right_motor.write(80);
            } else if (buttnum == 1) {
              followLine();
            } else if (buttnum == 2) {
              containRobot();
            }
      } else {
        int leftRead = left_motor.read();
        int rightRead = right_motor.read();

        if (leftRead == 80 && rightRead == 100) {
          if (buttnum == 6) {
            left_motor.write(90);
            right_motor.write(90);
          }
        } else if (leftRead == 100 && rightRead == 80) {
          if (buttnum == 5) {
            left_motor.write(90);
            right_motor.write(90);
          }
        } else if (leftRead == 100 && rightRead == 100) {
          if (buttnum == 8) {
            left_motor.write(90);
            right_motor.write(90);
          }
        } else if (leftRead == 80 && rightRead == 80) {
          if (buttnum == 7) {
            left_motor.write(90);
            right_motor.write(90);
          }
        }
      }
    }
}
