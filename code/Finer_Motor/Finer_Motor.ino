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
int leftMotorValue = 90;
int rightMotorValue = 90;
double speedMultiplier;


bool forwardPressed;
bool backPressed;
bool leftPressed;
bool rightPressed;

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

void setup(void)
{
  forwardPressed = false;
  backPressed = false;
  leftPressed = false;
  rightPressed = false;

  speedMultiplier = 1.0;
  
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

void loop(void)
{
    if (!leftPressed && !rightPressed && !forwardPressed && !backPressed) {
      left_motor.write(90);
      right_motor.write(90);
    }

    if (leftPressed) {
      Serial.println("Left Pressed");
    }
    if (rightPressed) {
      Serial.println("Right Pressed");
    }
    if (forwardPressed) {
      Serial.println("Forward Pressed");
    }
    if (backPressed) {
      Serial.println("Back Pressed");
    }

    left_motor.write(leftMotorValue);
    right_motor.write(rightMotorValue);

    Serial.println(leftMotorValue);
    Serial.println(rightMotorValue);
    
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
              backPressed = true;
            } if (buttnum == 5) {
              forwardPressed = true; 
            } if (buttnum == 8) {
              rightPressed = true;
            } else if (buttnum == 7) {
              leftPressed = true;
            } else if (buttnum == 1) {
              speedMultiplier += 0.2;
            } else if (buttnum == 2) {
              speedMultiplier -= 0.2;
            }
      } else {
        Serial.print("Button Just Released:");
        Serial.println(buttnum);
        if (buttnum == 6) {
          backPressed = false;
        } if (buttnum == 5) {
          forwardPressed = false;
        } if (buttnum == 8) {
          Serial.println("Right Released");
          rightPressed = false;
          if (rightPressed) Serial.println("Still Pressed");
        } if (buttnum == 7) {
          Serial.println("Left Released");
          leftPressed = false;
          if (rightPressed) Serial.println("Still Pressed");
        }
      }
    }

    leftMotorValue = 90;
    rightMotorValue = 90;

    if (forwardPressed) {
      leftMotorValue += 20 * speedMultiplier;
      rightMotorValue -= 20 * speedMultiplier;
      Serial.println("Forward Pressed");
    }
    if (backPressed) {
      leftMotorValue -= 20 * speedMultiplier;
      rightMotorValue += 20 * speedMultiplier;
      Serial.println("Back Pressed");
    }
    if (rightPressed) {
      leftMotorValue += 10 * speedMultiplier;
      rightMotorValue += 10 * speedMultiplier;
      Serial.println("Right Pressed");
    }
    if (leftPressed) {
      rightMotorValue -= 10 * speedMultiplier;
      leftMotorValue -= 10 * speedMultiplier;
      Serial.println("Left Pressed");
    }
}
