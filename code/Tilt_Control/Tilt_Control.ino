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
#include "Math.h"

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

void loop(void)
{
/* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  double x, y, z, w;
  // Quaternions
  if (packetbuffer[1] == 'Q') {
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    w = parsefloat(packetbuffer+14);
    Serial.print("Quat\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.print('\t');
    Serial.print(w); Serial.println();
  }
  
  double roll, pitch, yaw;

  roll = atan2(2*(w * x + y * z), 1 - 2*(x * x + y * y));
  pitch = asin(2*(w * y - x * z));
  yaw = atan2(2*(w * z + x * y), 1 - 2*(y * y + z * z));
  pitch = pitch * -1;
  roll = roll * 20;

  Serial.print("Roll: ");
  Serial.println(roll);

  Serial.print("Pitch: ");
  Serial.println(pitch);

  int left_motor_speed = 70;
  left_motor_speed += 40 * pitch;
  left_motor_speed += roll;

  int right_motor_speed = 110;
  right_motor_speed -= 40 * pitch;
  right_motor_speed += roll;

  left_motor.write(left_motor_speed);
  right_motor.write(right_motor_speed);
}
