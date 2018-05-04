/********* IMPORTS *********/
#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <FastPID.h>

/********* GLOBAL DEFINITIONS *********/
// Escs
#define esc_1 6
#define esc_2 3
#define esc_3 4
#define esc_4 5
// IMU
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
// Radio
int MAX_RETRIES = 10;

/********* GLOBAL VARIABLES *********/
// Motors
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
int throttlePin = 0;
int throttle = 0;
// Radio
RF24 radio(9,10); // Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL }; // Radio pipe addresses for the 2 nodes to communicate.
// IMU
LSM9DS1 imu;
float accX = 0;
float accY = 0;
float accZ = 0;
// PID
float Kp=0.1, Ki=0.5, Kd=0;
int output_bits = 8;
bool output_signed = false;
FastPID xAxisPID(Kp, Ki, Kd, output_bits, output_signed);
FastPID yAxisPID(Kp, Ki, Kd, output_bits, output_signed);

/********* SETUP FUNCTIONS *********/
void motorSetup() {
  /* Attaches all the motors */
  esc1.attach(esc_1);
  esc2.attach(esc_2);
  esc3.attach(esc_3);
  esc4.attach(esc_4);
}

void radioSetup() {
  /* Creates reading and writing pipe for radio */
  radio.begin();
  radio.setPALevel(RF24_PA_MIN); 
  radio.setChannel(108);
  radio.setRetries(15,15);
  radio.startListening(); // Start listening
  radio.printDetails();
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);
}

void imuSetup() {
  /* Sets up IMU on I2C on MEGA */
  pinMode(53,OUTPUT);
  if (!imu.begin())
    Serial.println("Failed to start IMU!");
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
}

void setup() {
  Serial.begin(115200);
  printf_begin();
  radioSetup();
  motorSetup();
  imuSetup();
}

/********* Helper Functions ********/
void getData() {
  /* Reads from IMU */
  if ( ! imu.accelAvailable() )
    Serial.println("Failed to read Accel.");
  imu.readAccel();
  accX = imu.calcAccel(imu.ax);
  accY = imu.calcAccel(imu.ay);
  accZ = imu.calcAccel(imu.az);
}

bool send_until_success(float dat) {
  /* Sends float until target responds or max_retries has been reached. */
  for (int i = 0; i < MAX_RETRIES; i++) {
    bool ok = radio.write( &dat, sizeof(float) );
    if (ok) return true;
    delay(10);
  }
  return false;
}

void writeToMotors(uint8_t mtr1, uint8_t mtr2, uint8_t mtr3, uint8_t mtr4) {
  /* Writes values to the four motors */
  esc1.write(mtr1);
  esc2.write(mtr2);
  esc3.write(mtr3);
  esc4.write(mtr4);
}



/********* MAIN LOOP *********/
void loop() {
  int tmp = Serial.parseInt();
  if (tmp != 0) throttle = tmp;
  Serial.println(throttle);
  delay(100);
//  throttle = map(throttle, 0, 1023, 0, 179);
}

