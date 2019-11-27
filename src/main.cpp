#include <Arduino.h>

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v6.12)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-10 - Uses the new version of the DMP Firmware V6.12
//                 - Note: I believe the Teapot demo is broken with this versin as 
//                 - the fifo buffer structure has changed
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <FS.h>
#include <SD.h>
#include <SPI.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#define ACCEL_FACTOR 0.00006103608759
#define GYRO_FACTOR 0.06103608759

const uint8_t NUM_SAMPLES = 50;
const uint8_t SAMPLE_SIZE = 150;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
uint32_t packet_counter = 0;


// SD card constants
static const uint8_t SCLK_PIN = 25;
static const uint8_t MISO_PIN = 32;
static const uint8_t MOSI_PIN = 13;
static const uint8_t SS_PIN = 33;

// SD card related variables
SPIClass *hspi = NULL;
uint8_t sample_counter = 0;
char sample_str[NUM_SAMPLES][SAMPLE_SIZE];
uint32_t read_index = 0;
uint32_t write_index = 0;

TaskHandle_t taskWriteSD;
TaskHandle_t taskReadMPU;
SemaphoreHandle_t xMutex = NULL;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
  // Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    // Serial.println("Message appended");
  }
  else
  {
    Serial.println("Append failed");
  }
  file.close();
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

//Task1code: blinks an LED every 1000 ms
void readMPU( void * pvParameters ){
  Serial.print("readMPU running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        uint8_t num_pkgs = fifoCount/packetSize;

        for(int i = 0; i < num_pkgs; i++) {
          if(sample_counter < NUM_SAMPLES) {
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGyro(&gy, fifoBuffer);
            sprintf(sample_str[write_index], "%u;%f;%f;%f;%f;%f;%f\n",
              packet_counter++,
              aa.x * ACCEL_FACTOR, aa.y * ACCEL_FACTOR, aa.z * ACCEL_FACTOR,
              gy.x * GYRO_FACTOR, gy.y * GYRO_FACTOR, gy.z * GYRO_FACTOR);
            // Serial.print(sample_str[write_index]);
            write_index++;
            write_index %= NUM_SAMPLES;
            xSemaphoreTake(xMutex,portMAX_DELAY);
            sample_counter++;
            xSemaphoreGive(xMutex);
          }
          else {
            Serial.println("buffer cheio!");
          }
        }     
    }
  } 
}

//Task2code: blinks an LED every 700 ms
void writeSD( void * pvParameters ){
  Serial.print("writeSD running on core ");
  Serial.println(xPortGetCoreID());
  uint8_t num_writes = 0;

  for(;;){
    num_writes = sample_counter;
    delay(1);
    if(num_writes >= (NUM_SAMPLES*0.5)) {      
      char *tempString = (char *) malloc(num_writes*SAMPLE_SIZE*sizeof(char));
      tempString[0] = '\0';

      for(int i = 0; i < num_writes; i++) {
        strcat(tempString, sample_str[(read_index + i) % NUM_SAMPLES]);
      }
      appendFile(SD, "/mpu-data.txt", tempString);
      Serial.print(tempString);
      read_index = (read_index + num_writes) % NUM_SAMPLES;
      xSemaphoreTake(xMutex,portMAX_DELAY);
      sample_counter -= num_writes;
      xSemaphoreGive(xMutex);
      free(tempString);
    }
  }
}

void setup() {
  // init SD card SPI interface
  hspi = new SPIClass(HSPI);
  pinMode(SS_PIN, OUTPUT); //HSPI SS
  // SCLK = 25, MISO = 32, MOSI = 13, SS = 33
  hspi->begin(SCLK_PIN, MISO_PIN, MOSI_PIN, SS_PIN); //SCLK, MISO, MOSI, SS
  if (!SD.begin(SS_PIN, *hspi))
  {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    return;
  }
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    // mpu.setXGyroOffset(51);
    // mpu.setYGyroOffset(8);
    // mpu.setZGyroOffset(21);
    // mpu.setXAccelOffset(1150); 
    // mpu.setYAccelOffset(-50); 
    // mpu.setZAccelOffset(1060); 
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0); 
    mpu.setYAccelOffset(0); 
    // mpu.setZAccelOffset(0); 

    // // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(5);
        mpu.CalibrateGyro(5);
        Serial.println();
        // mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    xMutex = xSemaphoreCreateMutex();

    if (xMutex != NULL) {
      //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
      xTaskCreatePinnedToCore(
                        writeSD,   /* Task function. */
                        "writeSD",     /* name of task. */
                        10000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        1,           /* priority of the task */
                        &taskWriteSD,      /* Task handle to keep track of created task */
                        1);          /* pin task to core 1 */
        delay(500);
      //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
      xTaskCreatePinnedToCore(
                        readMPU,   /* Task function. */
                        "readMPU",     /* name of task. */
                        10000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        1,           /* priority of the task */
                        &taskReadMPU,      /* Task handle to keep track of created task */
                        0);          /* pin task to core 0 */                  
      delay(500); 
      }

    packet_counter = 0;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed=1;
  TIMERG0.wdt_wprotect=0;
}