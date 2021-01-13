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

#define DEBUG
#define BENCHMARK

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  //#include "Wire.h"
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE
  #include "SBWire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
   ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define STATUS_LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define ERROR_LED_PIN 12 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
unsigned long loop_timer, loop_timer_stop, loop_count = 0, elapsed, mpuLoopCount = 0;
unsigned long mpuFifoTimerStart, mpuFifoTimerStart2, mpuFifoTimerStop;
unsigned long pidTimerStart, pidTimerStop;
byte eeprom_data[36];


// Flight mode definitions
#define AUTO_LEVELING 0
#define UNASSISTED 1
int flightMode;


// MPU control/status vars
int address, cal_int;
int16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
float acc_x_cal=-2222, acc_y_cal=1122, acc_z_cal=1260, gyro_x_cal=-25, gyro_y_cal=10, gyro_z_cal=-11;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize2;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize3;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize4;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint16_t fifoCount0;     // count of all bytes currently in FIFO
uint16_t fifoCounts[2][10];     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

bool fifoReady = false;
bool fifoInt = false;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprdeg[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector, in degrees
float yprset[3];        // [yaw, pitch, roll]   yaw/pitch/roll setting from remote control

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// ESC variables
unsigned long esc_loop_timer;
unsigned long timer_esc_1, timer_esc_2, timer_esc_3, timer_esc_4;
unsigned long esc_1, esc_2, esc_3, esc_4;
unsigned long escFL, escFR, escBL, escBR, throttle;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Receiver variabkes
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input[5];

float Cr=0, Cp=0;
float K1r=1, K1p=1,K2r=1, K2p=1, K3r=1, K3p=1;
float rLast=0, pLast=0;

// ================================================================
// ===               FLIGHT CONTROL ROUTINES                    ===
// ================================================================



void setup_MPU6050()
{    
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
}


void calib_MPU6050(bool doCalib)
{
  if(doCalib) {
    acc_x_cal = 0;
    acc_y_cal = 0;
    acc_z_cal = 0;
    gyro_x_cal = 0;
    gyro_y_cal = 0;
    gyro_z_cal = 0;

    Serial.println(F("Calibrating the gyro, this will take +/- 8 seconds"));
    Serial.print(F("Please wait"));
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++){              //Take 2000 readings for calibration.
      if(cal_int % 100 == 0)Serial.print(F("."));                //Print dot to indicate calibration.
      mpu.getMotion6(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);
      acc_x_cal += acc_x;                                //Ad roll value to gyro_roll_cal.
      acc_y_cal += acc_y;                                //Ad roll value to gyro_roll_cal.
      acc_z_cal += acc_z;                                //Ad roll value to gyro_roll_cal.
      gyro_x_cal += gyro_x;                                //Ad roll value to gyro_roll_cal.
      gyro_y_cal += gyro_y;                                //Ad roll value to gyro_roll_cal.
      gyro_z_cal += gyro_z;                                //Ad roll value to gyro_roll_cal.
      delay(4);                                                  //Wait 3 milliseconds before the next loop.
    }
    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    acc_x_cal /= 2000;                                       //Divide the roll total by 2000.
    acc_y_cal /= 2000;                                       //Divide the roll total by 2000.
    acc_z_cal /= 2000;                                       //Divide the roll total by 2000.
    gyro_x_cal /= 2000;                                      //Divide the pitch total by 2000.
    gyro_y_cal /= 2000;                                      //Divide the pitch total by 2000.
    gyro_z_cal /= 2000;                                      //Divide the pitch total by 2000.
  }
}


bool readFIFO()
{
  if (flightMode != AUTO_LEVELING) return false;
  
#ifdef BENCHMARK
  mpuFifoTimerStart = micros();
#endif

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
#ifdef BENCHMARK
  mpuFifoTimerStart2 = micros();
#endif

  mpuLoopCount += 1;

  if (mpuLoopCount == 8) {
    // There were too many failed attempts to read the FIFO, which is not good.
    // As a last resort, we try to reset the I2C bus
    Serial.println(F("Failed to read from FIFO for too long, resetting"));
    //Wire.reset();
    fifoCount = mpu.getFIFOCount();
    mpu.resetFIFO();
    mpu.resetDMP();
    return false;
  }

  if (mpuLoopCount > 12) {
    // We still cannot read the FIFO after the I2C reset, so we give up on the automatic leveling
    // and we switch to unassisted flight mode, for which the IMU readings are not needed
    flightMode = UNASSISTED;
    return false;
  }
  
  if (fifoCount < packetSize) return false;

  mpuLoopCount = 0;

  // check how many packets we have in the FIFO
  // if more than 1, reset the FIFO and read at the next cycle
  if (fifoCount >= packetSize3) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    mpuFifoTimerStop = micros();
    Serial.print(F("FIFO overflow 2: "));
    Serial.print(fifoCount);
    Serial.print(F(", "));
    Serial.println(packetSize);
    Serial.print("Elapsed time: ");
    Serial.println((mpuFifoTimerStop - loop_timer));
    //Serial.print("I2C frequency: ");
    //Serial.println(Wire.getClock());
    return false;
  // otherwise, read the available packet
  } else {
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    //mpu.dmpGetQuaternion(&q, fifoBuffer);
    //mpu.dmpGetGravity(&gravity, &q);
    //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //yprdeg[0] = ypr[0] * 180 / M_PI;
    //yprdeg[1] = ypr[1] * 180 / M_PI;
    //yprdeg[2] = ypr[2] * 180 / M_PI;

#ifdef BENCHMARK
    mpuFifoTimerStop = micros();
    if ((mpuFifoTimerStop - mpuFifoTimerStart) > 1800) {
      Serial.print("FIFO readout timeout exceeded: ");
      Serial.print(mpuFifoTimerStart2 - mpuFifoTimerStart);
      Serial.print("\t");
      Serial.println(mpuFifoTimerStop - mpuFifoTimerStart2);
    }
#endif
  }
  
  return true;
}


void printAngles()
{
    //Serial.print("FIFO count\t");
    //Serial.println(fifoCount);
    //elapsed = micros() - loop_timer;
#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.print(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    /*
      mpu.dmpGetAccel(&aa, fifoBuffer);
      Serial.print("\tRaw Accl XYZ\t");
      Serial.print(aa.x);
      Serial.print("\t");
      Serial.print(aa.y);
      Serial.print("\t");
      Serial.print(aa.z);
      mpu.dmpGetGyro(&gy, fifoBuffer);
      Serial.print("\tRaw Gyro XYZ\t");
      Serial.print(gy.x);
      Serial.print("\t");
      Serial.print(gy.y);
      Serial.print("\t");
      Serial.print(gy.z);
    */
    //Serial.println();

#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.print(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.print(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

#ifdef BENCHMARK
    Serial.print("\tFIFO took ");
    Serial.print(mpuFifoTimerStop - mpuFifoTimerStart);
    Serial.print("\tPID took ");
    Serial.println(pidTimerStop - pidTimerStart);
    //Serial.print("\tI2C frequency: ");
    //Serial.println(Wire.getClock());
#endif
}

#define HB_TIMEOUT 60
void heartBeat()
{
  if (flightMode != AUTO_LEVELING) {
    digitalWrite(STATUS_LED_PIN, LOW);
    return;
  }
  
  loop_count += 1;

  if(loop_count == HB_TIMEOUT) {
#ifdef DEBUG
    //Serial.print("DMP reading took ");
    //Serial.print(micros() - loop_timer);
    //Serial.print("\t");
    //Serial.println(fifoCount);
    printAngles();
#endif
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(13, HIGH);
  }

  if(loop_count == (HB_TIMEOUT+5)) {
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(STATUS_LED_PIN, LOW);
    loop_count = 0;
  }
}


void computeYPR()
{
  if (flightMode != AUTO_LEVELING) return;
  
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  //yprdeg[0] = ypr[0] * 180 / M_PI;
  //yprdeg[1] = ypr[1] * 180 / M_PI;
  //yprdeg[2] = ypr[2] * 180 / M_PI;
}



//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int normalize_receiver_channel(byte function){
  byte channel, reverse;                                                       //First we declare some local variables
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if (eeprom_data[function + 23] & 0b10000000) reverse = 1;                    //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  if(actual < center) {                                                        //The actual receiver value is lower than the center value
    if(actual < low) actual = low;                                             //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1) return 1500 + difference;                                 //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center) {                                                   //The actual receiver value is higher than the center value
    if(actual > high) actual = high;                                           //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1) return 1500 - difference;                                 //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}


void computeMotorSpeeds()
{
float dr, dp, r, p;
#ifdef BENCHMARK
  pidTimerStart = micros();
#endif

  if (flightMode == AUTO_LEVELING) {
    dp = ypr[1]-pLast;
    dr = ypr[2]-rLast;
    p = yprset[1]-ypr[1];
    r = yprset[2]-ypr[2];
    Cr=K1r*r+K2r*dr;
    Cp=K1p*p+K2p*dp;

    
    escFL = throttle - Cp + Cr;
    escFR = throttle - Cp - Cr;
    escBL = throttle + Cp + Cr;
    escBR = throttle + Cp - Cr;
    } else {
    
  }

#ifdef BENCHMARK
  pidTimerStop = micros();
  if ((pidTimerStop - pidTimerStart) > 1000) {
    Serial.print("PID timeout exceeded: ");
    Serial.println(pidTimerStop - pidTimerStart);
  }
#endif
}


void escSignalStart()
{ 
  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_esc_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_esc_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_esc_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_esc_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
}

void escSignalWait()
{
#ifdef BENCHMARK
  esc_loop_timer = micros();                                              //Read the current time.
  if ((timer_esc_1 <= esc_loop_timer) || (timer_esc_2 <= esc_loop_timer) || 
      (timer_esc_3 <= esc_loop_timer) || (timer_esc_4 <= esc_loop_timer)) {
        Serial.println("ESC timeout exceeded!!!");
  }
#endif

  while(PORTD >= 16) {                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_esc_1 <= esc_loop_timer) PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_esc_2 <= esc_loop_timer) PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_esc_3 <= esc_loop_timer) PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_esc_4 <= esc_loop_timer) PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
}



void storeCalibrations(int16_t cax, int16_t cay, int16_t caz, int16_t cgx, int16_t cgy, int16_t cgz)
{
  I2Cdev::writeWords(MPU6050_DEFAULT_ADDRESS, 0x06, 1, (uint16_t *)&cax);
  I2Cdev::writeWords(MPU6050_DEFAULT_ADDRESS, 0x06+2, 1, (uint16_t *)&cay);
  I2Cdev::writeWords(MPU6050_DEFAULT_ADDRESS, 0x06+4, 1, (uint16_t *)&caz);
  I2Cdev::writeWords(MPU6050_DEFAULT_ADDRESS, 0x13, 1, (uint16_t *)&cgx);
  I2Cdev::writeWords(MPU6050_DEFAULT_ADDRESS, 0x13+2, 1, (uint16_t *)&cgy);
  I2Cdev::writeWords(MPU6050_DEFAULT_ADDRESS, 0x13+4, 1, (uint16_t *)&cgz);
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // configure LED pins
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);

  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, HIGH);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial.begin(115200);

  setup_MPU6050();
  
  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again
  delay(1000);

  // compute the gyro and accelerometer offsets
  calib_MPU6050(false);

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  //mpu.setXAccelOffset(acc_x_cal);
  //mpu.setYAccelOffset(acc_y_cal);
  //mpu.setZAccelOffset(acc_z_cal);
  //mpu.setXGyroOffset(gyro_x_cal);
  //mpu.setYGyroOffset(gyro_y_cal);
  //mpu.setZGyroOffset(gyro_z_cal);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateAccel(6);
    //mpu.CalibrateGyro(6);
    storeCalibrations(acc_x_cal, acc_y_cal, acc_z_cal, gyro_x_cal, gyro_y_cal, gyro_z_cal);
    //storeCalibrations(1000, 2000, 3000, 0, 0, 0);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //Serial.println(F(")..."));
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    packetSize2 = packetSize * 2;
    packetSize3 = packetSize * 3;
    packetSize4 = packetSize * 4;
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure interrupt pins  
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
  //PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT4 (digital input 12)to trigger an interrupt on state change.

  loop_count = 0;

  mpu.resetFIFO();

  // initialize ESC signal lengths to minimum
  esc_1 = 1000;
  esc_2 = 1000;
  esc_3 = 1000;
  esc_4 = 1000;

  ypr[0] = 0;
  ypr[1] = 0;
  ypr[2] = 0;
  yprdeg[0] = 0;
  yprdeg[1] = 0;
  yprdeg[2] = 0;

  flightMode = AUTO_LEVELING;

  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);
  blinkState = LOW;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10, 11 or 12 changed state. This is used to read the receiver signals. 
//More information about this subroutine can be found in this video:
//https://youtu.be/bENjl1KQbvo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(last_channel_2 == 0){                                                //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    //Is input 11 high?
    if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  if (readFIFO()) {
    heartBeat();
  } else {
    computeMotorSpeeds();
  }

  loop_timer_stop = micros();
  if((loop_timer_stop - loop_timer) >= 4000) {
    Serial.print("Timeout exceeded! ");
    Serial.println(loop_timer_stop - loop_timer);
  }
  while((micros() - loop_timer) < 4000);

  loop_timer = micros();
  
  escSignalStart();
  computeYPR();
  escSignalWait();
}
