/*
 * OTIS Two Wheeled Self Balancing Robot
 * @author EThan Lew
 */

#include <Wire.h>
#include "PID_v1.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* MACROS */
#define SERIAL_BAUD 115200
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FAST_MODE 400000 
#define MPU_INT 19
/* Dual H-bridge macros */
#define DR0 16
#define PWM0 4
#define NEN0 0
#define DR1 18
#define PWM1 5
#define NEN1 17


/* System Tuning */
uint16_t gyro_bias[] = {10, 7, 14};
uint16_t accel_z_bias = 900;

/* MPU6050 Device Context */
MPU6050 mpu; 

/* MPU control/status vars */
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

uint8_t prevDuty = 0;

/* Processing variables */
float angle, angular_rate;

/* orientation/motion vars */
Quaternion q;          
VectorFloat gravity;   
int16_t gyro[3];        
float ypr[3];      

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady(){
  mpuInterrupt = true;
}

/* Setting PWM properties */
const int freq = 30000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int resolution = 8;
uint8_t dutyCycle = 200;

/* PID properties */
double originalSetpoint = -0.05
;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//double Kp = 90;
//double Kd = 7;
//double Ki = 100;
double Kp = 120;
double Kd = 5;
double Ki = 150;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
float curr =  0x7FFFFFFF;
float prev =  0x7FFFFFFF;
float diff = 0.0;

/* Serial In */
String serBuff = ""; 
 
void setup() {
  /* Set the serial baud rate*/
  Serial.begin(SERIAL_BAUD);
  /* Setup the I2C bus */
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FAST_MODE);
  /* Setup the IMU and relevant buffers */
  initialize_ypr();
  /* Setup the motors */
  initialize_pwm();

  /* Setup PID */
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255); 

 
}

void loop() {

  if(Serial.available() > 0)
  {
    float kp, ki, kd;
    
    serBuff = Serial.readString(); 
    
    char __serBuff[sizeof(serBuff)];
    serBuff.toCharArray(__serBuff, sizeof(__serBuff));    

    int result = sscanf(__serBuff, "%f, %f, %f", &kp, &ki, &kd);

    pid.SetTunings((double)kp, (double)ki, (double)kd);

    Serial.print("PID Gains Changed. P:");
    Serial.print(pid.GetKp());
    Serial.print(" I:");
    Serial.print(pid.GetKi());
    Serial.print(" D:");
    Serial.println(pid.GetKd());
  }
  
  fetch_ypr();
  input = ypr[1];

  pid.Compute();

  Serial.print(diff);
  Serial.print(" ");
  Serial.print(input);
  Serial.print(" ");
  Serial.print(output);

  if(output > 0.0){
    digitalWrite(DR1, true);
    digitalWrite(DR0, false);
    
  }
  else {
    digitalWrite(DR1, false);
    digitalWrite(DR0, true);
  }


  
  double duty_mag = abs(255.0/50.0*min(50, abs(output)));
  dutyCycle = (uint8_t)duty_mag;

  Serial.print(" ");
  Serial.println(dutyCycle);
  

  if(prevDuty != dutyCycle)
  {
    
    if(fabs(input) < 0.6){
      ledcWrite(pwmChannel1, dutyCycle); 
      ledcWrite(pwmChannel0, dutyCycle);
    } else {
      ledcWrite(pwmChannel0, 0); 
      ledcWrite(pwmChannel1, 0);
    }
  }


  if (fabs(input) > 1.0) {
    ledcWrite(pwmChannel0, 0); 
    ledcWrite(pwmChannel1, 0);
  }

  prevDuty = dutyCycle;
}

void initialize_pwm(){
  // Backwards: DR1 false, DR2 true
  // Forwards: DR1 true, DR2 false
  bool dir = true;
  
  pinMode(PWM1, OUTPUT);
  pinMode(DR1, OUTPUT);
  pinMode(NEN1, OUTPUT);

  pinMode(PWM0, OUTPUT);
  pinMode(DR0, OUTPUT);
  pinMode(NEN0, OUTPUT);

  digitalWrite(NEN1, 1);
  digitalWrite(NEN0, 1);
  digitalWrite(DR1, dir);
  digitalWrite(DR0, !dir);

  ledcSetup(pwmChannel0, freq, resolution);  
  ledcSetup(pwmChannel1, freq, resolution);  
  ledcAttachPin(PWM1, pwmChannel1);
  ledcAttachPin(PWM0, pwmChannel0);
}

void initialize_ypr(){
  /* Initialize the MPU */
  mpu.initialize();
  /* Verify the MPU */
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  /* Initialize the DMP */
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  /* Set the device biases appropiately */
  mpu.setXGyroOffset(gyro_bias[0]);
  mpu.setYGyroOffset(gyro_bias[1]);
  mpu.setZGyroOffset(gyro_bias[2]);
  mpu.setZAccelOffset(accel_z_bias);

  if (devStatus == 0)
  {
    /* turn on the DMP, now that it's ready */
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    /* Enable Interrupt Detection */
    Serial.println(F("Enabling interrupt detection (ESP32 pin 32)..."));
    pinMode(MPU_INT, INPUT_PULLUP);
    attachInterrupt(MPU_INT, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    /* set our DMP Ready flag so the main loop() function knows it's okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void fetch_ypr(){
    /* if programming failed, don't try to do anything */
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
  }
  /* reset interrupt flag and get INT_STATUS byte */
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  /* get current FIFO count */
  fifoCount = mpu.getFIFOCount();
  /* check for overflow (this should never happen unless our code is too inefficient) */
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    /* reset so we can continue cleanly */
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02)
  {
    /* wait for correct available data length, should be a VERY short wait */
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    /* read a packet from FIFO */
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    /* Get sensor data */
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    /* Note that 1.47 is zero tilt error */
    //angle = ypr[1] + 0.06;                   // 0.02 is center of gravity offset
    //angular_rate = -((double)gyro[1]/131.0); // converted to radian
    //Serial.println(ypr[0]);
  }
}
