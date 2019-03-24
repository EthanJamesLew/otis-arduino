/*
 * OTIS Two Wheeled Self Balancing Robot
 * @author EThan Lew
 */

#include <Wire.h>
#include "BluetoothSerial.h"
#include "PID_v1.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//#include <WiFi.h>

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
uint8_t dutyCycle0 = 200;
uint8_t dutyCycle1 = 200;

/* PID properties */
double originalSetpoint = -0.05;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//double Kpt = 90;
//double Kdt = 7;
//double Kit = 100;
double Kpt = 120;
double Kdt = 5;
double Kit = 150;

double setpointy = 100.0;
double inputy, outputy;
double Kpy = 15;
double Kdy = 2;
double Kiy = 0;
PID pidTilt(&input, &output, &setpoint, Kpt, Kit, Kdt, DIRECT);
PID pidYaw(&inputy, &outputy, &setpointy, Kpy, Kiy, Kdy, DIRECT);

float curr =  0x7FFFFFFF;
float prev =  0x7FFFFFFF;
float diff = 0.0;


/* Motor output */
double out0, out1;

uint8_t remote_buff[4]; 
const uint16_t* remote_16 = (const uint16_t*)remote_buff;

/* Serial In */
String serBuff = ""; 

/* Websockets Control */

#define SSID "OTIS-bot"
#define PASSWORD "japery2019"
#define SERVER_PORT 4141

#define PACKET_SIZE 4

#define IS_SERVER

enum Protocol{
    TILT_SET, 
    YAW_SET
};

/*
WiFiServer server(SERVER_PORT);
WiFiClient client;
size_t len;
*/
uint16_t tiltNumber, yawNumber;

BluetoothSerial SerialBT;
 
void setup() {
  /* Create Bluetooth Serial */
  SerialBT.begin("OTIS-BOT");
  /* Set the serial baud rate*/
  Serial.begin(SERIAL_BAUD);
  /* Setup the I2C bus */
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FAST_MODE);
  /* Setup the IMU and relevant buffers */
  initialize_ypr();
  /* Setup the motors */
  initialize_pwm();

  /* Setup PID */
  pidTilt.SetMode(AUTOMATIC);
  pidTilt.SetSampleTime(10);
  pidTilt.SetOutputLimits(-255, 255); 

  pidYaw.SetMode(AUTOMATIC);
  pidYaw.SetSampleTime(10);
  pidYaw.SetOutputLimits(-255, 255); 

  /* Setup the AP */
  //WiFi.mode(WIFI_AP);
  //WiFi.softAP(SSID, PASSWORD);
  //server.begin();  

 
}



void loop() {

  fetch_ypr();
  input = ypr[1];
  inputy = ypr[0];

  if(setpointy > 4.0){
    setpointy = ypr[0];
  }

  if (SerialBT.available() > 0)
  {
    SerialBT.readBytes(remote_buff, 4);

    tiltNumber = remote_16[0];
    yawNumber = remote_16[1];

    float tiltDecode = ((float)tiltNumber) /(65535.0f)*(2.0f) - 1.05f;
    float yawDecode = ((float)yawNumber) /(65535.0f)*(2.0f) - 1.0f;

    tiltDecode *= 0.3;
    yawDecode *= 0.2;

    Serial.print("Received. Tilt:");
    Serial.print(tiltDecode);
    Serial.print(" Yaw:");
    Serial.println(yawDecode);

    setpoint = tiltDecode;
    setpointy +=  yawDecode;   

    //setpoint = (0.4*((float)remote_16[0]))/((float)(2 << 16 - 1)) - 0.2;
    //setpointy = 2*3.1415 * ((float)remote_16[1])/((float)(2 << 16)) - 3.1415;
    //Serial.print(setpoint);
   // Serial.print(" ");
    //Serial.println(setpointy);
  }

/*
 client = server.available();
  if (client){  
    if (client.available()) {
      uint8_t buffer[PACKET_SIZE];
      len = client.read(buffer, PACKET_SIZE);
      const uint16_t* buff16 = (const uint16_t*)buffer;
      tiltNumber = buff16[0];
      yawNumber = buff16[1];

      float tiltDecode = ((float)tiltNumber) /(65535.0f)*(2.0f) - 1.0f;
      float yawDecode = ((float)yawNumber) /(65535.0f)*(2.0f) - 1.0f;

      tiltDecode *= 0.2;
      yawDecode *= 0.01;

      Serial.print("Received. Tilt:");
      Serial.print(tiltDecode);
      Serial.print(" Yaw:");
      Serial.println(yawDecode);

      setpoint = tiltDecode;
      setpointy +=  yawDecode;
    }
  }
  */
  
  if(Serial.available() > 0)
  {   
    Serial.setTimeout(90);
    serBuff = Serial.readString(); 

    if(serBuff.substring(0, 4) == "KILL"){
            Serial.println("Killing Motors");
            // TODO: Kill Motors
    }else if (serBuff.substring(0, 7) == "SETTILT"){
      double tiltAngle;
      char __serBuff[sizeof(serBuff)];
      serBuff.toCharArray(__serBuff, sizeof(__serBuff));    
      int result = sscanf(__serBuff, "SETTILT %lf", &tiltAngle);
      Serial.print("Setting Tilt: ");
      Serial.println(tiltAngle/100);      
      setpoint = tiltAngle/100.0;
    } else if(serBuff.substring(0, 6) == "SETYAW"){
      double yawAngle;
      char __serBuff[sizeof(serBuff)];
      serBuff.toCharArray(__serBuff, sizeof(__serBuff));    
      int result = sscanf(__serBuff, "SETYAW %lf", &yawAngle);
      Serial.print("Setting yaw: ");
      Serial.println(yawAngle/100);  
      setpointy = yawAngle/100.0;
      
    }else if(serBuff.substring(0, 6) == "SETPID") {
      double kpt, kit, kdt;
      char __serBuff[sizeof(serBuff)];
      serBuff.toCharArray(__serBuff, sizeof(__serBuff));    
      int result = sscanf(__serBuff, "SETPID %lf %lf %lf", &kpt, &kit, &kdt);
  
      pidTilt.SetTunings((double)kpt, (double)kit, (double)kdt);
  
      Serial.print("PID Gains Changed. P:");
      Serial.print(pidTilt.GetKp());
      Serial.print(" I:");
      Serial.print(pidTilt.GetKi());
      Serial.print(" D:");
      Serial.println(pidTilt.GetKd());
    }
  }
 

  

  Serial.print( wraptopi(setpointy - ypr[0]));
  Serial.print(" ");
  Serial.println(wraptopi(setpoint - ypr[1]));
  
  pidTilt.Compute();
  pidYaw.Compute();

  out0 = output - outputy;
  out1 = output + outputy;

  if(out0 > 0.0){
    digitalWrite(DR0, false);
  } else {
    digitalWrite(DR0, true);
  }

  if(out1 > 0.0){
    digitalWrite(DR1, true);
  } else {
    digitalWrite(DR1, false);
  }

  double duty_mag0 = abs(255.0/50.0*min(50, abs(out0)));
  double duty_mag1 = abs(255.0/50.0*min(50, abs(out1)));
  dutyCycle0 = (uint8_t)duty_mag0;
  dutyCycle1 = (uint8_t)duty_mag1;

  if(fabs(input) < 0.6){
    ledcWrite(pwmChannel1, dutyCycle1); 
    ledcWrite(pwmChannel0, dutyCycle0);
  } else {
    ledcWrite(pwmChannel0, 0); 
    ledcWrite(pwmChannel1, 0);
  }

  
/*
  Serial.print(ypr[0]);
  Serial.print(" ");
  Serial.print(ypr[1]);
  Serial.print(" ");
  Serial.print(outputy);
  Serial.print(" ");
  Serial.println(output);
  */

  /*if(output > 0.0){
    digitalWrite(DR1, true);
    digitalWrite(DR0, false);
    
  }
  else {
    digitalWrite(DR1, false);
    digitalWrite(DR0, true);
  }*/


  
  //double duty_mag = abs(255.0/50.0*min(50, abs(output)));
  //dutyCycle = (uint8_t)duty_mag;

  //Serial.print(" ");
  //Serial.println(dutyCycle);
  

  /*if(prevDuty != dutyCycle)
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
  }*/

//  prevDuty = dutyCycle;
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
