#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <string.h>

struct CAM {
  char nm[9];
  uint8_t state;
  int low_range;
  int high_range;
  boolean enable;
};

#define SEL1  5                       //Digital pin controlling SEL1
#define SEL2  6                       //Digital pin controlling SEL2
#define SEL3  7                       //Digital pin controlling SEL3
#define gate  8                       //Servo Connection MOSFET gate
#define pwm   9                       //Servo Connection PWM

#define numCams 8                     //Number of Camera's Connected
#define degreesPerCam (360/numCams)   //Calculation of degrees allocated per camera
#define degreesOffset 0               //Offset of Payload Zero Point (Camera 1 relative to sensor front)

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

//Prototypes
void displayCalStatus(void);
void cam1(void);
void cam2(void);
void cam3(void);
void cam4(void);
void cam5(void);
void cam6(void);
void cam7(void);
void cam8(void);
void initializeCams(void);
void switchCam(void);

//Global Variables
volatile int state;
volatile uint8_t g_sys,g_gyro,g_accel,g_mag;
volatile float absx,absy,absz;
struct CAM camera1,camera2,camera3,camera4,camera5,camera6,camera7,camera8;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  //Initialize GPIO
  pinMode(SEL1,OUTPUT);
  pinMode(SEL2,OUTPUT);
  pinMode(SEL3,OUTPUT);

  pinMode(A5,OUTPUT);
  pinMode(A4,OUTPUT);
  
  //Initialize Cameras
  initializeCams();
  cam1();
  
  //Initialize Globals
  absx,absy,absz = 0,0,0;
  g_sys,g_gyro,g_accel,g_mag = 0,0,0,0;
 
  Serial.println("Finished Initializing");
  delay(1000);
  bno.setExtCrystalUse(true);
}


void loop() {
  // put your main code here, to run repeatedly:
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  
  absx = event.orientation.x;
  absy = event.orientation.y;
  absz = event.orientation.z;
  
//  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(event.orientation.x, 4);
//  Serial.print("\tY: ");
//  Serial.print(event.orientation.y, 4);
//  Serial.print("\tZ: ");
//  Serial.print(event.orientation.z, 4);

  /* Optional: Display calibration status */
  displayCalStatus();

  /* New line for the next sample */
  //Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);

  switchCam();
}


void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  /* The data should be ignored until the system calibration is > 0 */
//  Serial.print("\t");
//  if (!system)
//  {
//    Serial.print("! ");
//  }
//
//  /* Display the individual values */
//  Serial.print("Sys:");
//  Serial.print(system, DEC);
//  Serial.print(" G:");
//  Serial.print(gyro, DEC);
//  Serial.print(" A:");
//  Serial.print(accel, DEC);
//  Serial.print(" M:");
//  Serial.print(mag, DEC);
  
  g_sys = system;
  g_gyro = gyro;
  g_accel = accel;
  g_mag = mag;
}

void initializeCams(void){
  strcpy(camera1.nm, "Camera 1");
  camera1.state = 1;
  camera1.low_range = (float)(360 - degreesPerCam);
  camera1.high_range = (float)(degreesPerCam/2);
  camera1.enable = true;
  Serial.print(camera1.nm); Serial.println(camera1.high_range);
  Serial.println(degreesPerCam);
  
  strcpy(camera2.nm, "Camera 2");
  camera2.state = 2;
  camera2.low_range = (float)(degreesPerCam/2);
  camera2.high_range = (float)(degreesPerCam/2*3);
  camera2.enable = true;

  strcpy(camera3.nm, "Camera 3");
  camera3.state = 3;
  camera3.low_range = camera2.high_range;
  camera3.high_range = (float)(degreesPerCam/2*5);
  camera3.enable = true;

  strcpy(camera4.nm, "Camera 4");
  camera4.state = 4;
  camera4.low_range = camera3.high_range;
  camera4.high_range = (float)(degreesPerCam/2*7);
  camera4.enable = true;

  strcpy(camera5.nm, "Camera 5");
  camera5.state = 5;
  camera5.low_range = camera4.high_range;
  camera5.high_range = (float)(degreesPerCam/2*9);
  camera5.enable = true;

  strcpy(camera6.nm, "Camera 6");
  camera6.state = 6;
  camera6.low_range = camera5.high_range;
  camera6.high_range = (float)(degreesPerCam/2*11);
  camera6.enable = true;

  strcpy(camera7.nm, "Camera 7");
  camera7.state = 7;
  camera7.low_range = camera6.high_range;
  camera7.high_range = (float)(degreesPerCam/2*13);
  camera7.enable = true;

  strcpy(camera8.nm, "Camera 8");
  camera8.state = 8;
  camera8.low_range = camera7.high_range;
  camera8.high_range = (float)(degreesPerCam/2*15);
  camera8.enable = true;
}

void switchCam(void){
  //Serial.print("absx: ");Serial.println(absx);
  if((camera1.high_range >= absx) && (numCams >= 1)){
    if(state != camera1.state){
      cam1();
    }
  }
  else if((camera2.high_range >= absx) && (numCams >= 2)){
    if(state != camera2.state){
      cam2();
    }
  }
  else if((camera3.high_range >= absx) && (numCams >= 3)){
    if(state != camera3.state){
      cam3();
    }
  }
  else if((camera4.high_range >= absx) && (numCams >= 4)){
    if(state != camera4.state){
      cam4();
    }
  }
  else if((camera5.high_range >= absx) && (numCams >= 5)){
    if(state != camera5.state){
      cam5();
    }
  }
  else if((camera6.high_range >= absx) && (numCams >= 6)){
    if(state != camera6.state){
      cam6();
    }
  }
  else if((camera7.high_range >= absx) && (numCams >= 7)){
    if(state != camera7.state){
      cam7();
    }
  }
  else if((camera8.high_range >= absx) && (numCams >= 8)){
    if(state != camera8.state){
      cam8();
    }
  }
  else{
    if(state != camera1.state){
      cam1();
    }
  }
}

void cam1(void){
  Serial.print("Switch to ");Serial.println(camera1.nm);
  digitalWrite(SEL1,LOW);
  digitalWrite(SEL2,LOW);
  digitalWrite(SEL3,LOW);
  state = camera1.state;
}
void cam2(void){
  Serial.println("Switch to Camera 2");
  digitalWrite(SEL1,HIGH);
  digitalWrite(SEL2,LOW);
  digitalWrite(SEL3,LOW);
  state = 2;
}
void cam3(void){
  Serial.println("Switch to Camera 3");
  digitalWrite(SEL1,LOW);
  digitalWrite(SEL2,HIGH);
  digitalWrite(SEL3,LOW);
  
  state = 3;
}
void cam4(void){
  Serial.println("Switch to Camera 4");
  digitalWrite(SEL1,HIGH);
  digitalWrite(SEL2,HIGH);
  digitalWrite(SEL3,LOW);
  state = 4;
}
void cam5(void){
  Serial.println("Switch to Camera 5");
  digitalWrite(SEL1,LOW);
  digitalWrite(SEL2,LOW);
  digitalWrite(SEL3,HIGH);
  state = 5;
}
void cam6(void){
  Serial.println("Switch to Camera 6");
  digitalWrite(SEL1,HIGH);
  digitalWrite(SEL2,LOW);
  digitalWrite(SEL3,HIGH);
  state = 6;
}
void cam7(void){
  Serial.println("Switch to Camera 7");
  digitalWrite(SEL1,LOW);
  digitalWrite(SEL2,HIGH);
  digitalWrite(SEL3,HIGH);
  state = 7;
}
void cam8(void){
  Serial.println("Switch to Camera 8");
  digitalWrite(SEL1,HIGH);
  digitalWrite(SEL2,HIGH);
  digitalWrite(SEL3,HIGH);
  state = 8;
}
