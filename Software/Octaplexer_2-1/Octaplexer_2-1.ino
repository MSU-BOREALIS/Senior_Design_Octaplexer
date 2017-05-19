#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <string.h>
#include <Servo.h>

struct CAM {
  char nm[9];
  uint8_t state;
  int low_range;
  int high_range;
  boolean enable;
};

#define PiCk  A0                      //Read or Write Data (write low)
#define PiLB  A1                      //Indicates which byte it is transmitting
#define PiHB  A2                      //Indicates which byte it is transmitting
#define Pi01  A3                      //Data Line 1
#define Pi02  2                       //Data line 2
#define Pi03  3                       //Data line 3
#define Pi04  4                       //Data line 4
//--------------
#define SEL1  5                       //Digital pin controlling SEL1
#define SEL2  6                       //Digital pin controlling SEL2
#define SEL3  7                       //Digital pin controlling SEL3
#define gate  8                       //Servo Connection MOSFET gate
#define pwm   9                       //Servo Connection PWM

#define numCams 8                     //Number of Camera's Connected
#define degreesPerCam (360/numCams)   //Calculation of degrees allocated per camera
#define declinationOffset 12.0        //Offset of Payload Zero Point (Camera 1 relative to sensor front)

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Servo tilt;
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
volatile int desiredBear;
volatile int desiredTilt;
volatile int previousTilt;
volatile bool shutter;
volatile bool disIMU;
volatile int camSel_disIMU; // value 1-8

struct CAM camera1,camera2,camera3,camera4,camera5,camera6,camera7,camera8;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  tilt.attach(pwm,1000,2000);
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
  
  pinMode(PiCk,INPUT);
  pinMode(PiHB,INPUT);
  pinMode(PiLB,INPUT);
  pinMode(Pi01,INPUT);
  pinMode(Pi02,INPUT);
  pinMode(Pi03,INPUT);
  pinMode(Pi04,INPUT);

  pinMode(pwm,OUTPUT);
  pinMode(gate,OUTPUT);
  delay(1000);
  updateTilt();
  digitalWrite(gate,LOW);
  initializeCams();
  cam1();
  
  //Initialize Globals
  absx,absy,absz = 0,0,0;
  g_sys,g_gyro,g_accel,g_mag = 0,0,0,0;
  desiredBear = 0;
  desiredTilt = 8;
  shutter = false;
  disIMU = false;
 
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
  //delay(BNO055_SAMPLERATE_DELAY_MS);
  readDesired();
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
  camera1.low_range = (float)(360 - (degreesPerCam/2));
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

void updateTilt(void){
  if(shutter){
    digitalWrite(gate,HIGH);
    //tilt.writeMicroseconds(1000);
    Serial.println("Shuttering");
    return;
  }
  int tiltangle = (1000.00/15)*desiredTilt;
  digitalWrite(gate,HIGH);
  tilt.writeMicroseconds(1000+tiltangle);
  previousTilt = tiltangle; 
}
  

void switchCam(void){
  float IMUangle = absx+declinationOffset+desiredBear;                      //360 - absx is due to the cameras numerically being in a counterclockwise direction
  while(IMUangle > 360){
    IMUangle -= 360;
  }
  if(disIMU){
    switch(camSel_disIMU){
      case 1: 
        if(state != camera1.state){
          cam1();
        }
        break;
      case 2:
        if(state != camera2.state){
          cam2();
        }
        break;
      case 3:
        if(state != camera3.state){
          cam3();
        }
        break;
      case 4: 
        if(state != camera4.state){
          cam4();
        }
        break;
      case 5: 
        if(state != camera5.state){
          cam5();
        }
        break;
      case 6: 
        if(state != camera6.state){
          cam6();
        }
        break;
      case 7: 
        if(state != camera7.state){
          cam7();
        }
        break;
      case 8:
        if(state != camera8.state){
          cam8();
        }
        break;
      default:
        if(state != camera1.state){
          cam1();
        }
        break;
    }
    return;
  }
  if((camera1.high_range >= IMUangle) && (numCams >= 1)){
    if(state != camera1.state){
      cam1();
    }
  }
  else if((camera2.high_range >= IMUangle) && (numCams >= 2)){
    if(state != camera2.state){
      cam2();
    }
  }
  else if((camera3.high_range >= IMUangle) && (numCams >= 3)){
    if(state != camera3.state){
      cam3();
    }
  }
  else if((camera4.high_range >= IMUangle) && (numCams >= 4)){
    if(state != camera4.state){
      cam4();
    }
  }
  else if((camera5.high_range >= IMUangle) && (numCams >= 5)){
    if(state != camera5.state){
      cam5();
    }
  }
  else if((camera6.high_range >= IMUangle) && (numCams >= 6)){
    if(state != camera6.state){
      cam6();
    }
  }
  else if((camera7.high_range >= IMUangle) && (numCams >= 7)){
    if(state != camera7.state){
      cam7();
    }
  }
  else if((camera8.high_range >= IMUangle) && (numCams >= 8)){
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

//Data communication between ATMEGA328P and Raspberry Pi
void readDesired(void){
  int bytesel = (digitalRead(PiHB)*10) + digitalRead(PiLB); // Decimal value that views like binary
  int data = (digitalRead(Pi01)*8) + (digitalRead(Pi02)*4) + (digitalRead(Pi03)*2) + digitalRead(Pi04);
  
  if(digitalRead(PiCk) == HIGH){
    switch(bytesel){
      case 0: //Update Camera Bearing Angle
        desiredBear = 360-((360.00/16)*data);
        break;
      case 1:
        desiredTilt = data;
        updateTilt();
        delay(8000);
        break;
      case 10:
        if(data == 0){
          disIMU = false;
        }
        else{
          disIMU = true;
          camSel_disIMU = data;
        }
        break;
      case 11:
         if(data == 0){
          shutter = false;
        }
        else{
          shutter = true;
          updateTilt();
        }
        break;
      default:
        break;
    }
  delay(900);
  digitalWrite(gate,LOW);
  Serial.print(desiredBear);Serial.print(", ");Serial.print(desiredTilt);Serial.print(", ");Serial.print(disIMU);Serial.print(", ");Serial.println(shutter);
  }
}


