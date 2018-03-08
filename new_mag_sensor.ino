#include <Wire.h>
#define Addr 0x1E               // 7-bit address of HMC5883 comHMC5883L_Arduinopass

int x, y, z;
int xmin = 0;
int ymin = 0;
int zmin = 0;
int xmax = 0;
int ymax = 0;
int zmax = 0;
int xoffset = 0;
int yoffset = 0;
int zoffset = 0;

void setup() {

InitMagandCalibrate();

}

void loop() {
int CurrentHeading;
  CurrentHeading = GetMagHeading ();
   Serial.println (CurrentHeading);

}

void InitMagandCalibrate (){  
  // put your setup code here, to run once:
  Serial.begin(9600);
    delay(100);                   // Power up delay
  Wire.begin();
  
  // Set operating mode to continuous
  Wire.beginTransmission(Addr); 
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();


  Serial.println("calibrating: Start ");
    

  for (int i = 0; i < 1000; i++) {
    calibrate();
    
  }

  finishcalibration();
  Serial.println("calibrating: End ");

}  


int GetMagHeading (){
  readMag();
  return calcRot();

  
}



void readMag() {
  // Initiate communications with compass
  Wire.beginTransmission(Addr);
  Wire.write(byte(0x03));       // Send request to X MSB register
  Wire.endTransmission();

  Wire.requestFrom(Addr, 6);    // Request 6 bytes; 2 bytes per axis
  if(Wire.available() <=6) {    // If 6 bytes available
    x = (Wire.read() << 8 | Wire.read()) + xoffset;
    z = (Wire.read() << 8 | Wire.read()) + yoffset;
    y = (Wire.read() << 8 | Wire.read()) + zoffset;
  }
 
}


void calibrate() {
  readMag();
  xmin = min(xmin, x);
  ymin = min(ymin, y);
  zmin = min(zmin, z);

  xmax = min(xmax, x);
  ymax = min(xmax, y);
  zmax = min(xmax, z);
}

void finishcalibration() {
  xoffset = (xmin + xmax) / 2;
  yoffset = (ymin + ymax) / 2;
  zoffset = (xmin + xmax) / 2;
}

int calcRot() {
//  int heading = (atan2(y, x) * 180) / 3.14159;
 float heading = atan2(y, x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.20;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/PI; 
  int headingDegInt = headingDegrees;


  return headingDegInt;
}

