#include <DebugMacros.h>
#include <LSM303CTypes.h>
#include <SparkFunIMU.h>
#include <SparkFunLSM303C.h>

float xmax = 0;
float xmin = 0;
float ymax = 0;
float ymin = 0;
float zmax = 0;
float zmin = 0;
float xoffset = 0;
float yoffset = 0;
float zoffset = 0;
float heading = 0;
float 
float MagX = 0;
float MagY = 0;
float MagZ = 0;

LSM303C IMU;

Servo rudderServo;

void setup() {
  Serial.begin(115200);

  RudderServo.attach(9);

  if (IMU.begin(
  ///// Interface mode options
    //MODE_SPI,
    MODE_I2C,

  ///// Magnetometer output data rate options
    //MAG_DO_0_625_Hz,
    //MAG_DO_1_25_Hz,
    //MAG_DO_2_5_Hz,
    //MAG_DO_5_Hz,
    //MAG_DO_10_Hz,
    //MAG_DO_20_hZ,
    //MAG_DO_40_Hz,
    MAG_DO_80_Hz,

  ///// Magnetic field full scale options
    //MAG_FS_4_Ga,
    //MAG_FS_8_Ga,
    //MAG_FS_12_Ga,
    MAG_FS_16_Ga,
                  
  ///// Magnetometer block data updating options
    //MAG_BDU_DISABLE,
    MAG_BDU_ENABLE,

  ///// Magnetometer X/Y axes ouput data rate
    //MAG_OMXY_LOW_POWER,
    //MAG_OMXY_MEDIUM_PERFORMANCE,
    MAG_OMXY_HIGH_PERFORMANCE,
    //MAG_OMXY_ULTRA_HIGH_PERFORMANCE,

  ///// Magnetometer Z axis ouput data rate
    //MAG_OMZ_LOW_PW,
    //MAG_OMZ_MEDIUM_PERFORMANCE,
    MAG_OMZ_HIGH_PERFORMANCE,
    //MAG_OMZ_ULTRA_HIGH_PERFORMANCE,

  ///// Magnetometer run mode
    MAG_MD_CONTINUOUS,
    //MAG_MD_SINGLE,
    //MAG_MD_POWER_DOWN_1,
    //MAG_MD_POWER_DOWN_2,

  ///// Acceleration full scale
    ACC_FS_2g,
    //ACC_FS_4g,
    //ACC_FS_8g,

  ///// Accelerometer block data updating
    //ACC_BDU_DISABLE,
    ACC_BDU_ENABLE,

  ///// Enable X, Y, and/or Z axis
    //ACC_DISABLE_ALL,
    //ACC_X_ENABLE,
    //ACC_Y_ENABLE,
    //ACC_Z_ENABLE,
    //ACC_X_ENABLE|ACC_Y_ENABLE,
    //ACC_X_ENABLE|ACC_Z_ENABLE,
    //ACC_Y_ENABLE|ACC_Z_ENABLE,
    ACC_X_ENABLE|ACC_Y_ENABLE|ACC_Z_ENABLE,

  ///// Accelerometer output data rate
    ACC_ODR_POWER_DOWN
    //ACC_ODR_10_Hz
    //ACC_ODR_50_Hz
    //ACC_ODR_100_Hz
    //ACC_ODR_200_Hz
    //ACC_ODR_400_Hz
    //ACC_ODR_800_Hz
    ) != IMU_SUCCESS)
  {
    Serial.println("Failed setup.");
    while(1);  // Stop here 
  }
}

void loop() {
  Serial.print("\nMagnetometer:\n");
  Serial.print(" X = ");
  Serial.println(IMU.readMagX(), 4);
  Serial.print(" Y = ");
  Serial.println(IMU.readMagY(), 4);
  Serial.print(" Z = ");
  Serial.println(IMU.readMagZ(), 4);
  delay(250);
}

//run at boot
void calibrateIMU() {
  //read data once to elimiate value jumps
  readIMU();

  //return the smallest value from the each axis, either the last recorded min or the curret magnatomiter reading
  xmin = min(xmin, MagX);
  ymin = min(ymin, MagY);
  zmin = min(zmin, MagZ);
  
  //return the smallest value from the each axis, either the last recorded max or the curret magnatomiter reading
  xmax = max(xmax, MagX);
  ymax = max(ymax, MagY);
  zmax = max(zmax, MagZ);
}

//run after looping { calibrateIMU(); }
void calculateCalibrationIMU() {
  
  //find the average for each min and max of each axis
  xoffset = (xmax + xmin) / 2.0;
  yoffset = (ymax + ymin) / 2.0;
  zoffset = (zmax + zmin) / 2.0;
}

//calculate rudder position
void setrudder() {
  //servo = subMagNorth + hedding;
  calculateHeading();

  rudderServo.write();
  
}

void calcuateHeading() {
  heading = (atan2(MagY, MagX) * 180) / 3.14159;
  //return heading;
}

//grab data from IMU and place it in the global scope
void readIMU() {
  MagX = IMU.readMagX();
  MagY = IMU.readMagY();
  MagZ = IMU.readMagZ();
}

