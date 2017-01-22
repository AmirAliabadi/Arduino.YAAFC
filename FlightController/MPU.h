#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40

#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42

#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48


double gyro[3] = {0,0,0};
double accl[3] = {0,0,0};
double gyro_read[3] = {0,0,0};
double accl_read[3] = {0,0,0};
double gyro_offsets[3] = {0,0,0};

/* MPU 6050 Orientation:

front left          front right  (CCW)
(CW)
           +------+
           |( )  .|--- Int
           |     .|
           |     .|--- sda
           |     .|--- scl
           |( )  .|--- gnd
           +------+--- vcc

back left          back right (CW)
(CCW)

Roll Right  = - Gryo
Roll Left   = + Gyro
Pitch Up    = - Gyro
Pitch Down  = + Gyro
Yaw Right   = + Gyro
Yaw Left    = - Gyro
*/

byte PIT = 0;
byte ROL = 1;
byte YAW = 2;

int mpu_address = 0x68;
void init_mpu() {
    system_check &= ~(INIT_MPU_ENABLED | INIT_MPU_STABLE);
  
    Wire.beginTransmission(mpu_address);         //Start communication with the address found during search.
    Wire.write(0x6B);                             //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                             //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                       //End the transmission with the gyro. 

    delay(10);

    Wire.beginTransmission(mpu_address);                        //Start communication with the address found during search.
    Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (0x1B)
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                      //End the transmission with the gyro  

    Wire.beginTransmission(mpu_address);                        //Start communication with the address found during search
    Wire.write(0x1B);                                            //Start reading @ register 0x1B
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(mpu_address, 1);                           //Request 1 bytes from the gyro
    
    while(Wire.available() < 1);                                 //Wait until the 6 bytes are received

    if(Wire.read() != 0x08){                                     //Check if the value is 0x08
      digitalWrite(13,HIGH);                                     //Turn on the warning led
      while(1) {
        delay(10);                                                //Stay in this loop forever
      }
    } 
    
    Wire.beginTransmission(mpu_address);                        //Start communication with the address found during search
    Wire.write(0x1A);                                            //We want to write to the CONFIG register (0x1A)
    Wire.write(0x03);                                            //Set the register bits as 00001000 (500dps full scale)
                                                                 //Set both the gyro and the accelerometer at full scale
    Wire.endTransmission();                                      //End the transmission with the gyro

    system_check |= INIT_MPU_ENABLED;            
}

bool gyro_lpf = false;
void read_mpu_process() {
// gyro read
  Wire.beginTransmission(mpu_address);      //Start communication with the gyro
  Wire.write(0x43);                         //Start reading @ register 43h and auto increment with every read
  Wire.endTransmission();                   //End the transmission
  Wire.requestFrom(mpu_address, 6);         //Request 6 bytes 
  
  while(Wire.available() < 6);              //Wait until the 6 bytes are received
  
  gyro_read[0] = Wire.read()<<8|Wire.read();    //Read high and low part of the gyro data
  gyro_read[1] = Wire.read()<<8|Wire.read();    //Read high and low part of the gyro data
  gyro_read[2] = Wire.read()<<8|Wire.read();    //Read high and low part of the gyro data
// gyro read

// accl read
  Wire.beginTransmission(mpu_address);          //Start communication with the gyro
  Wire.write(0x3B);                             //Start reading @ register 43h and auto increment with every read
  Wire.endTransmission();                       //End the transmission
  Wire.requestFrom(mpu_address, 6);             //Request 6 bytes from the gyro
  
  while(Wire.available() < 6);                  //Wait until the 6 bytes are received
  
  accl_read[0] = Wire.read()<<8|Wire.read();    //Read high and low part of the gyro data
  accl_read[1] = Wire.read()<<8|Wire.read();    //Read high and low part of the gyro data
  accl_read[2] = Wire.read()<<8|Wire.read();    //Read high and low part of the gyro data
// accl read    

  if( system_check & INIT_MPU_STABLE ) {
    gyro_read[0] = gyro_read[0] -  gyro_offsets[0] ;
    gyro_read[1] = gyro_read[1] -  gyro_offsets[1] ;    
    gyro_read[2] = gyro_read[2] -  gyro_offsets[2] ; 
  }

  if( gyro_lpf ) {
    // convert to degres/sec with a low pass filter
    gyro[0] = (gyro[0] * 0.8) + ((gyro_read[0] / 57.14286) * 0.2);
    gyro[1] = (gyro[1] * 0.8) + ((gyro_read[1] / 57.14286) * 0.2);
    gyro[2] = (gyro[2] * 0.8) + ((gyro_read[2] / 57.14286) * 0.2);
  }

  // Serial.print(accl_read[0]); Serial.print("\t");Serial.print(accl_read[1]); Serial.print("\t");Serial.println(accl_read[2]); 
}

void calibrate_gyro() {
  
  for(int i=0; i<2000; i++ ) {
    read_mpu_process();
    for(int j=0; j<3; j++ ) {
      gyro_offsets[j] += gyro[j] ;
    }
    delay(1);
  }
  
  for(int i=0; i<3; i++ ) {
    gyro_offsets[i] = gyro_offsets[i] / 2000;
  }  
  
  system_check |= INIT_MPU_STABLE;
}


