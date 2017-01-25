#define MPU6050_RA_PWR_MGMT_1       0x6B

#define MPU6050_RA_SMPLRT_DIV       0x19

#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C

#define MPU6050_RA_I2C_MST_CTRL     0x24

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
double temp_read;
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

    Wire.beginTransmission(mpu_address);          
    Wire.write(0x24);                             // We want to write to the MPU6050_RA_I2C_MST_CTRL register
    Wire.write(13);                               // 13 400kHz 
    Wire.endTransmission();                       

    Wire.beginTransmission(mpu_address);          
    Wire.write(0x1A);                           // MPU6050_RA_CONFIG 
    Wire.write(B00000000);                      // No DLPF, No External Sync
    // This works, and will apply a LPF but it will be on both the Gyro and Accelerometer
    // I may come back and use this but for now I will do a LPF within the firmware as part of the mpu read process
    // Note: In code the read + lpf is about 520us, when doing the lpf on the chip its about 320us
  //Wire.write(B00000001);                      // DLPF, Accel @ 184hz and Gyro @ 188hz. No External Sync
  //Wire.write(B00000010);                      // DLPF, Accel @ 94hz and Gyro @ 98hz. No External Sync
  //Wire.write(B00000110);                      // DLPF, Accel @ 5hz and Gyro @ 5hz. No External Sync
    Wire.endTransmission();                       

    Wire.beginTransmission(mpu_address);              
    Wire.write(0x1B);                            // Set MPU6050_RA_GYRO_CONFIG 
    Wire.write(B00011000);                       // full scale, no self test           
    Wire.endTransmission();                       

    Wire.beginTransmission(mpu_address);          
    Wire.write(0x1C);                            // Set MPU6050_RA_ACCEL_CONFIG 
    Wire.write(B00011000);                       // full scale, no self test       
    Wire.endTransmission();    

    system_check |= INIT_MPU_ENABLED;            
}

bool gyro_lpf = false;
void read_mpu_process() {
// gyro only reads: 520us
// gyro + accelerometer reads: 844us
// accelerometer reads: 408us

// gyro read
  Wire.beginTransmission(mpu_address);      //Start communication with the gyro
  Wire.write(0x43);                         //Start reading from register 0x43 
  Wire.endTransmission();                   //End the transmission
  Wire.requestFrom(mpu_address, 6);         //Request 6 bytes 
  
  while(Wire.available() < 6);              //Wait until the 6 bytes are received
  
  gyro_read[0] = Wire.read()<<8|Wire.read();    //Read high and low part of the gyro data
  gyro_read[1] = Wire.read()<<8|Wire.read();    //Read high and low part of the gyro data
  gyro_read[2] = Wire.read()<<8|Wire.read();    //Read high and low part of the gyro data
// gyro read

// accl read
  Wire.beginTransmission(mpu_address);          //Start communication with the gyro
  Wire.write(0x3B);                             //Start reading from register 0x3B 
  Wire.endTransmission();                       //End the transmission
  Wire.requestFrom(mpu_address, 6);             //Request 6 bytes from the gyro
  
  while(Wire.available() < 6);                  //Wait until the 6 bytes are received
  
  accl_read[0] = Wire.read()<<8|Wire.read();    //Read high and low part of the accel data
  accl_read[1] = Wire.read()<<8|Wire.read();    //Read high and low part of the accel data
  accl_read[2] = Wire.read()<<8|Wire.read();    //Read high and low part of the accel data
// accl read 
  
  gyro_read[0] = gyro_read[0] - gyro_offsets[0] ;
  gyro_read[1] = gyro_read[1] - gyro_offsets[1] ;    
  gyro_read[2] = gyro_read[2] - gyro_offsets[2] ; 
 
  if( gyro_lpf ) {
    // convert to degres/sec with a low pass filter
    gyro[0] = (gyro[0] * 0.8) + ((gyro_read[0] / 57.14286) * 0.2);
    gyro[1] = (gyro[1] * 0.8) + ((gyro_read[1] / 57.14286) * 0.2);
    gyro[2] = (gyro[2] * 0.8) + ((gyro_read[2] / 57.14286) * 0.2);
  } else {
    // unfiltered raw data (unless we turn on the DLPF within the 6050
    gyro[0] = gyro_read[0];
    gyro[1] = gyro_read[1];
    gyro[2] = gyro_read[2];
  }
}

void calibrate_gyro() {
  double temp_gyro_offsets[3] = {0,0,0};
 
  for(int i=0; i<2000; i++ ) {
    read_mpu_process();
    for(int j=0; j<3; j++ ) {
      temp_gyro_offsets[j] += gyro_read[j] ;
    }
    delay(1);
  }
  
  for(int i=0; i<3; i++ ) {
    gyro_offsets[i] = temp_gyro_offsets[i] / 2000.0;
  }  
  
  system_check |= INIT_MPU_STABLE;
  
}


