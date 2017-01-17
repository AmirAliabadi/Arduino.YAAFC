EEPROMData data_valid ;

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// orientation/motion vars
Quaternion q;                 // [w, x, y, z]         quaternion container
VectorInt16 aa;               // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;           // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;          // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;          // [x, y, z]            gravity vector
VectorInt16 gyro;


#define YAW 0
#define ROLL 1
#define PITCH 2
float ypr[3]      = {0.0f, 0.0f, 0.0f};


void init_mpu()
{
  accelgyro.initialize();
  
  Serial.println(F("#Test device"));
  if( !accelgyro.testConnection() )
  {
    Serial.println( F("MPU6050 failed") );
    while(1);
  } else {
    Serial.println( F("MPU6050 success") );
  }
  
  delay(500);
  
  devStatus = accelgyro.dmpInitialize();
  
  if (devStatus == 0)
  {
    EEPROM.get(0, data_valid);
    
    accelgyro.setXAccelOffset(data_valid.ax_offset);
    accelgyro.setYAccelOffset(data_valid.ay_offset);
    accelgyro.setZAccelOffset(data_valid.az_offset);
    accelgyro.setXGyroOffset(data_valid.gx_offset);
    accelgyro.setYGyroOffset(data_valid.gy_offset);
    accelgyro.setZGyroOffset(data_valid.gz_offset);
  
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_98);
    accelgyro.setDMPEnabled(true);
  
    mpuIntStatus = accelgyro.getIntStatus();    
    packetSize = accelgyro.dmpGetFIFOPacketSize();

  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("#DMP Init fail: "));
    Serial.println(devStatus);
    while(1);
  }  
}

int last_log = 0;
void read_mpu()
{
  mpuIntStatus = accelgyro.getIntStatus();
  fifoCount = accelgyro.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    accelgyro.resetFIFO();
    Serial.println("fifo overflow");
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();
  
    // read a packet from FIFO
    accelgyro.getFIFOBytes(fifoBuffer, packetSize);
  
    fifoCount -= packetSize;
  
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);

    accelgyro.dmpGetGyro(&gyro, fifoBuffer); // this is in degrees/s?  I'm certain deg/sec    


    ///////////////////////////////////////////////////////////////    
    // this give you the angle the chip is sat
    accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);  // this is in radians    
    
    #define A_180_DIV_PI 57.2957795131
    ypr[YAW]    = ((ypr[YAW] * A_180_DIV_PI) ) ; // YAW
    ypr[PITCH]  = ((ypr[PITCH] * A_180_DIV_PI) ) ; // my ROLL
    ypr[ROLL]   = ((ypr[ROLL] * A_180_DIV_PI) ) ; // my PITCH    
    ///////////////////////////////////////////////////////////////

if( millis() - last_log  > 500 ) {
    last_log = millis();
    Serial.print(gyro.x); Serial.print("\t");
    Serial.print(gyro.y); Serial.print("\t");
    Serial.print(gyro.z); Serial.print("\t");
    Serial.print(ypr[YAW]); Serial.print("\t");
    Serial.print(ypr[PITCH]); Serial.print("\t");
    Serial.print(ypr[ROLL]); Serial.println("\t");
}

  } else {
    //Serial.println(mpuIntStatus);
  }
}

