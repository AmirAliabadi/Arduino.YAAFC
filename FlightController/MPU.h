#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_9Axis_MotionApps41.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

////////////////////////////////////////////////////////////////
// MPU setup
MPU6050 mpu;

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

VectorInt16 gyro;
//
////////////////////////////////////////////////////////////////

void init_mpu()
{
    system_check &= ~(INIT_MPU_ENABLED | INIT_MPU_STABLE);
      
    mpu.initialize();

#ifdef DEBUG
    Serial.println(mpu.testConnection() ? F("#MPU6050 ok") : F("MPU6050 failed"));
    Serial.println(F("#Init DMP"));
#endif    

    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
      // Supply your own gyro offsets here, scaled for min sensitivity
/*      
      mpu.setXAccelOffset(eeprom_data.ax_offset);
      mpu.setYAccelOffset(eeprom_data.ay_offset);
      mpu.setZAccelOffset(eeprom_data.az_offset);
      mpu.setXGyroOffset(eeprom_data.gx_offset);
      mpu.setYGyroOffset(eeprom_data.gy_offset);
      mpu.setZGyroOffset(eeprom_data.gz_offset);
*/      
      
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);

///////////////////////////////////////////////////////////////////
//#define MPU6050_DLPF_BW_256         0x00
//#define MPU6050_DLPF_BW_188         0x01
//#define MPU6050_DLPF_BW_98          0x02
//#define MPU6050_DLPF_BW_42          0x03
//#define MPU6050_DLPF_BW_20          0x04
//#define MPU6050_DLPF_BW_10          0x05
//#define MPU6050_DLPF_BW_5           0x06
/*
* DLPF_CFG  | Bandwidth | Delay   | Bandwidth | Delay   | Sample Rate
*  ---------+-----------+---------+-----------+---------+-------------
* 0         | 260Hz     | 0ms     | 256Hz     | 0.98ms  | 8kHz
* 1         | 184Hz     | 2.0ms   | 188Hz     | 1.9ms   | 1kHz
* 2         | 94Hz      | 3.0ms   | 98Hz      | 2.8ms   | 1kHz
* 3         | 44Hz      | 4.9ms   | 42Hz      | 4.8ms   | 1kHz
* 4         | 21Hz      | 8.5ms   | 20Hz      | 8.3ms   | 1kHz
* 5         | 10Hz      | 13.8ms  | 10Hz      | 13.4ms  | 1kHz
* 6         | 5Hz       | 19.0ms  | 5Hz       | 18.6ms  | 1kHz
* 7         | -- Reserved -- | -- Reserved -- | Reserved  
*/
//      mpu.setDLPFMode(MPU6050_DLPF_BW_5);
     
      // enable Arduino interrupt detection
      //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      //attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
      
      mpuIntStatus = mpu.getIntStatus();

      packetSize = mpu.dmpGetFIFOPacketSize();
      
      system_check |= INIT_MPU_ENABLED;
      
    }
    else
    {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
#ifdef DEBUG      
      Serial.print(F("#DMP Init fail: "));
      Serial.println(devStatus);      
#endif      
    }
}


////////////////////////////////////////////////////////////////
// read_mpu
//
void read_mpu_process()
{
  // get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();

#ifdef DEBUG
    Serial.println(F("#Foflw"));
#endif

  } // otherwise, check for DMP data ready interrupt (this should happen frequently)
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetGyro(&gyro, fifoBuffer); // this is in degrees/s?  I'm certain deg/sec
  
  }
}
