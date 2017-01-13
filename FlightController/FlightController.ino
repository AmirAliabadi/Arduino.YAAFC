//#define DEBUG

#include <EEPROM.h>             //Include the EEPROM.h library so we can store information onto the EEPROM
#include <Wire.h>
#include "I2Cdev.h"

//#define DEBUG
#include "FlightController.h"
#include "PPM.h"
#include "ESC.h"
#include "MPU.h"

int throttle_input = 0;
int pitch_input = 0;
int roll_input = 0;
int yaw_input = 0;


void setup() {
#ifdef DEBUG
  Serial.begin(57600);
#endif

  system_check = INIT_CLEARED;

  EEPROM.get(0, eeprom_data);
  if(eeprom_data.id[0] != 'A' && eeprom_data.id[1] != 'A') {
    // EEPROM is not configured
    // FAIL the bootup
  }  

  Wire.begin();
  
  attachInterrupt(digitalPinToInterrupt(3), ppmRising, RISING);  // PPM input setup

  wait_for_initial_inputs();

  init_esc();
  init_mpu();

}

void loop() {

  pitch_input    = ppm_channels[1] ;  // Read ppm channel 1
  roll_input     = ppm_channels[2] ;  // Read ppm channel 2
  throttle_input = ppm_channels[3] ;  // Read ppm channel 3
  yaw_input      = ppm_channels[4] ;  // Read ppm channel 4

  // Look for ESC Arm/Disarm gestures
  if( throttle_input < 1050 && yaw_input > 1950 && roll_input < 1050 && pitch_input < 1050 ) {
    disarm_esc();
  } else if ( throttle_input < 1050 && yaw_input < 1050 && roll_input < 1050 && pitch_input > 1950 ) {
    arm_esc();
  }

  read_mpu_process();               // READ MPU   
    
#ifdef DEBUG
    Serial.print( "gyro: " )
    Serial.print( gyro.x ); 
    Serial.print( "\t" ); 
    Serial.print( gyro.y ); 
    Serial.print( "\t" ); 
    Serial.println( gyro.z );
#endif   

  if( system_check & INIT_ESC_ARMED ) {

    // DO PID CALCUATIONS
    // TODO:

    // READ BATTERY LEVEL
    // TODO:

    // DO MOTOR MIX ALGORITHM
    // TODO:
    
    va = throttle_input;
    vb = throttle_input;
    vc = throttle_input;
    vd = throttle_input;
    
  } else {
    va = MIN_ESC_SIGNAL;
    vb = MIN_ESC_SIGNAL;
    vc = MIN_ESC_SIGNAL;
    vd = MIN_ESC_SIGNAL;
        
  }

  update_motors();
}

