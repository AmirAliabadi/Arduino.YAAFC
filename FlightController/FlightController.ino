//#define DEBUG

#include <EEPROM.h>             //Include the EEPROM.h library so we can store information onto the EEPROM
#include <Wire.h>
#include "I2Cdev.h"

//#define DEBUG
#include "FlightController.h"
#include "PPM.h"
#include "ESC.h"
#include "MPU.h"
#include "PID.h"

int throttle_input = 0;
int pitch_input = 0;
int roll_input = 0;
int yaw_input = 0;

int throttle = MIN_ESC_SIGNAL;


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
  init_pid();

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

         if( throttle_input < 1100  && throttle >= 1400 ) { throttle -=  10;  } // AA fast decrease in throttle
    else if( throttle_input < 1200  && throttle >= 1010 ) { throttle -=   5;  } // AA medium decrease in throttle
    else if( throttle_input < 1400  && throttle >= 1001 ) { throttle -=   1;  } // AA slow decrease in throttle   
    else if( throttle_input > 1600  && throttle <= 1999 ) { throttle +=   1;  } // AA slow increase in throttle
    else if( throttle_input > 1800  && throttle <= 1990 ) { throttle +=  10;  } // AA medium increase in throttle   
    else if( throttle_input < 1010                      ) { throttle = MIN_ESC_CUTOFF;  }   

    if( throttle > MAX_ESC_SIGNAL ) throttle = MAX_ESC_SIGNAL;
    if( throttle < MIN_ESC_CUTOFF ) throttle = MIN_ESC_CUTOFF;

    // DO PID CALCUATIONS
    // We have the gyro data and the stick inputs
    //pid_yaw_rate   = yaw_pid.calculate( yaw_input, gyro.z );
    //pid_pitch_rate = pitch_pid.calculate( pitch_input, gyro.x ); 
    //pid_roll_rate  = roll_pid.calculate( roll_input, gyro.y ); 

    // READ BATTERY LEVEL
    // TODO:

    // DO MOTOR MIX ALGORITHM
    // This is for an X setup
    va = throttle - pid_pitch_rate + pid_roll_rate - pid_yaw_rate; //Calculate the pulse for esc a (front-right - CCW)
    vb = throttle + pid_pitch_rate + pid_roll_rate + pid_yaw_rate; //Calculate the pulse for esc b (rear-right  -  CW)
    vc = throttle + pid_pitch_rate - pid_roll_rate - pid_yaw_rate; //Calculate the pulse for esc c (rear-left   - CCW)
    vd = throttle - pid_pitch_rate - pid_roll_rate + pid_yaw_rate; //Calculate the pulse for esc d (front-left  -  CW)

    if( va < MIN_ESC_CUTOFF ) va = MIN_ESC_CUTOFF;
    if( vb < MIN_ESC_CUTOFF ) vb = MIN_ESC_CUTOFF;
    if( vc < MIN_ESC_CUTOFF ) vc = MIN_ESC_CUTOFF;
    if( vd < MIN_ESC_CUTOFF ) vd = MIN_ESC_CUTOFF;

    if( va > MAX_ESC_SIGNAL ) va = MAX_ESC_SIGNAL;
    if( vb > MAX_ESC_SIGNAL ) vb = MAX_ESC_SIGNAL;
    if( vc > MAX_ESC_SIGNAL ) vc = MAX_ESC_SIGNAL;
    if( vd > MAX_ESC_SIGNAL ) vd = MAX_ESC_SIGNAL;
    
  } else {
    
    pid_reset();
    
    va = MIN_ESC_SIGNAL;
    vb = MIN_ESC_SIGNAL;
    vc = MIN_ESC_SIGNAL;
    vd = MIN_ESC_SIGNAL;
        
  }

  update_motors();
}

