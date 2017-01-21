//#define DEBUG

#include <EEPROM.h>             //Include the EEPROM.h library so we can store information onto the EEPROM
#include <Wire.h>

#define DEBUG
#include "FlightController.h"
#include "PPM.h"
#include "ESC.h"
#include "MPU.h"

// input values from receiver
// ranges from 1000 to 2000 (ms)
int throttle_input = 0;
int pitch_input = 0;
int roll_input = 0;
int yaw_input = 0;

// using a center stick throttle
// throttle value accumulates or decays bases on throttle_input
int throttle = MIN_ESC_SIGNAL;

#include "PID.h"

void setup() {
#ifdef DEBUG
  Serial.begin(57600);
#endif

  DDRB |= B00110000;                                           //ports 12 and 13 as output.

  system_check = INIT_CLEARED;

/*
  EEPROM.get(0, eeprom_data);
  if(eeprom_data.id[0] != 'A' && eeprom_data.id[1] != 'A') {
#ifdef DEBUG
  Serial.println("Please run Calibration");
#endif
    while(1) {
      digitalWrite(13, !digitalRead(13));
      delay(10);
      digitalWrite(13, !digitalRead(13));
      delay(100);      
    }
  } 
*/  

  Wire.begin();
  Wire.setClock(400000L);   // i2c at 400k Hz

  attachInterrupt(digitalPinToInterrupt(3), ppmRising, RISING);  // PPM input setup

  #ifndef DEBUG
    wait_for_initial_inputs();
  #endif

  init_mpu();
  calibrate_gyro();
  init_esc();
  init_pid();

  gyro_lpf = true;
}

unsigned int throttle_tick_count = 0;
void loop() {

  pitch_input    = ppm_channels[1] ;  // Read ppm channel 1
  roll_input     = ppm_channels[2] ;  // Read ppm channel 2
  throttle_input = ppm_channels[3] ;  // Read ppm channel 3
  yaw_input      = ppm_channels[4] ;  // Read ppm channel 4

  // Look for ESC Arm/Disarm gestures
  if( throttle_input < 1050 && yaw_input > 1950 && roll_input < 1050 && pitch_input < 1050 ) {
    disarm_esc();
  } else if ( throttle_input < 1050 && yaw_input < 1050 && roll_input < 1050 && pitch_input > 1950 ) {
    throttle = 0;
    pid_reset();
    gyro[0] = 0; 
    gyro[1] = 0; 
    gyro[2] = 0;
    arm_esc();
  }

  // adjust so 1500 = Zero input
  pitch_input = 1500 - pitch_input ;
  roll_input  = 1500 - roll_input ;
  yaw_input   = 1500 - yaw_input ;    

  digitalWrite(12, HIGH);           // 250Hz , this is at about 100 Hz with the i2cdevlib
  read_mpu_process();               // READ MPU,  300uS ,  with the i2cdev DMP mode it was 2ms  
                                    // 500us read with lpf and offsets
  digitalWrite(12, LOW);

  if( system_check & INIT_ESC_ARMED ) {

    // slow down the throttle accum/decay behavior by only doing it every 10ms
    throttle_tick_count ++;

    // increase/decrease throttle based on throttle_input
         if( throttle_input < 1100 && throttle >= 1400 && throttle_tick_count > 2 ) { throttle -=  10; throttle_tick_count=0; } // AA fast decrease in throttle
    else if( throttle_input < 1200 && throttle >= 1010 && throttle_tick_count > 2 ) { throttle -=   5; throttle_tick_count=0; } // AA medium decrease in throttle
    else if( throttle_input < 1400 && throttle >= 1001 && throttle_tick_count > 2 ) { throttle -=   1; throttle_tick_count=0; } // AA slow decrease in throttle   
    else if( throttle_input > 1600 && throttle <= 1999 && throttle_tick_count > 2 ) { throttle +=   1; throttle_tick_count=0; } // AA slow increase in throttle
    else if( throttle_input > 1800 && throttle <= 1990 && throttle_tick_count > 2 ) { throttle +=  10; throttle_tick_count=0; } // AA medium increase in throttle   
    else if( throttle_input < 1010 ) { throttle = MIN_ESC_CUTOFF;  }   

#ifdef DEBUG
    // Serial.println( throttle );
#endif    

    // limit throttle between MIN_ESC_CUTOFF (keep motors running) to MAX_ESC_SIGNAL (typically 2000ms)
    if( throttle > MAX_ESC_SIGNAL ) throttle = MAX_ESC_SIGNAL;
    if( throttle < MIN_ESC_CUTOFF ) throttle = MIN_ESC_CUTOFF;

    // DO PID CALCUATIONS
    // We have the gyro data and the stick inputs
    do_pid_compute();   // 200us

    // READ BATTERY LEVEL
    // TODO:

    // DO MOTOR MIX ALGORITHM
    // This is for an X setup
    va = throttle - pitch_pid_rate_out + roll_pid_rate_out - yaw_pid_rate_out; //Calculate the pulse for esc a (front-right - CCW)
    vb = throttle + pitch_pid_rate_out + roll_pid_rate_out + yaw_pid_rate_out; //Calculate the pulse for esc b (rear-right  -  CW)
    vc = throttle + pitch_pid_rate_out - roll_pid_rate_out - yaw_pid_rate_out; //Calculate the pulse for esc c (rear-left   - CCW)
    vd = throttle - pitch_pid_rate_out - roll_pid_rate_out + yaw_pid_rate_out; //Calculate the pulse for esc d (front-left  -  CW)

#ifdef DEBUG
    Serial.print( throttle ); Serial.print( "\t" ); Serial.print( va ); Serial.print( "\t" ); Serial.print( vb ); Serial.print( "\t" ); Serial.print( vc ); Serial.print( "\t" ); Serial.println( vd );
    //Serial.print( throttle ); Serial.print( "\t" ); Serial.print( pitch_pid_rate_out ); Serial.print( "\t" ); Serial.print( roll_pid_rate_out ); Serial.print( "\t" ); Serial.print( yaw_pid_rate_out ); Serial.print( "\t" ); Serial.println( vb );
#endif

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

