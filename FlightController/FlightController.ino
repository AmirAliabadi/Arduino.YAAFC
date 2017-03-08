//#define DEBUG

#include <EEPROM.h>             //Include the EEPROM.h library so we can store information onto the EEPROM
#include <Wire.h>
//#include <Kalman.h> twice as slow as complementary filer

#define DEBUG
//#define UNIT_TEST_MODE

#include "FlightController.h"

// input values from receiver
// ranges from 1000 to 2000 (ms)
int throttle_setpoint = 0;
int pitch_setpoint    = 0;
int roll_setpoint     = 0;
int yaw_setpoint      = 0;

//Kalman kalmanX; // Create the Kalman instances
//Kalman kalmanY;

// using a center stick throttle
// throttle value accumulates or decays bases on throttle_input
// throttle is a value of 1000 to 2000
int throttle = MIN_ESC_SIGNAL;

double dt = 0.0;
unsigned long  timer = 0;

#include "PPM.h"
#include "MPU.h"
#include "PID.h"
#include "ESC.h"

void setup() {
#ifdef DEBUG
  Serial.begin(57600);
#endif

  system_check = INIT_CLEARED;
      
  DDRB |= B00110000;                                           //ports 12 and 13 as output.

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

  ppm_channels[0] = 0;  // sync channel
  ppm_channels[1] = 1500;
  ppm_channels[2] = 1500;
  ppm_channels[3] = 1500;
  ppm_channels[4] = 1500;
  ppm_channels[5] = 1500;
  ppm_channels[6] = 1500;
  ppm_channels[7] = 1500;
  ppm_channels[8] = 1500;

#ifndef UNIT_TEST_MODE
  attachInterrupt(digitalPinToInterrupt(3), ppmRising, RISING);  // PPM input setup
#ifdef DEBUG
  delay(50);
  if( !ppm_sync ) Serial.println("Power on Transmitter");
#endif
  while( !ppm_sync ) {
    delay(10);        // wait for ppm sync
  }
  wait_for_initial_inputs();  // wait for all stick to be neutral
#endif  

  init_mpu();
  calibrate();
  init_esc();
  init_pid();

#ifdef UNIT_TEST_MODE
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A); 
#endif

  timer = micros();

//  kalmanX.setAngle(0);
//  kalmanY.setAngle(0);

#ifdef DEBUG
    Serial.println( "init_comleted" );
#endif
}

unsigned int guesture_count = 0;
float throttle_input_gain = 0.0;
float f_throttle = MIN_ESC_SIGNAL;

#ifdef DEBUG
int foo=4;
#endif

boolean calibartion_mode = false; // no pids, throttle max is 2000us
void loop() {

  dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  roll_setpoint       = ppm_channels[1] ;  // Read ppm channel 1
  pitch_setpoint      = ppm_channels[2] ;  // Read ppm channel 2
  throttle_setpoint   = ppm_channels[3] ;  // Read ppm channel 3
  yaw_setpoint        = ppm_channels[4] ;  // Read ppm channel 4

  // 20us of deadband
  if( pitch_setpoint    >= 1490 && pitch_setpoint     <= 1510 ) pitch_setpoint    = 1500;
  if( roll_setpoint     >= 1490 && roll_setpoint      <= 1510 ) roll_setpoint     = 1500;  
  if( throttle_setpoint >= 1490 && throttle_setpoint  <= 1510 ) throttle_setpoint = 1500;  
  if( yaw_setpoint      >= 1490 && yaw_setpoint       <= 1510 ) yaw_setpoint      = 1500;  

  ////////////////////////////////////////////////////////////////////
  // Simple Arm/Disarm.  Hold throttle at lowest position for 500ms
  if( throttle <= MIN_ESC_CUTOFF ) {
    // Look for ESC Arm/Disarm gestures
    if( throttle_setpoint < 1050 ) {
        guesture_count ++;
    } else {
      guesture_count = 0;
    }

    if( guesture_count >= 500 ) {
      if( system_check & INIT_ESC_ARMED ) {
        disarm_esc();
      } else {
        throttle = 0;
        pid_reset();
        gyro_pitch = 0; 
        gyro_roll = 0; 
        gyro_yaw = 0;
        arm_esc();      
      }    
      guesture_count = 0;
    }
  }
  // Simple Arm/Disarm. 
  ////////////////////////////////////////////////////////////////////

  // safety/testing
  // only yaw when throttle is nuteral
  if( throttle_setpoint != 1500 ) {
    yaw_setpoint = 1500;
  }  
  // safety/testing

  // adjust so 1500 = Zero input
  throttle_setpoint = (throttle_setpoint - 1500) ;
  pitch_setpoint    = (pitch_setpoint    - 1500) * -1.0; // inverted signal from TX
  roll_setpoint     = (roll_setpoint     - 1500) ;
  yaw_setpoint      = (yaw_setpoint      - 1500) ;   

  //digitalWrite(12,HIGH);

  // read_mpu_process();      // 784us : gyro+accel raw reads with LPF : moved the MPU read process to the ESC PWM 1000us idle time
  mpu_conversion_process();   // 544us : convert to degress/sec and calculate angles using complinetary filter.
  
  digitalWrite(12,LOW);

  throttle_input_gain = throttle_setpoint / 600.0;

  if( calibartion_mode ) {
    
    throttle = (int)( f_throttle += throttle_input_gain );

    if( f_throttle > MAX_ESC_SIGNAL ) f_throttle = MAX_ESC_SIGNAL;
    if( f_throttle < MIN_ESC_CUTOFF ) f_throttle = MIN_ESC_CUTOFF;    

    if( throttle > MAX_ESC_SIGNAL ) throttle = MAX_ESC_SIGNAL;
    if( throttle < MIN_ESC_CUTOFF ) throttle = MIN_ESC_CUTOFF;    

    va = throttle ; // front right - CCW
    vb = throttle ; // front left  -  CW
    vc = throttle ; // back left   - CCW
    vd = throttle ; // back right  -  CW   
    
  } else  if( system_check & INIT_ESC_ARMED ) {

    throttle = (int)( f_throttle += throttle_input_gain );

    if( f_throttle > MAX_ESC_SIGNAL ) f_throttle = MAX_ESC_SIGNAL;
    if( f_throttle < MIN_ESC_CUTOFF ) f_throttle = MIN_ESC_CUTOFF;    

    if( throttle > MAX_ESC_SIGNAL ) throttle = MAX_ESC_SIGNAL;
    if( throttle < MIN_ESC_CUTOFF ) throttle = MIN_ESC_CUTOFF;

    // DO PID CALCUATIONS
    do_pid_compute();   // 200us

    // READ BATTERY LEVEL
    // TODO:

    if (throttle > 1800) throttle = 1800;     // leave room for the PID controllers

    // DO MOTOR MIX ALGORITHM : X Setup
    va = throttle - pitch_pid_rate_out + roll_pid_rate_out - yaw_pid_rate_out; // front right - CCW
    vb = throttle + pitch_pid_rate_out + roll_pid_rate_out + yaw_pid_rate_out; // front left  -  CW
    vc = throttle + pitch_pid_rate_out - roll_pid_rate_out - yaw_pid_rate_out; // back left   - CCW
    vd = throttle - pitch_pid_rate_out - roll_pid_rate_out + yaw_pid_rate_out; // back right  -  CW

#ifdef DEBUG
    // Serial.print( ppm_channels[foo] );
    // Serial.print("\t");

    // pitch test
    // Serial.print( va ); Serial.print( "\t" ); Serial.println( vd );    // stick pitch input - nose up, should increase, gryo up should decrease
    // Serial.print( vb ); Serial.print( "\t" ); Serial.println( vc );    // stick pitch input - nose up, should decrease, gryo up should increase

    // roll test
    // Serial.print( va ); Serial.print( "\t" ); Serial.println( vb );  // stick roll input - right down, should decrease, gyro right should increase
    // Serial.print( vc ); Serial.print( "\t" ); Serial.println( vd );  // stick roll input - right down, should increase, gyro right should decrease

    // yaw test
    // Serial.print( va ); Serial.print( "\t" ); Serial.println( vc );     // stick yaw right - increase  , hard left increase
    // Serial.print( vb ); Serial.print( "\t" ); Serial.println( vd );     // stick yaw left - decrease  
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

#ifdef DEBUG
  // Serial.println( accl_read[2] );

  //Serial.print( pitch_angle ); Serial.print( "\t" ); Serial.println( roll_angle  );
  //Serial.print( pitch_angle ); Serial.print( "\t" ); Serial.println( gyro_pitch  );

/*
  Serial.print( gyro_pitch - ((pitch_angle - pitch_setpoint/15.0) * .5 ) );
  Serial.print( "\t" ); 
  Serial.println( gyro_roll - ((roll_angle - roll_setpoint/15.0) * .5 ) );
*/  

/*
  Serial.print( va ); Serial.print( "\t" ); Serial.print( vb );     
  //Serial.println("");
  Serial.print( "\t" ); 
  Serial.print( vc ); Serial.print( "\t" ); Serial.println( vd ); 
*/
  
#endif
  
  update_motors();

}

#ifdef UNIT_TEST_MODE
// unit test harness
volatile int aa_dir = 10;
volatile unsigned long my_counter = 0;
// Interrupt is called once a millisecond
SIGNAL(TIMER0_COMPA_vect) 
{
  my_counter ++ ;
  if( my_counter == 10 ) {
        
    if( system_check & INIT_PID_ON ) {
  
        if( system_check & INIT_ESC_ARMED ) {

          if( throttle < 1400 ) {
            ppm_channels[3] = 1600;
          } else {
            ppm_channels[3] = 1500;


            if( ppm_channels[foo] >= 1800 || ppm_channels[foo] <= 1200 ) {
              aa_dir = aa_dir * -1;
            }

            ppm_channels[foo] += aa_dir ;
          }
          
        } else {
          ppm_channels[3] = 1000;
          
        }
    
      //ppm_channels[1] ;  // roll
      //ppm_channels[2] ;  // pitch
      //ppm_channels[3] ;  // throttle
      //ppm_channels[4] ;  // yaw

      }

      my_counter = 0;      
  }
}
#endif
