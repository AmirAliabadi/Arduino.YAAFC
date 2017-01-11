#include "FlightController.h"
#include "PPM.h"
#include "ESC.h"

#define DEBUG

int throttle_input = 0;
int pitch_input = 0;
int roll_input = 0;
int yaw_input = 0;


void setup() {
#ifdef DEBUG
  Serial.begin(57600);
#endif

  system_check = INIT_CLEARED;
  
  attachInterrupt(digitalPinToInterrupt(3), ppmRising, RISING);  // PPM input setup

  wait_for_initial_inputs();

  init_esc();
  arm_esc();  
}

void loop() {

  pitch_input    = ppm_channels[1] ;
  roll_input     = ppm_channels[2] ;
  throttle_input = ppm_channels[3] ;
  yaw_input      = ppm_channels[4] ;    

  va = throttle_input;
  vb = throttle_input;
  vc = throttle_input;
  vd = throttle_input;

  update_motors();
}

