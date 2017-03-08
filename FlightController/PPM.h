
#define NUMBER_OF_PPM_CHANNELS 8

// INT range : -32,768 to 32,767 

volatile boolean        ppm_sync = false;
volatile unsigned long  last_ppm_clock = 9999;
volatile unsigned long  current_ppm_clock = 0;
volatile unsigned long  ppm_dt = 0;
volatile unsigned short ppm_current_channel = 99;
volatile unsigned int   ppm_channels[9] = {0,0,0,0,0,0,0,0,0}; // at most 10 channels (sync chaneel + 8 = 9)

volatile unsigned short sync_loss_counter = 0;
void ppmRising() {
  current_ppm_clock = micros();
  ppm_dt = current_ppm_clock - last_ppm_clock;

  if( ppm_sync == false ) {
    if( ppm_dt >= 8500 ) {
      if( sync_loss_counter < 10 ) {
        sync_loss_counter ++;
      }
    } else if( ppm_dt >= 3500 ) {
      ppm_current_channel = 0;
      ppm_channels[ppm_current_channel] = ppm_dt;
      if( sync_loss_counter > 0 ) sync_loss_counter --;
      else ppm_sync = true;
    }
  }
  else {
    ppm_current_channel++;
    if( ppm_current_channel > NUMBER_OF_PPM_CHANNELS ) {
      ppm_sync = false;
    }
    else {
      if( ppm_dt > 900 and ppm_dt < 2200 ) ppm_channels[ppm_current_channel] = ppm_dt; 
      else ppm_sync = false;
    }
  }

  last_ppm_clock = current_ppm_clock;   
}

////////////////////////////////////////////////////////////
// wait for all inputs to go to center, 
// then force throttle to zero and then back to center
void wait_for_initial_inputs() {

#ifdef DEBUG
  Serial.println( "center all sticks" );
#endif  
  byte b = B00000000;
  while( b != B00001111 ) {
    if( ppm_channels[1] > 1490 && ppm_channels[1]  < 1510 ) b = b | B00000001;
    if( ppm_channels[2] > 1490 && ppm_channels[2]  < 1510 ) b = b | B00000010;    
    if( ppm_channels[3] > 1490 && ppm_channels[3]  < 1510 ) b = b | B00000100;
    if( ppm_channels[4] > 1490 && ppm_channels[4]  < 1510 ) b = b | B00001000;
  }

  // wait for throttle to go to zero
#ifdef DEBUG
  Serial.println( "throttle to zero" );
#endif    
  while(1) {
    if( ppm_channels[3] < 1050 ) break;
  }

  // wait for throttle to go to zero position
#ifdef DEBUG
  Serial.println( "throttle to center" );
#endif    
  while(1) {
    if( ppm_channels[3] > 1490 && ppm_channels[3]  < 1510 ) break;
  }  
  
}

