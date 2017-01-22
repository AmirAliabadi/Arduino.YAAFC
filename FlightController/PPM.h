
#define NUMBER_OF_PPM_CHANNELS 4

// INT range : -32,768 to 32,767 

volatile boolean ppm_sync = false;
volatile unsigned long last_ppm_clock = 99999;
volatile unsigned long current_ppm_clock = 0;
volatile unsigned int ppm_dt = 0;
volatile unsigned short ppm_current_channel = 99;
volatile unsigned int ppm_channels[11] = {0,0,0,0,0,0,0,0,0,0,0}; // at most 10 channels (sync chaneel + 10 = 11)

void ppmRising() {
  current_ppm_clock = micros();
  ppm_dt = current_ppm_clock - last_ppm_clock;
  if( ppm_dt >= 3500 ) {
    ppm_sync = true;
    ppm_current_channel = 0;
    ppm_channels[ppm_current_channel] = ppm_dt;         
  }
  else {
    if( ppm_sync ) {
      ppm_current_channel++;
      if( ppm_current_channel > NUMBER_OF_PPM_CHANNELS ) ppm_sync = false;
      else ppm_channels[ppm_current_channel] = ppm_dt; 
    }
  }
  last_ppm_clock = current_ppm_clock;   
}

void wait_for_initial_inputs() {
  byte b = B00000000;
  while( b != B00001111 ) {
    if( ppm_channels[1] > 1490 && ppm_channels[1]  < 1510 ) b = b | B00000001;
    if( ppm_channels[2] > 1490 && ppm_channels[2]  < 1510 ) b = b | B00000010;    
    if( ppm_channels[3] > 1490 && ppm_channels[3]  < 1510 ) b = b | B00000100;
    if( ppm_channels[4] > 1490 && ppm_channels[4]  < 1510 ) b = b | B00001000;
  }

  // wait for throttle to go to zero
  while(1) {
    if( ppm_channels[3] < 1050 ) break;
  }

  // wait for throttle to go to zero position
  while(1) {
    if( ppm_channels[3] > 1490 && ppm_channels[3]  < 1510 ) break;
  }  
}

