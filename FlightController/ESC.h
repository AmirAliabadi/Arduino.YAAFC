#define PWM_FERQUENCY 4000  // 250hz, very stable
//#define PWM_FERQUENCY 3400 // 290hz, stable

unsigned long last_pwm_pulse = 0;
unsigned long esc_pwm_timmer = 0;

unsigned long timer_channel_a = 0;
unsigned long timer_channel_b = 0; 
unsigned long timer_channel_c = 0;
unsigned long timer_channel_d = 0; 

//                 abcd
byte motors = B00000000;

long va,vb,vc,vd ;  // motors a,b,c and d outputs


void init_esc()
{
//  DDRD |= B00001000;                                           //Configure digital poort 3 as output
//  DDRD |= B00100000;                                           //Configure digital poort 5 as output
    DDRD |= B01000000;                                           //Configure digital poort 6 as output
    DDRB |= B00001110;                                           //Configure digital poort 9, 10, 11 as output.

  system_check |= INIT_ESC_ATTACHED;
}

void arm_esc()
{
  system_check |= INIT_ESC_ARMED;
}

void disarm_esc()
{
  system_check &= ~(INIT_ESC_ARMED);
}

void update_motors()
{
  // wait for next rising pulse time
  while( (micros() - last_pwm_pulse) < PWM_FERQUENCY );
  last_pwm_pulse= micros(); 

  // all PWM pins HIGH
//PORTD |= B00001000; // Set digital port 3 high
  PORTD |= B01000000; // Set digital port 6 high
  PORTB |= B00001110; // Set digital port 9,10,11 high

  // compute when each PWM pin should go low
  timer_channel_a = last_pwm_pulse + va;
  timer_channel_b = last_pwm_pulse + vb;
  timer_channel_c = last_pwm_pulse + vc;
  timer_channel_d = last_pwm_pulse + vd;

  // All pins stay HIGH for at least 1000uS
  // so we have 1000uS of time right here
  // begin
  // -- do something that is < 1000us guaranteed
  // -- one really good idea is the MPU read, that will for the next 
  // -- processing / pid loop

  // read_mpu_process(); 576us without and conversions, just the raw gyro+accel read
  // mpu_conversion_process(); 668us 
  
  // end

  motors = B00001111;
  while( motors ) 
  {
      esc_pwm_timmer = micros();
      if( (motors & B00001000) && ( esc_pwm_timmer > timer_channel_a )){ PORTD &= B10111111; motors &= B00000111; }
      if( (motors & B00000100) && ( esc_pwm_timmer > timer_channel_b )){ PORTB &= B11110111; motors &= B00001011; }
      if( (motors & B00000010) && ( esc_pwm_timmer > timer_channel_c )){ PORTB &= B11111011; motors &= B00001101; }
      if( (motors & B00000001) && ( esc_pwm_timmer > timer_channel_d )){ PORTB &= B11111101; motors &= B00001110; }      
  }
}
