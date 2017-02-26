#define PWM_FERQUENCY 4000 // 250hz, very stable
//#define PWM_FERQUENCY 4000 // flight tested : works
//#define PWM_FERQUENCY 3300 // flight tested : works
//#define PWM_FERQUENCY 3100 // 323hz, untested

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
#ifdef DEBUG
    Serial.println( "init_esc" );
#endif   
  
//  DDRD |= B00001000;      // Configure digital poort 3 as output
//  DDRD |= B00100000;      // Configure digital poort 5 as output
//  DDRD |= B01000000;      // Configure digital poort 6 as output
//  DDRB |= B00001110;      // Configure digital poort 9, 10, 11 as output.
  DDRD |= B11110000;        // Configure digital poort 4, 5, 6 and 7 as output.

  system_check |= INIT_ESC_ATTACHED;
}

void arm_esc()
{
#ifdef DEBUG
    Serial.println( "arm_esc" );
#endif     
  system_check |= INIT_ESC_ARMED;
}

void disarm_esc()
{
#ifdef DEBUG
    Serial.println( "disarm_esc" );
#endif   
  system_check &= ~(INIT_ESC_ARMED);
}

void update_motors()
{
  // wait for next rising pulse time
  while( (micros() - last_pwm_pulse) < PWM_FERQUENCY );
  last_pwm_pulse= micros(); 

  // all PWM pins HIGH
// PORTD |= B00001000; // Set digital port 3 high
// PORTD |= B01000000; // Set digital port 6 high
// PORTB |= B00001110; // Set digital port 9,10,11 high

  PORTD |= B11111000;  // set digital pins 4,5,6,7 high

  // compute when each PWM pin should go low
  timer_channel_a = last_pwm_pulse + va;
  timer_channel_b = last_pwm_pulse + vb;
  timer_channel_c = last_pwm_pulse + vc;
  timer_channel_d = last_pwm_pulse + vd;

  // All pins stay HIGH for at least 1000uS
  // so we have ~1000uS of time right here
  // begin
  // -- do something that is < 1000us guaranteed
  //digitalWrite(12,HIGH);

  read_mpu_process(); // 784us
  // mpu_conversion_process(); 668us 

  //digitalWrite(12,LOW);   
  // end

  motors = B00001111;
  while( motors ) 
  {
      esc_pwm_timmer = micros();
//      if( (motors & B00001000) && ( esc_pwm_timmer > timer_channel_a )){ PORTD &= B10111111; motors &= B00000111; } // pin 6 low
//      if( (motors & B00000100) && ( esc_pwm_timmer > timer_channel_b )){ PORTB &= B11110111; motors &= B00001011; }
//      if( (motors & B00000010) && ( esc_pwm_timmer > timer_channel_c )){ PORTB &= B11111011; motors &= B00001101; }
//      if( (motors & B00000001) && ( esc_pwm_timmer > timer_channel_d )){ PORTB &= B11111101; motors &= B00001110; }  

        if( (motors & B00001000) && ( esc_pwm_timmer > timer_channel_a )){ PORTD &= B11101111; motors &= B00000111; } // pin 4 low
        if( (motors & B00000100) && ( esc_pwm_timmer > timer_channel_b )){ PORTD &= B11011111; motors &= B00001011; } // pin 5 low
        if( (motors & B00000010) && ( esc_pwm_timmer > timer_channel_c )){ PORTD &= B10111111; motors &= B00001101; } // pin 6 low
        if( (motors & B00000001) && ( esc_pwm_timmer > timer_channel_d )){ PORTD &= B01111111; motors &= B00001110; } // pin 7 low
  }
}
