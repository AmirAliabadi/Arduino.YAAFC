/* 
 *  YMFC PID values
 *  yaw: 4.0, 0.02, 0.0
 *  pitch/roll : 1.04, 0.05, 15
 */

float y_pid_gains[3]  = {5.00, 0.00,  00.0};
float p_pid_gains[3]  = {1.04, 0.04,  15.0};
float r_pid_gains[3]  = {1.04, 0.04,  15.0};

float yaw_pid_term[3]      = {0,0,0};
float pitch_pid_term[3]    = {0,0,0};
float roll_pid_term[3]     = {0,0,0};

float yaw_pid_rate_out     = 0.0;
float pitch_pid_rate_out   = 0.0;
float roll_pid_rate_out    = 0.0;

float pid_error;
float y_last_error    = 0.0;
float p_last_error    = 0.0;
float r_last_error    = 0.0;

void init_pid()
{
#ifdef DEBUG
    Serial.println( "init_pid" );
#endif
  
  system_check |= INIT_PID_ON ;
}

void pid_reset() 
{
  yaw_pid_rate_out   = 0.0;
  pitch_pid_rate_out = 0.0;
  roll_pid_rate_out  = 0.0;

  y_last_error    = 0.0;
  p_last_error    = 0.0;
  r_last_error    = 0.0;

  for( byte b =0; b<3; b++ ) {
    yaw_pid_term[b]   = 0;
    pitch_pid_term[b] = 0;
    roll_pid_term[b]  = 0;  
  }
  
}

void do_pid_compute()
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // COMPUTE YAW PID
    pid_error = gyro_yaw - yaw_setpoint ;
    yaw_pid_term[0] = y_pid_gains[0] * pid_error;                     // pTerm;
    yaw_pid_term[2] = y_pid_gains[2] * ( pid_error - y_last_error );  // dTerm = dGain * (current error - last_error)
    y_last_error = pid_error;                                         // update yaw last_error for next time

    yaw_pid_rate_out  = yaw_pid_term[0] + yaw_pid_term[1] + yaw_pid_term[2] ;

    if( yaw_pid_rate_out >  400.0 ) yaw_pid_rate_out =  400.0;
    if( yaw_pid_rate_out < -400.0 ) yaw_pid_rate_out = -400.0;  


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // COMPUTE PITCH PID

    // pitch_angle
    // (pitch_setpoint - 15) / 3.0
    
    pid_error = gyro_pitch - pitch_setpoint ; 
    pitch_pid_term[0] = p_pid_gains[0] * pid_error;                     // pTerm;    
    pitch_pid_term[2] = p_pid_gains[2] * ( pid_error - p_last_error );  // dTerm = dGain * (current error - last_error)
    p_last_error = pid_error;                                           // update pitch last_error for next time
    
    pitch_pid_term[1] += p_pid_gains[1] * pid_error;                    // integrate the iTerm

    if( pitch_pid_term[1] >  400.0 ) pitch_pid_term[1] =  400.0;
    if( pitch_pid_term[1] < -400.0 ) pitch_pid_term[1] = -400.0;    
    
    pitch_pid_rate_out  = pitch_pid_term[0] + pitch_pid_term[1] + pitch_pid_term[2] ;

    if( pitch_pid_rate_out >  400.0 ) pitch_pid_rate_out =  400.0;
    if( pitch_pid_rate_out < -400.0 ) pitch_pid_rate_out = -400.0;    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // COMPUTE ROLL PID

    // roll_angle
    // (roll_setpoint - 15) / 3.0
    
    pid_error = gyro_roll - roll_setpoint;     
    roll_pid_term[0] = r_pid_gains[0] * pid_error;                      // pTerm;    
    roll_pid_term[2] = r_pid_gains[2] * ( pid_error - r_last_error );   // dTerm = dGain * (current error - last_error)    
    r_last_error = pid_error;                                           // update roll last_error for next time   

    roll_pid_term[1] += r_pid_gains[1] * pid_error;                     //  integrate the iTerm
    if( roll_pid_term[1] >  400.0 ) roll_pid_term[1] =  400.0;
    if( roll_pid_term[1] < -400.0 ) roll_pid_term[1] = -400.0;    
    
    roll_pid_rate_out  = roll_pid_term[0] + roll_pid_term[1] + roll_pid_term[2] ; 

    if( roll_pid_rate_out >  400.0 ) roll_pid_rate_out =  400.0;
    if( roll_pid_rate_out < -400.0 ) roll_pid_rate_out = -400.0;  

}
