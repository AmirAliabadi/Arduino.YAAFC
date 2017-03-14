/* 
 *  YMFC PID values
 *  yaw: 4.0, 0.02, 0.0
 *  pitch/roll : 1.04, 0.05, 15
 */

int attitude_pTerm_index = 0;
float attitude_pTerm[3]  = {0.60, 0.75,  0.88};
float  yaw_pid_gains[3]  = {4.00, 0.02,  00.0};
float rate_pid_gains[3]  = {1.03, 0.04,  18.0};

float yaw_pid_term[3]      = {0,0,0};
float pitch_pid_term[3]    = {0,0,0};
float roll_pid_term[3]     = {0,0,0};

float yaw_pid_rate_out     = 0.0;
float pitch_pid_rate_out   = 0.0;
float roll_pid_rate_out    = 0.0;

float pid_error;
float yaw_last_error      = 0.0;
float pitch_last_error    = 0.0;
float roll_last_error     = 0.0;

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

  yaw_last_error      = 0.0;
  pitch_last_error    = 0.0;
  roll_last_error     = 0.0;

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
    pid_error = gyro_yaw - yaw_setpoint/3.0 ;
    yaw_pid_term[0] = yaw_pid_gains[0] * pid_error;                       // pTerm;
    yaw_pid_term[2] = yaw_pid_gains[2] * ( pid_error - yaw_last_error );  // dTerm = dGain * (current error - last_error)
    yaw_last_error = pid_error;                                           // update yaw last_error for next time

    yaw_pid_term[1] += yaw_pid_gains[1] * pid_error;                         // integrate the iTerm

    if( yaw_pid_term[1] >  400.0 ) yaw_pid_term[1] =  400.0;
    if( yaw_pid_term[1] < -400.0 ) yaw_pid_term[1] = -400.0;       

    yaw_pid_rate_out  = yaw_pid_term[0] + yaw_pid_term[1] + yaw_pid_term[2] ;

    if( yaw_pid_rate_out >  400.0 ) yaw_pid_rate_out =  400.0;
    if( yaw_pid_rate_out < -400.0 ) yaw_pid_rate_out = -400.0;  


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // COMPUTE PITCH PID

    // pitch_angle
    // (pitch_setpoint - 15) / 3.0
    // attitude_error = ((pitch_angle - pitch_setpoint) * .5 )

    //pid_error = gyro_pitch - pitch_setpoint ; 
    pid_error = gyro_pitch + ( attitude_pTerm[attitude_pTerm_index] * (pitch_angle - pitch_setpoint/15.0) ) ; 
    
    pitch_pid_term[0] = rate_pid_gains[0] * pid_error;                          // pTerm;    
    pitch_pid_term[2] = rate_pid_gains[2] * ( pid_error - pitch_last_error );   // dTerm = dGain * (current error - last_error)
    pitch_last_error = pid_error;                                               // update pitch last_error for next time
    
    pitch_pid_term[1] += rate_pid_gains[1] * pid_error;                         // integrate the iTerm

    if( pitch_pid_term[1] >  400.0 ) pitch_pid_term[1] =  400.0;
    if( pitch_pid_term[1] < -400.0 ) pitch_pid_term[1] = -400.0;    
    
    pitch_pid_rate_out  = pitch_pid_term[0] + pitch_pid_term[1] + pitch_pid_term[2] ;

    if( pitch_pid_rate_out >  400.0 ) pitch_pid_rate_out =  400.0;
    if( pitch_pid_rate_out < -400.0 ) pitch_pid_rate_out = -400.0;    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // COMPUTE ROLL PID

    // roll_angle
    // (roll_setpoint - 15) / 3.0
    // attitude_error = (roll_setpoint - roll_angle)

    
    //pid_error = gyro_roll - roll_setpoint;  
    pid_error = gyro_roll + ( attitude_pTerm[attitude_pTerm_index] * (roll_angle - roll_setpoint/15.0) ) ; 
       
    roll_pid_term[0] = rate_pid_gains[0] * pid_error;                           // pTerm;    
    roll_pid_term[2] = rate_pid_gains[2] * ( pid_error - roll_last_error );     // dTerm = dGain * (current error - last_error)    
    roll_last_error = pid_error;                                                // update roll last_error for next time   

    roll_pid_term[1] += rate_pid_gains[1] * pid_error;                          //  integrate the iTerm
    if( roll_pid_term[1] >  400.0 ) roll_pid_term[1] =  400.0;
    if( roll_pid_term[1] < -400.0 ) roll_pid_term[1] = -400.0;    
    
    roll_pid_rate_out  = roll_pid_term[0] + roll_pid_term[1] + roll_pid_term[2] ; 

    if( roll_pid_rate_out >  400.0 ) roll_pid_rate_out =  400.0;
    if( roll_pid_rate_out < -400.0 ) roll_pid_rate_out = -400.0;  

}
