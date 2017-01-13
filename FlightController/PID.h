/* 
 *  YMFC PID values
 *  yaw: 4.0, 0.02, 0.0
 *  pitch/roll : 1.04, 0.05, 15
 */

float pid_yaw_rate;
float pid_pitch_rate;
float pid_roll_rate;

void init_pid()
{
  system_check |= INIT_PID_ON ;
}

void pid_reset() 
{
}

void do_pid_compute()
{

}
