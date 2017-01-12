#define LED_PIN 13            // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define LOG_FREQUENCY  250    // ms DEBUG Logging interval

#define INIT_CLEARED          B00000000
#define INIT_ESC_ATTACHED     B00000001
#define INIT_ESC_ARMED        B00000010
#define INIT_THROTTLE_ACTIVE  B00000100
#define INIT_MOTORS_ENABLED   B00001000
#define INIT_MPU_ENABLED      B00010000
#define INIT_MPU_STABLE       B00100000
#define INIT_PID_ON           B01000000

////////////////////////////////////////////////////////////////
// ESC Settings
#define MAX_ESC_SIGNAL    2000    // This is the max output that will be sent to ESC.
#define MIN_ESC_CUTOFF    1100    // Minimum ESC signal to spin props
#define MIN_ESC_SIGNAL    1000    // Minimum ESC signal, attach esc with this singal, should be not prop spinning.
#define MOTOR_PIN_A       6 
#define MOTOR_PIN_B       9       
#define MOTOR_PIN_C       11       
#define MOTOR_PIN_D       10     

#define MIN_INPUT_THRUST  0
#define MAX_INPUT_THRUST  1000


byte system_check = INIT_CLEARED;
