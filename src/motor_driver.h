/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef CYTRON_MOTOR_DRIVER
  #define LEFT_DIR 2
  #define LEFT_PWM 3
  #define RIGHT_DIR 4
  #define RIGHT_PWM 5
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
