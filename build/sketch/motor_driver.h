#line 1 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/motor_driver.h"
/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 4
  #define LEFT_MOTOR_BACKWARD  7
  #define RIGHT_MOTOR_FORWARD  3
  #define LEFT_MOTOR_FORWARD   8
  #define RIGHT_MOTOR_ENABLE 2  
  #define LEFT_MOTOR_ENABLE 6
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
