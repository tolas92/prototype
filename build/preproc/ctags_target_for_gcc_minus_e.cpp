# 1 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */

   /* The Pololu VNH5019 dual motor driver shield */
   //#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA

   /* Encoders directly attached to Arduino board */


   /* L298 Motor driver*/



//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h


/* Serial port baud rate */


/* Maximum PWM signal */



# 80 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 2




/* Include definition of serial commands */
# 86 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 2

/* Sensor functions */
# 89 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 2

/* Include servo support if required */






  /* Motor driver function definitions */
# 99 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 2

  /* Encoder driver function definitions */
# 102 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 2

  /* PID parameters and functions */
# 105 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 2
  /* Run the PID loop at 30 times per second */


  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / 30 /* Hz*/;

  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */

  long lastMotorCommand = 2000;


/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
int arg1;
int arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = 
# 142 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3 4
       __null
# 142 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
           ;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case 'b':
    Serial.println(115200);
    break;
  case 'a':
    Serial.println(analogRead(arg1));
    break;
  case 'd':
    Serial.println(digitalRead(arg1));
    break;
  case 'x':
    analogWrite(arg1, arg2);
    Serial.println("OK");
    break;
  case 'w':
    if (arg2 == 0) digitalWrite(arg1, 0x0);
    else if (arg2 == 1) digitalWrite(arg1, 0x1);
    Serial.println("OK");
    break;
  case 'c':
    if (arg2 == 0) pinMode(arg1, 0x0);
    else if (arg2 == 1) pinMode(arg1, 0x1);
    Serial.println("OK");
    break;
  case 'p':
    Serial.println(Ping(arg1));
    break;
# 198 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
  case 'e':
    Serial.print(readEncoder(0));
    Serial.print(" ");
    Serial.println(readEncoder(1));
    break;
   case 'r':
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case 'm':
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK");
    break;
  case 'o':
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Sneaky way to temporarily disable the PID
    setMotorSpeeds(arg1,arg2);
    Serial.println("OK");
    break;
  case 'u':
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;

  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(115200);

// Initialize the motor controller if used */


    //set as inputs
    
# 255 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
   (*(volatile uint8_t *)((0x0A) + 0x20)) 
# 255 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
        &= ~(1<<
# 255 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                2 
# 255 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                /*pin 2*/);
    
# 256 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
   (*(volatile uint8_t *)((0x0A) + 0x20)) 
# 256 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
        &= ~(1<<
# 256 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                3 
# 256 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                /*pin 3*/);
    
# 257 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
   (*(volatile uint8_t *)((0x07) + 0x20)) 
# 257 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
        &= ~(1<<
# 257 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                4 
# 257 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                /*pin A4*/);
    
# 258 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
   (*(volatile uint8_t *)((0x07) + 0x20)) 
# 258 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
        &= ~(1<<
# 258 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                5 
# 258 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                /*pin A5*/);

    //enable pull up resistors
    
# 261 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
   (*(volatile uint8_t *)((0x0B) + 0x20)) 
# 261 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
         |= (1<<
# 261 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                2 
# 261 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                /*pin 2*/);
    
# 262 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
   (*(volatile uint8_t *)((0x0B) + 0x20)) 
# 262 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
         |= (1<<
# 262 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                3 
# 262 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                /*pin 3*/);
    
# 263 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
   (*(volatile uint8_t *)((0x08) + 0x20)) 
# 263 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
         |= (1<<
# 263 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                4 
# 263 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                /*pin A4*/);
    
# 264 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
   (*(volatile uint8_t *)((0x08) + 0x20)) 
# 264 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
         |= (1<<
# 264 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                5 
# 264 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                /*pin A5*/);

    // tell pin change mask to listen to left encoder pins
    
# 267 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
   (*(volatile uint8_t *)(0x6D)) 
# 267 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
          |= (1 << 
# 267 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                   2 
# 267 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                   /*pin 2*/)|(1 << 
# 267 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                                         3 
# 267 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                                         /*pin 3*/);
    // tell pin change mask to listen to right encoder pins
    
# 269 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
   (*(volatile uint8_t *)(0x6C)) 
# 269 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
          |= (1 << 
# 269 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                   4 
# 269 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                   /*pin A4*/)|(1 << 
# 269 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                                          5 
# 269 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                                          /*pin A5*/);

    // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    
# 272 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
   (*(volatile uint8_t *)(0x68)) 
# 272 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
         |= (1 << 
# 272 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                  1
# 272 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                       ) | (1 << 
# 272 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3
                                 2
# 272 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                                      );

  initMotorController();
  resetPID();


/* Attach servos if used */
# 288 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = 
# 302 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3 4
                                  __null
# 302 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                                      ;
      else if (arg == 2) argv2[index] = 
# 303 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3 4
                                       __null
# 303 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                                           ;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[index] = 
# 312 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3 4
                      __null
# 312 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                          ;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

// If we are using base control, run a PID calculation at the appropriate intervals

  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > 2000) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }


// Sweep servos






}
# 1 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/encoder_driver.ino"
/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
# 31 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/encoder_driver.ino"
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0}; //encoder lookup table

  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  
# 36 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/encoder_driver.ino" 3
 extern "C" void __vector_11 (void) __attribute__ ((signal,used, externally_visible)) ; void __vector_11 (void)
# 36 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/encoder_driver.ino"
                  {
   static uint8_t enc_last=0;

 enc_last <<=2; //shift previous state two places
 enc_last |= (
# 40 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/encoder_driver.ino" 3
             (*(volatile uint8_t *)((0x09) + 0x20)) 
# 40 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/encoder_driver.ino"
                  & (3 << 2)) >> 2; //read the current state into lowest 2 bits

   left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  
# 46 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/encoder_driver.ino" 3
 extern "C" void __vector_10 (void) __attribute__ ((signal,used, externally_visible)) ; void __vector_10 (void)
# 46 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/encoder_driver.ino"
                  {
        static uint8_t enc_last=0;

 enc_last <<=2; //shift previous state two places
 enc_last |= (
# 50 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/encoder_driver.ino" 3
             (*(volatile uint8_t *)((0x06) + 0x20)) 
# 50 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/encoder_driver.ino"
                  & (3 << 4)) >> 4; //read the current state into lowest 2 bits

   right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == 0) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == 0){
      left_enc_pos=0L;
      return;
    } else {
      right_enc_pos=0L;
      return;
    }
  }




/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(0);
  resetEncoder(1);
}
# 1 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/motor_driver.ino"
/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/
# 59 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/motor_driver.ino"
  void initMotorController() {
    digitalWrite(2, 0x1);
    digitalWrite(6, 0x1);
  }

  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;

    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;

    if (i == 0) {
      if (reverse == 0) { analogWrite(8, spd); analogWrite(7, 0); }
      else if (reverse == 1) { analogWrite(7, spd); analogWrite(8, 0); }
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if (reverse == 0) { analogWrite(3, spd); analogWrite(4, 0); }
      else if (reverse == 1) { analogWrite(4, spd); analogWrite(3, 0); }
    }
  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(0, leftSpeed);
    setMotorSpeed(1, rightSpeed);
    Serial.print(leftSpeed);
    Serial.print(",");
    Serial.println(rightSpeed);
  }
# 1 "/home/tolasing/ros2_ws/src/arduino_package/ros_arduino_bridge/ROSArduinoBridge/servos.ino"
/***************************************************************
   Servo Sweep - by Nathaniel Gallinger

   Sweep servos one degree step at a time with a user defined
   delay in between steps.  Supports changing direction 
   mid-sweep.  Important for applications such as robotic arms
   where the stock servo speed is too fast for the strength
   of your system.

 *************************************************************/
