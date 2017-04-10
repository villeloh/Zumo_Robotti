#include <project.h>
#include <stdio.h>
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "I2C_made.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "IR.h"
#include "Ambient.h"
#include "Beep.h"

void Custom_forward(uint8 speed);
void Right_turn(uint8 speed);
void Left_turn(uint8 speed);
void Forceful_left_turn(uint8 speed);
void Forceful_right_turn(uint8 speed);
void Corrective_twitch(uint8 dir);



// Custom class for defining motor movement methods.

// Custom 'forward' method to calibrate the forward moving direction (default motor_forward() veers 
// to the left due to the physical differences between the two motors).
// NOTE: The motors should always be stopped by using the default method, because this method 
// will leave the left motor running if used for that purpose!
void Custom_forward(uint8 speed)
{
    MotorDirLeft_Write(0);      // set LeftMotor forward mode
    MotorDirRight_Write(0);     // set RightMotor forward mode
    PWM_WriteCompare1(speed + 7); 
    PWM_WriteCompare2(speed); 

}

// Called in main.c when the robot is starting to veer to the left (off the black line).
// NOTE: It SLOWS down the right motor in order to turn to the right! This is to preserve 
// speed, as we can drive at full speed by default if we use this approach
// (instead of speeding up the left motor to turn).
void Right_turn(uint8 speed)
{   
    PWM_WriteCompare2(speed); 
}

// Called in main.c when the robot is starting to veer to the right (off the black line).
// NOTE: It SLOWS down the left motor in order to turn to the left! This is to preserve 
// speed, as we can drive at full speed by default if we use this approach
// (instead of speeding up the right motor to turn).
void Left_turn(uint8 speed)
{
        PWM_WriteCompare1(speed);
}

// For use in steep curves. Turns to the left more forcefully, by spinning the left track backwards
void Forceful_left_turn(uint8 speed)
{
    MotorDirLeft_Write(1); 
    PWM_WriteCompare1(248 - speed); // to preseve the behaviour where a lower speed value leads to a steeper turn 
}

// For use in steep curves. Turns to the right more forcefully, by spinning the right track backwards
void Forceful_right_turn(uint8 speed)
{
    MotorDirRight_Write(1); 
    PWM_WriteCompare2(248 - speed); // to preseve the behaviour where a lower speed value leads to a steeper turn
}

// An experimental method to be used for a slight 'corrective twitch' after a right or left turn.
// At the moment, it simply stops the respective motor for a small, fixed amount of time (technically *forever*, 
// but due to where this method will be called, in practice this should never actually occur).
void Corrective_twitch(uint8 dir)
{
    // '0' = left turn
    if (dir == 0) 
    {
        PWM_WriteCompare1(0);
        CyDelay(2); // dummy value; optimal value must be found experimentally
        
      // '1' = right turn  
    } else if (dir == 1) {
    
        PWM_WriteCompare2(0);
        CyDelay(2); // dummy value; optimal value must be found experimentally
    }

}












