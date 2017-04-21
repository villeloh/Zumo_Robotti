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
void Turn(uint8 turn, int dir_flag, float diff_ave);


// Custom class for defining motor movement methods.

// Custom 'forward' method to calibrate the forward moving direction (default motor_forward() veers 
// to the left due to the physical differences between the two motors).
// NOTE: The motors should always be stopped by using the default method, because this method 
// will leave the left motor running if used for that purpose!
void Custom_forward(uint8 speed)
{
    MotorDirLeft_Write(0);      // set LeftMotor forward mode
    MotorDirRight_Write(0);     // set RightMotor forward mode
    PWM_WriteCompare1(speed + 15); 
    PWM_WriteCompare2(speed);
}

// Called in main.c when the robot is starting to veer to the left (off the black line).
// NOTE: It SLOWS down the right motor in order to turn to the right! This is to preserve 
// speed, as we can drive at full speed by default if we use this approach
// (instead of speeding up the left motor to turn).
void Right_turn(uint8 speed)
{   
    PWM_WriteCompare2(240 - speed); 
}

// Called in main.c when the robot is starting to veer to the right (off the black line).
// NOTE: It SLOWS down the left motor in order to turn to the left! This is to preserve 
// speed, as we can drive at full speed by default if we use this approach
// (instead of speeding up the right motor to turn).
void Left_turn(uint8 speed)
{
        PWM_WriteCompare1(255 - speed); // calibration is needed in the turn method as well
}

// A new turn method that takes direction as a parameter.
// Also, if we're already moving towards the center of the line, turns in the opposite direction instead.
// This should accomplish what I tried to do with the 'corrective twitch' method earlier (in a much clumsier way).
void Turn(uint8 turn, int dir_flag, float diff_ave)
{
    
    if (dir_flag == 1)
    {
        
        if (diff_ave >= 0)
        {
            PWM_WriteCompare2(240 - turn);
            
        } else if (diff_ave < 0 && diff_ave >= -300) {
            
            PWM_WriteCompare1( 1.5 * (255 - turn) ); // turn is less steep if diff_ave >= -300
        
        } else if (diff_ave < -300) {
        
            PWM_WriteCompare1(255 - turn); // regular steepness of turn
            
        }

    } else if (dir_flag == 2) {
    
        if (diff_ave >= 0)
        {
            PWM_WriteCompare1(255 - turn);
            
        } else if (diff_ave < 0 && diff_ave >= -300) {
            
            PWM_WriteCompare2( 1.5 * (240 - turn) ); // turn is less steep if diff_ave >= -300
        
        } else if (diff_ave < -300) {
            
            PWM_WriteCompare2(240 - turn); // regular steepness of turn
        
        } 
    
    }

}










