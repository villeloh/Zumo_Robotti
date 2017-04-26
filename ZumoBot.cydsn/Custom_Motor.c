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
void Turn(uint32 turn, int dir_flag, float diff_ave);


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

// A new turn method that takes direction as a parameter.
// Also, if we're already moving towards the center of the line, turns in the opposite direction instead.
// This should accomplish what I tried to do with the 'corrective twitch' method earlier (in a much clumsier way).
void Turn(uint32 turn, int dir_flag, float blackDiff)
{
    
    
    if (dir_flag == 1)
    {
        
        if (blackDiff >= -200)
        {
            PWM_WriteCompare2(240 - turn);
            PWM_WriteCompare1(255);
            
        } else if (blackDiff < -200 && blackDiff >= -550) {
            
            turn *= 0.5;
            PWM_WriteCompare1( 255 - turn ); // turn is less steep if diff_ave >= -300
            PWM_WriteCompare2(240);
        
        } else if (blackDiff < -550) {
        
            PWM_WriteCompare1(255 - turn); 
            PWM_WriteCompare2(240);
        }

    } else if (dir_flag == 2) {
    
        if (blackDiff >= -200)
        {
            
            PWM_WriteCompare1(255 - turn);
            PWM_WriteCompare2(240);
            
        } else if (blackDiff < -200 && blackDiff >= -550) {
                     
            turn *= 0.5;
            PWM_WriteCompare2( 240 - turn ); // turn is less steep if diff_ave >= -300
            PWM_WriteCompare1(255);
        
        } else if (blackDiff < -550) {            
            
            PWM_WriteCompare2( 240 - turn ); 
            PWM_WriteCompare1(255);
        } 
    
    }

}










