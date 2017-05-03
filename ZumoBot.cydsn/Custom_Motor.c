
/**
 * @file    Custom_Motor.c
 * @brief   Custom file for defining motor movement methods.
*/

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
void Custom_backward(uint8 speed);
void Turn(uint32 turn, int dir_flag);
void Backward_turn(uint32 turn, int dir_flag);
void Ultrasharp_turn(uint32 delay, int dir_flag);

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

// Custom method for backward movement, similar to the forward method defined above.
void Custom_backward(uint8 speed)
{
    MotorDirLeft_Write(1);      // set LeftMotor backward mode
    MotorDirRight_Write(1);     // set RightMotor backward mode
    PWM_WriteCompare1(speed + 15); 
    PWM_WriteCompare2(speed);
}

// A new turn method that takes direction as an argument.
// NOTE: the normalization of 'turn' within acceptable limits (0-240) is done in main.c, to make it more transparent.
void Turn(uint32 turn, int dir_flag)
{   
    // left side turn logic
    if (dir_flag == 1)
    {
        
        // Turns right when left sensor activated.
        PWM_WriteCompare2(240 - turn); 
        PWM_WriteCompare1(255);
            
     // right side turn logic
    } else if (dir_flag == 2) {

        // Turns left when right sensor activated.
        PWM_WriteCompare1(255 - turn); 
        PWM_WriteCompare2(240);

    }

}

void Backward_turn(uint32 turn, int dir_flag)
{   
    MotorDirLeft_Write(1);      // set LeftMotor backward mode
    MotorDirRight_Write(1);     // set RightMotor backward mode
    
    // left side turn logic
    if (dir_flag == 1)
    {
        
        // Turns right when left sensor activated.
        PWM_WriteCompare1(255 - turn); 
        PWM_WriteCompare2(240);
            
     // right side turn logic
    } else if (dir_flag == 2) {

        // Turns left when right sensor activated.
        PWM_WriteCompare2(240 - turn); 
        PWM_WriteCompare1(255);

    }

}

void Ultrasharp_turn(uint32 delay, int dir_flag)
{
    if (dir_flag == 1)
    { 
        MotorDirLeft_Write(1);
        MotorDirRight_Write(0);
        PWM_WriteCompare1(255);
        PWM_WriteCompare2(240);
        CyDelay(delay);
    
    } else if (dir_flag == 2) {
    
        MotorDirLeft_Write(0);
        MotorDirRight_Write(1);
        PWM_WriteCompare1(255);
        PWM_WriteCompare2(240);
        CyDelay(delay);
    
    }

}







