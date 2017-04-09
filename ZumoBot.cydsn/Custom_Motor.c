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

void Custom_forward(uint8 speed,uint32 delay);
void Custom_left_correct(uint8 speed);
void Custom_right_correct(uint8 speed);


// Custom class for defining motor movement methods.



// Custom 'forward' method to calibrate the forward moving direction (default motor_forward() veers to the left due to the physical differences 
// between the two motors).
// NOTE: The motors should always be stopped using the default method, because this method will leave the left motor running if used for that purpose!
void Custom_forward(uint8 speed,uint32 delay)
{
    MotorDirLeft_Write(0);      // set LeftMotor forward mode
    MotorDirRight_Write(0);     // set RightMotor forward mode
    PWM_WriteCompare1(speed + 7); 
    PWM_WriteCompare2(speed); 
    CyDelay(delay);
}

// Called in main.c when the robot is starting to veer to the left (off the black line).
// NOTE: It SLOWS down the right motor in order to turn to the right! This is to preserve speed, as we can drive at full speed by default if we use this approach
// (instead of speeding up the left motor to turn).
void Custom_left_correct(uint8 speed)
{
    PWM_WriteCompare2(speed); 
}

// Called in main.c when the robot is starting to veer to the right (off the black line).
// NOTE: It SLOWS down the left motor in order to turn to the left! This is to preserve speed, as we can drive at full speed by default if we use this approach
// (instead of speeding up the right motor to turn).
void Custom_right_correct(uint8 speed)
{
        PWM_WriteCompare1(speed);
}














