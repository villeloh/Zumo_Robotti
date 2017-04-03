
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

float ADC_result_to_volts(int16 adcresult);

float ADC_result_to_volts(int16 adcresult)
{
    float volts;
    
    volts = (adcresult / 4095.0) * (1.5 * 5.0);
    // printf("using custom func!\n");
    
    return volts;
    
}











