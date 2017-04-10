
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
void Beep(uint32 length, uint8 pitch);
void Measure_Voltage();
void BatteryLed_Write(uint8 value);

// Custom class for measuring battery voltage at regular intervals, and for playing warning sounds & flashing lights at low voltage levels.



// Subroutine for custom unit conversion (used below in Measure_Voltage()).
float ADC_result_to_volts(int16 adcresult)
{
    float volts;
    
    volts = (adcresult / 4095.0) * (1.5 * 5.0);
    
    return volts;
    
}

// For measuring the voltage of the batteries at regular intervals. Used in main.c.
void Measure_Voltage()
{
    int16 adcresult = 0;
    float volts = 0.0;
    
    int i;
    
    ADC_Battery_StartConvert();
    if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT)) 
    {   // wait for get ADC converted value
        adcresult = ADC_Battery_GetResult16();
        volts = ADC_result_to_volts(adcresult); // convert value to volts
        
        // Different types of beep sounds for different voltage levels. If the charge is over 5.0 V, there will be 
        // three long and low-sounding beeps; from 5.0 to 4.5 V, six medium-length, medium-pitch beeps; from 4.5 V to 
        // 4.1 V, 9 loud and sharp beeps; and finally if the voltage reaches 4.1 V, there will be a never-ending flurry of extremely short and sharp beeps.
        // EDIT: Added flashing light effects to go with the beeps :)
        if (volts) // null check is needed to prevent alarms when the robot is off but the chip is on
        {
        
            if (volts >= 5.0)
            { for (i = 0; i < 3; i++) 
                {
                    Beep(80, 60);
                    BatteryLed_Write(1);
                    CyDelay(600);
                    BatteryLed_Write(0);
                }
            } else if (volts < 5.0 && volts >= 4.5) {
             for (i = 0; i < 6; i++) 
                {
                    Beep(40, 80);
                    BatteryLed_Write(1);
                    CyDelay(500);
                    BatteryLed_Write(0);
                }            
            } else if (volts < 4.5 && volts >= 4.1) {
            
                for (i = 0; i < 9; i++) 
                {
                    Beep(30, 95);
                    BatteryLed_Write(1);
                    CyDelay(400);
                    BatteryLed_Write(0);
                }       
            } else if (volts < 4.1 && volts > 0.0) {
            
                while (1) 
                {
                    Beep(20, 110);
                    BatteryLed_Write(1);
                    CyDelay(200);
                    BatteryLed_Write(0);
                    printf("%f \n", volts);
                    printf("DANGER! Battery LOW! Please charge ASAP!!!\n");
                }
           
            }
                  
            // Print bit & voltage values
            printf("%d %f\r\n",adcresult, volts);
        }
            CyDelay(500);
    }
}










