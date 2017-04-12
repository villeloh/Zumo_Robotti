/**
* @mainpage ZumoBot Project
* @brief    You can make your own ZumoBot with various sensors.
* @details  <br><br>
    <p>
    <B>General</B><br>
    You will use Pololu Zumo Shields for your robot project with CY8CKIT-059(PSoC 5LP) from Cypress semiconductor.This 
    library has basic methods of various sensors and communications so that you can make what you want with them. <br> 
    <br><br>
    </p>
*/
    
/**
 * @file    main.c
 * @brief   
 * @details  ** You should enable global interrupt for operating properly. **<br>&nbsp;&nbsp;&nbsp;CyGlobalIntEnable;<br>
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

int rread(void);
void motor_start();
void motor_forward(uint8 speed,uint32 delay);
void Custom_forward(uint8 speed);
void motor_turn(uint8 l_speed, uint8 r_speed, uint32 delay);
void reflectance_set_threshold(uint16_t l3, uint16_t l1, uint16_t r1, uint16_t r3);
void Measure_Voltage();
void Right_turn(uint8 speed);
void Left_turn(uint8 speed);


int main()
{
    // 'Time counter' for the voltage measurement interval
    int cycles = 0;
    
    // Needed for using the button to start the robot's movement routine
    int exit = 0;
    
    int exitMain = 0;
    
    int check = 0;
    
    // turn value for the motors to use
    uint8 turn = 0;
    
    // color thresholds for use in different behaviours
    int black_threshold_l3 = 21000;
    int black_threshold_l1 = 17500;
    int black_threshold_r1 = 22000;
    int black_threshold_r3 = 21500;
        
    int white_threshold_l3 = 5793;
    int white_threshold_l1 = 4800;
    int white_threshold_r1 = 4822;
    int white_threshold_r3 = 6293;
    
    int maxDiff_l3 = black_threshold_l3 - white_threshold_l3;
    int maxDiff_l1 = black_threshold_l1 - white_threshold_l1;
    int maxDiff_r1 = black_threshold_r1 - white_threshold_r1;
    int maxDiff_r3 = black_threshold_r3 - white_threshold_r3;
    
    int blackness1_left = 0;
    int blackness2_left = 0;
    int blackness1_right = 0;
    int blackness2_right = 0;
    
    float diff_left = 0.0;
    float diff_right = 0.0;
    
    // maximum movement speed of the robot
    int speed = 124;
    
    // reflectance sub-limits for different turning behaviour
    int left_forceful_limit = 14000;
    int right_forceful_limit = 17000;
      
    struct sensors_ ref;
    struct sensors_ dig;
    CyGlobalIntEnable; 
    UART_1_Start();
    ADC_Battery_Start();   
    Measure_Voltage(); // measure battery voltage at robot start
    
    motor_start();
    motor_forward(0, 0); // stop the motor at robot start, as it seems to run at a low speed if simply started up
    
    // Initialize IR sensors
    sensor_isr_StartEx(sensor_isr_handler); 
    reflectance_set_threshold(black_threshold_l3, black_threshold_l1, black_threshold_r1, black_threshold_r3);  
    reflectance_start();
    IR_led_Write(1);

    //printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // To start the robot's movement routine, press the button
    while (exit == 0) 
    {
        button = SW1_Read();
        
        if (button == 0) 
        {
            exit = 1;
            CyDelay(10);
        }
    }
    
    Custom_forward(speed);

    while(exitMain == 0)
    {    
        
        reflectance_read(&ref); // raw sensor value; 0 - 24 000
        //printf("%d %d %d %d \r\n", ref.l3, ref.l1, ref.r1, ref.r3);       //print out each period of reflectance sensors
        reflectance_digital(&dig);      //print out 0 or 1 according to results of reflectance period
        //printf("%d %d %d %d \r\n", dig.l3, dig.l1, dig.r1, dig.r3);        //print out 0 or 1 according to results of reflectance period
        //CyDelay(1000); // comment this delay out when doing movement tests / racing !!!
        
        
        // Line-following logic.
        // NOTE: due to the calibration of the motor speeds, 248 (255 - 7) is our current max speed!
        if (dig.l1 == 1)
        { 
            // When the robot starts to veer off to the left, do a correction towards the right, until the veering off has been corrected.
            do {
                
                // Obtain an initial value for blackness1_left, then close access to the if statement (needed for the loop to work correctly).
                if (check == 0)
                {
                    reflectance_read(&ref);
                    blackness1_left = ref.l1;
                    CyDelay(1);
                    check = 1;                   
                }
                
                // Obtain a second value (1 millisecond later).
                reflectance_read(&ref);
                reflectance_digital(&dig); // needed to check if the turn should be ended
                blackness2_left = ref.l1;
                
                // Calculate the difference between the two values.
                // (A check for negativity is needed due to the 'wobbly' behaviour of the reflectance sensor.)
                diff_left = blackness1_left - blackness2_left;
                if (diff_left < 0)
                {
                    diff_left = 0;
                }
                
                // Determine the correct turn amount (0 is max turn, 248 is no turn).
                
                // The 'base' turn amount is simply the robot's speed value multiplied by the latest measurement 
                // divided by the maximum (threshold) blackness value (determined for each sensor at the edge of the black line).
                // This is then further modified by the rate of change multiplied by a coefficient (to be determined experimentally; dummy value 5000).
                // The more rapid the change of values (i.e. the difference between two measurements), the less the final value of turn 
                // becomes, thus leading to a steeper turn. 
                // Finally, a < 0 check was added to pre-empt any potential issues with large diff values.
                turn = speed * (ref.l1 / black_threshold_l1);
                turn -= ( 5000 * (diff_left / maxDiff_l1));
                if (turn < 0)
                {
                    turn = 0;
                }
                                
                // Execute the turn.
                Right_turn(turn);
                             
                // Store the value of the second measurement to the variable for the first.
                // As the loop continues, blackness2's value is stored in blackness1 and then blackness2 gets a new, measured value; etc.
                blackness1_left = blackness2_left;
                
                CyDelay(1);
                               
                //printf("%d\n", turn);
                
            } while (dig.l1 == 1);
            // Return 'check' to its initial value, so that we can get an initial measurement 
            // once the loop starts again (i.e. when the robot needs to turn again).
            check = 0;
            // Since the turn has ended, continue forward with the designated speed.
            Custom_forward(speed);
                    
        } else if (dig.r1 == 1) {
            
            // When the robot starts to veer off to the right, do a correction towards the left, until the veering off has been corrected.
            do {
                
                // Obtain an initial value for blackness1_right, then close access to the if statement (needed for the loop to work correctly).
                if (check == 0)
                {
                    reflectance_read(&ref);
                    blackness1_right = ref.r1;
                    CyDelay(1);
                    check = 1;
                }
                
                // Obtain a second value (1 millisecond later).
                reflectance_read(&ref);
                reflectance_digital(&dig); 
                blackness2_right = ref.r1;
                
                // Calculate the difference between the two values.
                // (A check for negativity is needed due to the 'wobbly' behaviour of the reflectance sensor.)
                diff_right = blackness1_right - blackness2_right;
                if (diff_right < 0)
                {
                    diff_right = 0;
                }
                
                // Determine the correct turn amount (0 is max turn, 248 is no turn).
                
                // The 'base' turn amount is simply the robot's speed value multiplied by the latest measurement 
                // divided by the maximum (threshold) blackness value (determined for each sensor at the edge of the black line).
                // This is then further modified by the rate of change multiplied by a coefficient (to be determined experimentally; dummy value 5000).
                // The more rapid the change of values (i.e. the difference between two measurements), the less the final value of turn 
                // becomes, thus leading to a steeper turn. 
                // Finally, a < 0 check was added to pre-empt any potential issues with large diff values.
                turn = speed * (ref.r1 / black_threshold_r1);
                turn -= ( 5000 * (diff_right / maxDiff_r1));
                if (turn < 0)
                {
                    turn = 0;
                }  
                
                // Execute the turn.
                Left_turn(turn);
                
                // Store the value of the second measurement to the variable for the first.
                // As the loop continues, blackness2's value is stored in blackness1 and then blackness2 gets a new, measured value; etc.
                blackness1_right = blackness2_right;
                                
                CyDelay(1);
                               
                //printf("%d\n", turn);
                
            } while (dig.r1 == 1);
            // Return 'check' to its initial value, so that we can get an initial measurement 
            // once the loop starts again (i.e. when the robot needs to turn again).
            check = 0;
            // Since the turn has ended, continue forward with the designated speed.
            Custom_forward(speed);
        } 
                                    
        // For measuring the battery voltage at regular intervals. 
        // 80000 'cycles' should equal ~80 seconds, due to the delay that is used below (1).
        // NOTE: the cycle limit will have to be adjusted each time we add delays to the while loop! 
        // There must be a way around this, but for now we should keep this in mind and adjust it as needed.
        // NOTE2: If ALL delays are disabled, change the limit to 30 000 000 !
        cycles++;
        if (cycles >= 80000)
        {
            Measure_Voltage();
            cycles = 0;
        }
        
        CyDelay(1);
        
        // If the button is pressed while movement routine is running, exit routine and stop the motors
        button = SW1_Read();
        if (button == 0) 
        {
            exitMain = 1;
            CyDelay(10);
        }
    }
    
    motor_stop();
    while(1) {}
     
 }   

// ===================================================================================================================== //
// ===================================================================================================================== //


//*/

/*//ultra sonic sensor//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    Ultra_Start();                          // Ultra Sonic Start function
    while(1) {
        //If you want to print out the value  
        printf("distance = %5.0f\r\n", Ultra_GetDistance());
        CyDelay(1000);
    }
}   
//*/


/*//nunchuk//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
  
    nunchuk_start();
    nunchuk_init();
    
    for(;;)
    {    
        nunchuk_read();
    }
}   
//*/


/*//IR receiver//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    
    unsigned int IR_val; 
    
    for(;;)
    {
       IR_val = get_IR();
       printf("%x\r\n\n",IR_val);
    }    
 }   
//*/


/*//Ambient light sensor//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
    
    I2C_Start();
    
    I2C_write(0x29,0x80,0x00);          // set to power down
    I2C_write(0x29,0x80,0x03);          // set to power on
    
    for(;;)
    {    
        uint8 Data0Low,Data0High,Data1Low,Data1High;
        Data0Low = I2C_read(0x29,CH0_L);
        Data0High = I2C_read(0x29,CH0_H);
        Data1Low = I2C_read(0x29,CH1_L);
        Data1High = I2C_read(0x29,CH1_H);
        
        uint8 CH0, CH1;
        CH0 = convert_raw(Data0Low,Data0High);      // combine Data0
        CH1 = convert_raw(Data1Low,Data1High);      // combine Data1

        double Ch0 = CH0;
        double Ch1 = CH1;
        
        double data = 0;
        data = getLux(Ch0,Ch1);
        
        // If you want to print out data
        //printf("%lf\r\n",data);    
    }    
 }   
//*/


/*//accelerometer//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
  
    I2C_Start();
  
    uint8 X_L_A, X_H_A, Y_L_A, Y_H_A, Z_L_A, Z_H_A;
    int16 X_AXIS_A, Y_AXIS_A, Z_AXIS_A;
    
    I2C_write(ACCEL_MAG_ADDR, ACCEL_CTRL1_REG, 0x37);           // set accelerometer & magnetometer into active mode
    I2C_write(ACCEL_MAG_ADDR, ACCEL_CTRL7_REG, 0x22);
    
    
    for(;;)
    {
        //print out accelerometer output
        X_L_A = I2C_read(ACCEL_MAG_ADDR, OUT_X_L_A);
        X_H_A = I2C_read(ACCEL_MAG_ADDR, OUT_X_H_A);
        X_AXIS_A = convert_raw(X_L_A, X_H_A);
        
        Y_L_A = I2C_read(ACCEL_MAG_ADDR, OUT_Y_L_A);
        Y_H_A = I2C_read(ACCEL_MAG_ADDR, OUT_Y_H_A);
        Y_AXIS_A = convert_raw(Y_L_A, Y_H_A);
        
        Z_L_A = I2C_read(ACCEL_MAG_ADDR, OUT_Z_L_A);
        Z_H_A = I2C_read(ACCEL_MAG_ADDR, OUT_Z_H_A);
        Z_AXIS_A = convert_raw(Z_L_A, Z_H_A);
        
        printf("ACCEL: %d %d %d %d %d %d \r\n", X_L_A, X_H_A, Y_L_A, Y_H_A, Z_L_A, Z_H_A);
        value_convert_accel(X_AXIS_A, Y_AXIS_A, Z_AXIS_A);
        printf("\n");
        
        CyDelay(50);
    }
}   
//*/


/*//reflectance//
int main()
{
    struct sensors_ ref;
    struct sensors_ dig;
    CyGlobalIntEnable; 
    UART_1_Start();
  
    sensor_isr_StartEx(sensor_isr_handler);
    
    reflectance_start();

    IR_led_Write(1);
    for(;;)
    {
        reflectance_read(&ref); // raw sensor value; 0 - 22 000
        printf("%d %d %d %d \r\n", ref.l3, ref.l1, ref.r1, ref.r3);       //print out each period of reflectance sensors
        reflectance_digital(&dig);      //print out 0 or 1 according to results of reflectance period
        printf("%d %d %d %d \r\n", dig.l3, dig.l1, dig.r1, dig.r3);        //print out 0 or 1 according to results of reflectance period
        
        CyDelay(500);
    }
}   
//*/

 /* //motor//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();

    motor_start();              // motor start

    motor_forward(100,2000);     // moving forward
    motor_turn(200,50,2000);     // turn
    motor_turn(50,200,2000);     // turn
    motor_backward(100,2000);    // movinb backward
       
    motor_stop();               // motor stop
    
    for(;;)
    {

    }
}
//*/
    

/*//gyroscope//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
  
    I2C_Start();
  
    uint8 X_L_G, X_H_G, Y_L_G, Y_H_G, Z_L_G, Z_H_G;
    int16 X_AXIS_G, Y_AXIS_G, Z_AXIS_G;
    
    I2C_write(GYRO_ADDR, GYRO_CTRL1_REG, 0x0F);             // set gyroscope into active mode
    I2C_write(GYRO_ADDR, GYRO_CTRL4_REG, 0x30);             // set full scale selection to 2000dps    
    
    for(;;)
    {
        //print out gyroscope output
        X_L_G = I2C_read(GYRO_ADDR, OUT_X_AXIS_L);
        X_H_G = I2C_read(GYRO_ADDR, OUT_X_AXIS_H);
        X_AXIS_G = convert_raw(X_H_G, X_L_G);
        
        
        Y_L_G = I2C_read(GYRO_ADDR, OUT_Y_AXIS_L);
        Y_H_G = I2C_read(GYRO_ADDR, OUT_Y_AXIS_H);
        Y_AXIS_G = convert_raw(Y_H_G, Y_L_G);
        
        
        Z_L_G = I2C_read(GYRO_ADDR, OUT_Z_AXIS_L);
        Z_H_G = I2C_read(GYRO_ADDR, OUT_Z_AXIS_H);
        Z_AXIS_G = convert_raw(Z_H_G, Z_L_G);
     
        // If you want to print value
        printf("%d %d %d \r\n", X_AXIS_G, Y_AXIS_G, Z_AXIS_G);
        CyDelay(50);
    }
}   
//*/


/*//magnetometer//
int main()
{
    CyGlobalIntEnable; 
    UART_1_Start();
  
    I2C_Start();
   
    uint8 X_L_M, X_H_M, Y_L_M, Y_H_M, Z_L_M, Z_H_M;
    int16 X_AXIS, Y_AXIS, Z_AXIS;
    
    I2C_write(GYRO_ADDR, GYRO_CTRL1_REG, 0x0F);             // set gyroscope into active mode
    I2C_write(GYRO_ADDR, GYRO_CTRL4_REG, 0x30);             // set full scale selection to 2000dps
    I2C_write(ACCEL_MAG_ADDR, ACCEL_CTRL1_REG, 0x37);           // set accelerometer & magnetometer into active mode
    I2C_write(ACCEL_MAG_ADDR, ACCEL_CTRL7_REG, 0x22);
    
    
    for(;;)
    {
        X_L_M = I2C_read(ACCEL_MAG_ADDR, OUT_X_L_M);
        X_H_M = I2C_read(ACCEL_MAG_ADDR, OUT_X_H_M);
        X_AXIS = convert_raw(X_L_M, X_H_M);
        
        Y_L_M = I2C_read(ACCEL_MAG_ADDR, OUT_Y_L_M);
        Y_H_M = I2C_read(ACCEL_MAG_ADDR, OUT_Y_H_M);
        Y_AXIS = convert_raw(Y_L_M, Y_H_M);
        
        Z_L_M = I2C_read(ACCEL_MAG_ADDR, OUT_Z_L_M);
        Z_H_M = I2C_read(ACCEL_MAG_ADDR, OUT_Z_H_M);
        Z_AXIS = convert_raw(Z_L_M, Z_H_M);
        
        heading(X_AXIS, Y_AXIS);
        printf("MAGNET: %d %d %d %d %d %d \r\n", X_L_M, X_H_M, Y_L_M, Y_H_M, Z_L_M, Z_H_M);
        printf("%d %d %d \r\n", X_AXIS,Y_AXIS, Z_AXIS);
        CyDelay(50);      
    }
}   
//*/


/*


    <p>
    <B>Sensors</B><br>
    &nbsp;Included: <br>
        &nbsp;&nbsp;&nbsp;&nbsp;LSM303D: Accelerometer & Magnetometer<br>
        &nbsp;&nbsp;&nbsp;&nbsp;L3GD20H: Gyroscope<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Reflectance sensor<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Motors
    &nbsp;Wii nunchuck<br>
    &nbsp;TSOP-2236: IR Receiver<br>
    &nbsp;HC-SR04: Ultrasonic sensor<br>
    &nbsp;APDS-9301: Ambient light sensor<br>
    &nbsp;IR LED <br><br><br>
    </p>
    
    <p>
    <B>Communication</B><br>
    I2C, UART, Serial<br>
    </p>

*/


#if 0
int rread(void)
{
    SC0_SetDriveMode(PIN_DM_STRONG);
    SC0_Write(1);
    CyDelayUs(10);
    SC0_SetDriveMode(PIN_DM_DIG_HIZ);
    Timer_1_Start();
    uint16_t start = Timer_1_ReadCounter();
    uint16_t end = 0;
    while(!(Timer_1_ReadStatusRegister() & Timer_1_STATUS_TC)) {
        if(SC0_Read() == 0 && end == 0) {
            end = Timer_1_ReadCounter();
        }
    }
    Timer_1_Stop();
    
    return (start - end);
}
#endif

/* Don't remove the functions below */
int _write(int file, char *ptr, int len)
{
    (void)file; /* Parameter is not used, suppress unused argument warning */
	int n;
	for(n = 0; n < len; n++) {
        if(*ptr == '\n') UART_1_PutChar('\r');
		UART_1_PutChar(*ptr++);
	}
	return len;
}

int _read (int file, char *ptr, int count)
{
    int chs = 0;
    char ch;
 
    (void)file; /* Parameter is not used, suppress unused argument warning */
    while(count > 0) {
        ch = UART_1_GetChar();
        if(ch != 0) {
            UART_1_PutChar(ch);
            chs++;
            if(ch == '\r') {
                ch = '\n';
                UART_1_PutChar(ch);
            }
            *ptr++ = ch;
            count--;
            if(ch == '\n') break;
        }
    }
    return chs;
}
/* [] END OF FILE */
