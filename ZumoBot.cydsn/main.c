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
#include <stdlib.h>
#include <math.h>
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


// Declare used methods
int rread(void);
void motor_start();
void motor_forward(uint8 speed,uint32 delay);
void Custom_forward(uint8 speed);
void motor_turn(uint8 l_speed, uint8 r_speed, uint32 delay);
void reflectance_set_threshold(uint16_t l3, uint16_t l1, uint16_t r1, uint16_t r3);
void Measure_Voltage();
void Turn(uint8 turn, int dir_flag, float diff_ave);


int main()
{
    // 'Time counter' for the voltage measurement interval
    int cycles = 0;
    
    // Needed for using the button to start the robot's movement routine
    int exit = 0;
   
    // direction flag for correct turning behaviour. '1' = 'left', '2' = 'right'.
    int dir_flag = 0;
    
    //For counting blackLines (to stop the robot at race end)
    int countLines = 0;
    int inBlack = 0;
    int exitMainLoop = 0;
    
    // Turn value for the motors to use.
    uint8 turn = 0;
    
    // (Maximum) movement speed of the robot.
    uint8 speed = 240;
    
    // Reflectance thresholds (determined experimentally) for use in different movement behaviours.
    int black_threshold_l3 = 21000;
    int black_threshold_l1 = 18000; // 'sure bet' working value: 17 500 // actual line edge value: ~16 000
    int black_threshold_r1 = 22600; // 'sure bet' working value: 22 000 // actual line edge value: ~18 000 
    int black_threshold_r3 = 21500;
        
    int white_threshold_l3 = 5793;
    int white_threshold_l1 = 4500;
    int white_threshold_r1 = 4522;
    int white_threshold_r3 = 6293;
    
    int maxDiff_l1 = black_threshold_l1 - white_threshold_l1;
    int maxDiff_r1 = black_threshold_r1 - white_threshold_r1;
   
    // Needed for determining turn behaviour
    int digital = 0;
    int maxDiff = 0;
    int black_threshold = 0;
    float blackness_1 = 23999;
    float blackness_2 = 23999;
    float blackDiff = 0.0;
    float diffs[3] = {0, 0, 0};
    float diff_ave = 0;
    int i;
    float diff_pos = 0;
    float norm_blackness_2 = 0;
 
    // Initialize various starting thingies
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

    BatteryLed_Write(0); // Switch led off 
    
    uint8 button; // make button exist
    
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
    
    // Start going forward.
    Custom_forward(speed);
    
    // Giant loop to run the movement logic in.
    while(exitMainLoop == 0)
    {    
        
        reflectance_read(&ref); // raw reflectance value ('blackness') from the sensor; 0 - 23 999
        //printf("%d %d %d %d \r\n", ref.l3, ref.l1, ref.r1, ref.r3);       //print out each period of reflectance sensors
        reflectance_digital(&dig);      //print out 0 or 1 according to results of reflectance period
        //printf("%d %d %d %d \r\n", dig.l3, dig.l1, dig.r1, dig.r3);        //print out 0 or 1 according to results of reflectance period
        //CyDelay(500); // comment this delay out when doing movement tests / racing !!!
        
        
        
        // Line-following logic.
        // NOTE: due to the calibration of the motor speeds, 240 (255 - 15) is our current max speed!
        
        // If-switch between left and right sensor activation (affects turning logic directly below).
        if (dig.l1 == 1) 
        {
            dir_flag = 1;
            digital = dig.l1;
            
            blackness_1 = ref.l1;
            CyDelay(1);
            
            black_threshold = black_threshold_l1;
            maxDiff = maxDiff_l1;
   
        } else if (dig.r1 == 1) {
        
            dir_flag = 2;
            digital = dig.r1;
            
            blackness_1 = ref.r1;
            CyDelay(1);
            
            black_threshold = black_threshold_r1;
            maxDiff = maxDiff_r1;
            
        }
         
        // When the measured blackness value drops below the threshold, start the appropriate turning routine.
        // Added a further check to weed out the effect of random 'freak' values: the absolute blackness difference between
        // the sensors must be greater than 4 000 (the 'natural' difference is about 2 000, so this equates to ~2 000 units of 'actual' difference).
        if (digital == 1 /*&& ( abs(ref.l1 - ref.r1) > 3400) */ )
        { 

            // When the robot starts to veer off to the left or right, do a corrective turn in the opposite direction.
            do {
                
                // Obtain a second ref value (1 millisecond later).
                reflectance_read(&ref);
                reflectance_digital(&dig); // needed to check if the turn should be ended
                
                // This if-härdelli is needed solely due to the way that structs work...
                if (dir_flag == 1) 
                {
                    digital = dig.l1;
                    blackness_2 = ref.l1;
                } else {
                    digital = dig.r1;
                    blackness_2 = ref.r1;
                }
                               
                // Calculate the difference between the two values.
                blackDiff = blackness_1 - blackness_2;
                
                // Store the value of blackDiff in an array and move the other stored values +1 forward
                // (the stored values of blackDiff will be used to determine turning behavior directly below)
                diffs[2] = diffs[1];
                diffs[1] = diffs[0];
                diffs[0] = blackDiff;
                
                // Use average of 3 diffs for determining behaviour. it should smooth out the rough edges.
                diff_ave = ( diffs[0] + diffs[1] + diffs[2]) / 3;
                
                // Normalize diff_ave (needed for use in the exponential 3rd component of 'turn', to prevent 'freak' or undefined behaviour).
                diff_pos = diff_ave;
                if (diff_pos < 0 ) { diff_pos *= -1.0; }
                
                if (diff_pos > 2000.0) { diff_pos = 2000.0; }
                
                // Normalize blackness_2 (assigned to a new variable; original blackness_2 remains unmodified).
                // The normalized value is used instead of plain blackness_2 in the first exponential component of 'turn', directly below.
                // This should ensure that random 'freak' sensor values, like 5 000 blackness when the last value was 18 000, have limited effect
                // on the middle component of 'turn'.
                norm_blackness_2 = blackness_2;
                if (norm_blackness_2 < 10000)
                {
                    norm_blackness_2 = 10000;
                }
                          
                            // NOTE: All left side values in the comments !!
                            /* NOTE2: Values will need adjustment (upwards)!!
                            
                            left black_threshold = 18 000
                            left maxDiff = black_threshold - white_threshold = 13 500
                           
                            For base turn:
                            18 000 (black_threshold) + 60 constant:
                            if blackness_2 = 20 000 ==> turn = 33
                            if blackness_2 = 18 000 ==> turn = 60
                            if blackness_2 = 16 000 ==> turn = 86
                            if blackness_2 = 14 000 ==> turn = 112
                            if blackness_2 = 12 000 ==> turn = 138
                            if blackness_2 = 10 000 ==> turn = 164
                
                            */
                            
                // 'Base' turn component. Depends linearly on the last measured blackness value, calibrated with the black_threshold + 60 (simply a ball-park constant,
                // to raise the value of turn globally.           
                turn = 60 + speed * ( (black_threshold - blackness_2) / black_threshold );
                
                // First exponential turn component; depends on the normalized blackness value (calibrated with the black_threshold) raised to the power of 2.
                // If the blackness is doubled, its effect on turn is reduced to 25 %. '57' is simply a ballpark coefficient, to fit the final value 
                // in an appropriate range.
                turn += 57 * powf((black_threshold / norm_blackness_2), 2.0); // max effect on turn is ~+180 (with a norm_blackness_2 <= 10000). with 20 000, turn += 45.
                // With 16 000 blackness (line edge value), turn += 72.
                // (The lower values might seem a bit low. They may be raised once the behaviour of the robot appears more normal.)
                
                // Second exponential turn component. Depends on the positive, normalized average of the last 3 measured blackDiffs, 
                // raised to the power of 1.3 and calibrated with the value of maxDiff. Again '225' is simply a ball-park coefficient.
                turn += 225 * ( powf(diff_pos, 1.3) / maxDiff );
                // 500 diff: +73 turn; 1000 diff: +130 turn; 2000 diff: +324 turn (=> 240);
                // NOTE: These values may be TOO large... Experiments are needed.
                
                // Ideally, the last component should always dominate, since it's the most critical tool for detecting steep turns and reacting to them.

                // Minimum final turn amount (with ~20 000 blackness and 0 diff) should be ~79 atm. (seems a bit large... at any rate, it cannot be any larger!) 
                // (Note that this value combination should actually be impossible to reach under most circumstances.)
                // With 17 999 blackness and zero diff: 117 final turn. (seems high..?)
                // With 16 000 blackness ('real' line edge) and zero diff: 158 final turn. (this otoh seems quite ok)
                // With 14 000 blackness (over the edge!) and zero diff: 206 final turn. (along with diff's effect, should result in max turn most of the time)
                
                // Check for egregious values of 'turn' and correct them if found.
                if (turn > 240)
                {
                    turn = 240;
                } else if (turn < 0) {
                    turn = 0;   
                }
                
                //printf("dir: %d, turn: %d, diff_temp: %f, blackness_2: %f \n", dir_flag, turn, diff_temp, blackness_2);
                                                                                                       
                // Execute the turn (right turn if the left sensor activated, left turn if the right one activated).
                // If diff_ave < 0, the turn is made in the opposite direction (moving towards center of line for a while => turns away from center).
                // Between 0 and -300, the opposite turn is milder (75 % of regular turn value), while with a diff_ave < -300, regular turn value is used.
                // NOTE: Further modification of the opposite turn's turn value (in the Turn method) may be necessary to achieve optimal results.
                Turn(turn, dir_flag, diff_ave);
                             
                // Store the value of the second blackness measurement to the variable for the first.
                // As the loop continues, blackness_2's value is stored in blackness_1 and then blackness_2 gets a new, measured value; etc.
                blackness_1 = blackness_2;
                
                CyDelay(1);
                         
            } while (digital == 1);
                                                
            // Since the turn has ended, reset the stored diff values back to zero, so the next turn can have a fresh start with new values.
            for (i = 0; i < 3; i++)
            {
                diffs[i] = 0;
            }
          
            // For added safety, set turn to zero... May not be necessary.
            turn = 0;
            dir_flag = 0; // for safety as well...
            // Since the turn has ended, continue forward at the designated speed.
            Custom_forward(speed);
                                          
        } 
        
        /* ENABLE FOR RACE !!!!!!!!!!!!!!!!
        //First let's put line recognition logic here. It may be necessary to have line regocnition logic also in loops for turnings.
        if (dig.l3 == 0 && dig.r3 == 0){
            inBlack = 1;
        }
        
        if (inBlack == 1 && dig.r3 == 1 && dig.l3 == 1){
            ++countLines;
            inBlack = 0;
        }
        
        //this must change (2=>1), when added moving to starting-line logic.
        if (countLines == 2 && dig.r3 == 0 && dig.l3 == 0){
            motor_forward(0,0);
            exitMainLoop = 1;
        }        
                        
        */
        
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
    }  
    
    //empty loop to end with
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
