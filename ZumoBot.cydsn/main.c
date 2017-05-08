/**
* @mainpage ZumoBot Project
* @brief    PROJECT MOCHA -- race version of the robot project.
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
#include "Reflectance.h"
#include "IR.h"

// Declare used functions.
int rread(void);
int get_IR();
void motor_start();
void motor_forward(uint8 speed,uint32 delay);
void motor_turn(uint8 l_speed, uint8 r_speed, uint32 delay);
void reflectance_set_threshold(uint16_t l3, uint16_t l1, uint16_t r1, uint16_t r3);
void Measure_Voltage();
void Custom_forward(uint8 speed);
void Turn(uint32 turn, int dir_flag);

int main()
{
    // ==================== CRUCIAL DEFINITIONS ======================================= //
    
    // (Maximum) movement speed of the robot.
    uint8 speed = 240;
    
    // Turn value for the motors to use, and its components (for calculating the final value).
    uint32 turn = 0;
    uint32 turnComp_1 = 0;
    uint32 turnComp_2 = 0;
    
    // Reflectance thresholds (determined experimentally) for use in different movement behaviours.
    int black_threshold_l3 = 21000; // actual line edge value: somewhere betwen 20 000 - 21 000.
    int black_threshold_l1 = 18000; // 'sure bet' working value: 17 500 // actual line edge value: ~16 000 // 18 000
    int black_threshold_r1 = 22500; // 'sure bet' working value: 22 000 // actual line edge value: ~18 000 // 22 500
    int black_threshold_r3 = 21000; // actual line edge value: somewhere between 20 000 - 21 500.
        
    //int white_threshold_l3 = 5793;
    int white_threshold_l1 = 4500;
    int white_threshold_r1 = 4522;
    //int white_threshold_r3 = 6293;
    
    // Maximum amounts of difference in measured blackness value (used in various operations).
    int maxDiff_l1 = black_threshold_l1 - white_threshold_l1;
    int maxDiff_r1 = black_threshold_r1 - white_threshold_r1;
    
    // ==================== OTHER STUFFS ======================================= //
    
    // 'Time counter' for the voltage measurement interval.
    int cycles = 0;
    
    // Needed for using the button to start the robot's movement routine.
    int buttonPress = 0;
   
    // Direction flag for correct turning behaviour. '0' = 'left turn', '1' = 'right turn'.
    int dir_flag = 0;
    
    // For counting blackLines (to stop the robot at race end).
    int countLines = 0;
    int inBlack = 0;
    int exitMainLoop = 0;
    
    // Needed for stopping at the first black line and waiting for IR signal.
    int firstStop_flag = 0;

    // Needed for determining turn behaviour.
    int digital = 0;
    int maxDiff = 0;
    int black_threshold = 0;
    float blackness_1 = 23999;
    float blackness_2 = 23999;
    float blackDiff = 0.0;
    float diffs[3] = {0, 0, 0};
    float diff_ave = 0;
    float diff_prev_ave = 0;
    float diff_norm = 0;
    float norm_blackness_2 = 0;
    
    // Needed for storing remote controller IR signal.
    unsigned int IR_val; 
    
    uint8 button; // make button exist
    
    // ==================== ACTUAL START ======================================= //
 
    // Initialize various starting thingies
    struct sensors_ ref;
    struct sensors_ dig;
    CyGlobalIntEnable; 
    UART_1_Start();
    ADC_Battery_Start();   
    Ultra_Start();
    
    Measure_Voltage(); // measure battery voltage at robot start
    
    motor_start(); // start the motor
    motor_forward(0, 0); // stop the motor at robot start, as it seems to run at a low speed if simply started up
    
    // Initialize IR sensors.
    sensor_isr_StartEx(sensor_isr_handler); 
    reflectance_set_threshold(black_threshold_l3, black_threshold_l1, black_threshold_r1, black_threshold_r3);  
    reflectance_start();
    IR_led_Write(1);

    BatteryLed_Write(0); // Switch led off 
    
    
    // Wait for button press to start the robot's movement routine.
    while (buttonPress == 0) 
    {
        button = SW1_Read();
        
        if (button == 0) 
        {
            buttonPress = 1;
            CyDelay(10);
        }
    }
    
    #if (1)
        
    while (firstStop_flag == 0)
    {
    
        // Go forward at low speed until meeting the first horizontal black line.
        // Then wait for the IR signal to proceed.
        Custom_forward(speed/2.5);
        
        reflectance_read(&ref);
        reflectance_digital(&dig);
        
        if (dig.l3 == 0 && dig.r3 == 0)
        {
            inBlack = 1;
        }
        
        reflectance_read(&ref);
        reflectance_digital(&dig);
         
        if (inBlack == 1 && dig.l3 == 1 && dig.r3 == 1)
        {
            motor_forward(0,0);
            firstStop_flag = 1;
            inBlack = 0;
        }
        
        CyDelay(1);
    
    }
    
    
    // Wait for IR signal. Upon getting it, proceed forward at full speed for 0.4 seconds.
    IR_val = get_IR();
    if (IR_val) 
    {
        Custom_forward(speed);
        CyDelay(400); // This stops any interference with the stopping logic in the main movement loop, and gives a nice 'initial spurt'. Inelegant, but it works!
    }
    
    
    #endif
   
    // Giant loop to run the movement logic in.
    while(exitMainLoop == 0)
    {    

        // Line-following logic.
        // NOTE: due to the calibration of the motor speeds, 240 (255 - 15) is our current max speed!
        
        // Start going forward.
        Custom_forward(speed);
        
        // Read initial blackness value. (This only needs to be done once, hence it's outside the turn loop.)
        reflectance_read(&ref); // raw reflectance value ('blackness') from the sensor; 0 - 23 999
        reflectance_digital(&dig); // print out 0 or 1 according to results of reflectance period
        
        if (dig.l1 == 1) 
        {
            blackness_1 = ref.l1;
        } else if (dig.r1 == 1) {
            blackness_1 = ref.r1;   
        }   
        
        // If-switch between left and right sensor activation (affects turning logic directly below).
        if (dig.l1 == 1) 
        {
            dir_flag = 1; // left sensor activated, so we must turn to the RIGHT
            digital = dig.l1;
            black_threshold = black_threshold_l1;
            maxDiff = maxDiff_l1;
   
        } else if (dig.r1 == 1) {
        
            dir_flag = 0; // right sensor activated, so we must turn to the LEFT
            digital = dig.r1;
            black_threshold = black_threshold_r1;
            maxDiff = maxDiff_r1;
            
        }
        
        // When the measured blackness value drops below the designated blackness threshold, start the appropriate turning routine.
        if (digital == 1)
        { 

            // When the robot starts to veer off to the left or right, do a corrective turn in the opposite direction.
            do {
                
                // Obtain a second blackness value (1 millisecond later).
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
                               
                // Calculate the difference between the two measured values.
                blackDiff = blackness_1 - blackness_2;
                
                // Store the value of blackDiff in an array and move the other stored values +1 forward
                // (the stored values of blackDiff will be used to determine turning behavior directly below).
                // (NOTE: not done with 'for' because there's so few array members.)
                diffs[2] = diffs[1];
                diffs[1] = diffs[0];
                diffs[0] = blackDiff;
                
                // Use average of 3 diffs for determining behaviour; it should smooth out the rough edges.
                diff_ave = ( diffs[0] + diffs[1] + diffs[2]) / 3;
                
                // Needed for normalizing 'freak' blackness values (explained below).
                diff_prev_ave = (diffs[2] + diffs[1]) / 2;
                
                // Normalize diff_ave (needed for use in the exponential 2nd component of 'turn', to prevent potential 'freak' behaviour).
                diff_norm = diff_ave;
                if (diff_norm > 2000.0) { diff_norm = 2000.0; } // 2 000 seems like a reasonable limit, based on measured diff values
                
                // Normalize the value of blackness_2 (assigned to a new variable; original blackness_2 remains unmodified).
                // The normalized value is used instead of plain blackness_2 in the first component of 'turn', directly below.
                // This should ensure that random 'freak' sensor values, like 5 000 blackness when the last value was 18 000, will have limited effect
                // on the final value of 'turn'.
                norm_blackness_2 = blackness_2;
                               
                // If the latest positive difference in blackness values is more than 3000 AND the previous diff average (calculated WITHOUT the new big jump) was less
                // than 1000, normalize blackness value with the TOTAL diff average (INCLUDING the new big jump).
                // This should 'cancel out' almost the whole effect of the big jump, thus normalizing away any bad behaviour due to 'freak' values (speck of dust, etc).
                if ((blackDiff > 3000) && (fabsf(diff_prev_ave) < 1000) )
                {
                    norm_blackness_2 -= diff_ave;  // seems like a good way to normalize, as the greater the jump will be, the greater also the normalization.
                    // ... If a fluke occurs anyway, it should be a minor one, as the big jump inevitably dominates the value of diff_ave. 
                    // Otoh, whether 3000 diff is a good limit for triggering the normalization is very much
                    // up for debate; experiments will be needed.
                }
                
                // When moving away from the line (or close enough to moving away), set turn to zero (our abortive attempts at 
                // various 'corrective twitch' behaviours were finally laid to a well-deserved rest).
                if (diff_norm < -100) 
                { 
                    turn = 0; 
                    
                } else if (diff_norm >= -100) {             
                 
                    // 'Base' turn factor. Calibrated with black_threshold to obtain equivalent behavior for the right and left-hand sides.
                    // Produces exponentially larger values with smaller input norm_blackness_2 values. Thus the further away from the line 
                    // we are, the greater the amount of turn (it's a very mild exponent atm; may be increased if needed).
                    // '9 000' is simply a ball-park constant, to set the component within acceptable limits.
                    turnComp_1 = 9000 * ( black_threshold / powf(norm_blackness_2,1.5) );
                                  
                    // Second turn factor. Used multiplicatively with the first one in order to obtain the second half of the final turn 
                    // equation (1.+ 1.*2.). The larger the measured (and normalized) difference in blackness values, the larger turn becomes, 
                    // again exponentially, but this time with a more severe exponent (because diff should dominate turn behaviour).
                    // Calibrated with maxDiff to obtain equivalent behaviour for the left and right-hand sides.
                    turnComp_2 = 0.095 * ( powf( fabsf(diff_norm), 1.83 ) / maxDiff ); 
                    // NOTE: using the absolute value causes 'wrong' turn behaviour between diff -100-0, but it's worth it to 
                    // eliminate possible 'turbulence' (due to measurement inaccuracy) on the borderline between negative and positive diff values.
                    
                    // The final turn equation combines the raw flatness of the first turn factor with the refinement that comes from multiplying it with the second one.
                    // The robot would not turn tight corners reliably without having a non-zero turn factor at all times, so the first element was made additive;
                    // minimum turn is about 60 atm, and this seems to give nice, smooth results.
                    turn = turnComp_1 + turnComp_1*turnComp_2;
                    
                    // NOTE: moved resulting turn values in an excel file (NOT the one in Google Drive!)!
                    
                    // Ideally, the second component should always dominate, since it's the most critical tool for detecting steep turns and reacting to them.
        
                    // Check for egregious values of 'turn' and correct them if found.
                    if (turn > 240)
                    {
                        turn = 240;                 
                    }
                }
                                                                                                       
                // Execute the turn (right turn if the left sensor activated, left turn if the right one activated).
                Turn(turn, dir_flag);
                             
                // Store the value of the second blackness measurement to the variable for the first.
                // As the loop continues, blackness_2's value is stored in blackness_1 and then blackness_2 gets a new, measured value; etc.
                blackness_1 = blackness_2;
              
                // Line recognition logic (needed for stopping).
                if (dig.l3 == 0 && dig.r3 == 0){
                    inBlack = 1;
                }
                
                if (inBlack == 1 && dig.r3 == 1 && dig.l3 == 1){
                    ++countLines;
                    inBlack = 0;
                }
                
                // Changed value from '2' to '1', to account for the added IR signal logic.
                if (countLines == 1 && dig.r3 == 0 && dig.l3 == 0){
                    motor_forward(0,0);
                    exitMainLoop = 1;
                }
                
                CyDelay(1);
                      
            } while (digital == 1);
                                                
            // Since the turn has ended, reset the stored diff values back to zero, so the next turn can have a fresh start with new values.
            int i;
            for (i = 0; i < 3; i++)
            {
                diffs[i] = 0;
            }
          
            // For added safety, set turn to zero... Should not be necessary, but you never know.
            turn = 0;                                         
        } 
          
        // Line recognition logic (needed for stopping).
        if (dig.l3 == 0 && dig.r3 == 0){
            inBlack = 1;
        }
        
        if (inBlack == 1 && dig.r3 == 1 && dig.l3 == 1){
            ++countLines;
            inBlack = 0;
        }
        
        // Changed value from '2' to '1', to account for the added IR signal logic.
        if (countLines == 1 && dig.r3 == 0 && dig.l3 == 0){
            motor_forward(0,0);
            exitMainLoop = 1;
        }  
        
        /*
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
        */
        
        CyDelay(1); 
        
    }  
    
    // empty loop to end with
    while(1) {}
    
}
    
// ===================================================================================================================== //
// ===================================================================================================================== //

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
