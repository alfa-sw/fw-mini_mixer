#ifndef USERPARAMS_H       //avoid double inclusion
#define USERPARAMS_H

/********************************************************************
* © 2009 Microchip Technology Inc.
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
*******************************************************************************/
    
/********************** Motor parameters *******************************/

    #define UNIPOLAR                0               //set to 0 for Bipolar motor configuration
    #if defined(NANOTEC_MOTOR) 
    /*Cx_COLOR*/ 	   #define MOTOR_R                  2.27   //2.0        //(Ohm) motor resistance in the configuration in which it is conected
    /*Cx_COLOR*/     #define MOTOR_L                 0.0036          //(H)	motor  inductance
    /*Cx_COLOR*/     #define DC_BUS                  24.0            //(V) 	board power supply
    /*Cx_COLOR*/     #define RATED_MOTOR_CURRENT     1.34   //1.9        //(Apk) 	maximum desired motor current (rated motor current)
    #elif defined(BASE_ACTUATOR) 
    // /* Bx_BASE*/ 		 #define MOTOR_R                0.8//1.07 //1.07 = 0.8 + 0.27      //(Ohm) motor resistance in the configuration in which it is conected
    // /* Bx_BASE*/     #define MOTOR_L                 0.0076// 0.0038           //(H)	motor  inductance
    // /* Bx_BASE*/     #define DC_BUS                  48.0            //(V) 	board power supply
    // /* Bx_BASE*/ 	   #define RATED_MOTOR_CURRENT     2.5  //2.0 (Apk) 	maximum desired motor current (rated motor current)
     /* Bx_BASE*/ 		  #define MOTOR_R                 0.63//2.4//6.08       //(Ohm) motor resistance in the configuration in which it is conected
     /* Bx_BASE*/     #define MOTOR_L                 0.0024           //(H)	motor  inductance
     /* Bx_BASE*/     #define DC_BUS                  48.0            //(V) 	board power supply
     /* Bx_BASE*/ 	    #define RATED_MOTOR_CURRENT     2.5//1.6 /*75% valore nominale*///0.85*1.41    //(Apk) 	maximum desired motor current (rated motor current)
    #elif defined(MOVE_X_AXIS)
    // #if defined(XY_BANCO_MEDICON)
    // /* MOVE_X_AXIS*/ 		 #define MOTOR_R                 2.4             //(Ohm) motor resistance in the configuration in which it is conected
    // /* MOVE_X_AXIS*/     #define MOTOR_L                 0.0024           //(H)	motor  inductance
    // /* MOVE_X_AXIS*/     #define DC_BUS                  48.0            //(V) 	board power supply
    // /* MOVE_X_AXIS*/ 	   #define RATED_MOTOR_CURRENT    1.6             /*75% valore nominale*///0.85*1.41   
    // #else
    /* MOVE_X_AXIS*/ 		 #define MOTOR_R                 0.63//2.4//6.08       //(Ohm) motor resistance in the configuration in which it is conected
    /* MOVE_X_AXIS*/     #define MOTOR_L                 0.0024           //(H)	motor  inductance
    /* MOVE_X_AXIS*/     #define DC_BUS                  48.0            //(V) 	board power supply
    /* MOVE_X_AXIS*/ 	   #define RATED_MOTOR_CURRENT     2.5//1.6 /*75% valore nominale*///0.85*1.41    //(Apk) 	maximum desired motor current (rated motor current)
    // #endif
    #elif defined(MOVE_Y_AXIS)
    /*MOVE_Y_AXIS*/ 		 #define MOTOR_R                 2.3           //(Ohm) motor resistance in the configuration in which it is conected
    /*MOVE_Y_AXIS*/     #define MOTOR_L                 0.004          //(H)	motor  inductance
    /*MOVE_Y_AXIS*/     #define DC_BUS                  24.0            //(V) 	board power supply
    /*MOVE_Y_AXIS*/     #define RATED_MOTOR_CURRENT     1.7        //2.4    //(Apk) 	maximum desired motor current (rated motor current)
    #elif defined(STORAGE_CONTAINER_ACTUATOR)
    /*Cx_COLOR*/ 		 #define MOTOR_R                 2.9           //(Ohm) motor resistance in the configuration in which it is conected
    /*Cx_COLOR*/     #define MOTOR_L                 0.0034          //(H)	motor  inductance
    /*Cx_COLOR*/     #define DC_BUS                  24.0            //(V) 	board power supply
    /*Cx_COLOR*/     #define RATED_MOTOR_CURRENT     1.2        //2.4    //(Apk) 	maximum desired motor current (rated motor current)
    #elif defined(AUTOCAP_ACTUATOR)
    /*AUTOCAP*/ 		  #define MOTOR_R                 2           //(Ohm) motor resistance in the configuration in which it is conected
    /*AUTOCAP*/     #define MOTOR_L                 0.0036          //(H)	motor  inductance
    /*AUTOCAP*/     #define DC_BUS                  24.0            //(V) 	board power supply
    /*AUTOCAP*/     #define RATED_MOTOR_CURRENT     1.4       //1.4    //(Apk) 	maximum desired motor current (rated motor current)
    #else
    /*Cx_COLOR*/ 		 #define MOTOR_R                 2.3           //(Ohm) motor resistance in the configuration in which it is conected
    /*Cx_COLOR*/     #define MOTOR_L                 0.004          //(H)	motor  inductance
    /*Cx_COLOR*/     #define DC_BUS                  48.0            //(V) 	board power supply
    /*Cx_COLOR*/     #define RATED_MOTOR_CURRENT     1.7        //2.4    //(Apk) 	maximum desired motor current (rated motor current)
    #endif
  
		#define VAVV_COEFF_MOD 					100L 
    #define CORRECTION_COEFF 				100L 
		#define MOTOR_R_OHM_x_100       MOTOR_R*100L
	
		#define MOTOR_L_H_x_10000       MOTOR_L*10000L
		#define RATED_MOTOR_CURRENT_A_x_100     RATED_MOTOR_CURRENT*100L             //(A) 	maximum desired motor current (rated motor current)
		
		#define NUM_STEP_PER_REVOLUTION 200L
		
		#define ACCELERATION_MAX        10000           //maximum acceleration rate in full steps/s^2
    #define DECELERATION_MAX        10000           //maximum deceleration rate in full steps/s^2
                                                    //minimum value is 125;
 
/******************** Software Parameters ****************************/  

    #define SPEED_CONTROL_RATE	    2.0             //rate of speed controller in ms

    //Control Mode
    #define FIXED_VOLTAGE           0               //in this mode a fixed voltage is applied to the motor windings
    #define FIXED_CURRENT           1               //in this mode the current amplitude is fixed
    
    //Decay driving mode
    #define FIXED_DECAY             0               //in this mode only one current decay mode is used
    #define ALTERNATE_DECAY         1               //in this mode the current decay is alternated between two modes
    
    #define MAX_STEP_RESOLUTION     ST_1_64STEP     //minimum step size

    //Speed sources
    #define POT_REF_SPEED           0               //speed controller reference is set by POT
    #define DMCI_REF_SPEED          1               //speed controller reference is set by DMCI
    
    //PI controller defines
    #define RISE_TIME_USEC                     70L 
    #define CONV_USEC_SEC                      1000000L
    #define PI_GAIN                             (12* CONV_USEC_SEC)/(RISE_TIME_USEC*DC_BUS)      //7143L            //PI gain value
    
 #if defined(BASE_ACTUATOR)  
    #define PI_ANTI_WINDUP_GAIN1  	  10000//500            //Anti-Windup gain value at low speeds
    #define PI_ANTI_WINDUP_GAIN2  	  17000            //Anti-Windup gain at high speeds
 #else
    #define PI_ANTI_WINDUP_GAIN1  	  10000//500            //Anti-Windup gain value at low speeds
    #define PI_ANTI_WINDUP_GAIN2  	  17000            //Anti-Windup gain at high speeds
 #endif
    #define PI_ANTI_WINDUP_SPEED    2800            //speed at which the anti-windup gain is changed; corresponds to 336RPM
                                                    //this speed is the point where steps begin to dissapear and the current has a sinewave shape
                                                    //the value depends on motor inductance and driving voltage
    #define PWM_MINIMUM_DUTY        7.0             //minimum duty cycle used in closed loop PI control in percent
    #define POS_FIXED_GAIN          (long)(7*32768) //position controller gain
                                                    //reducing the gain results in slower deceleration during position control
                                                    //increasing the gain results in position overshoots and motor oscillation around teh desired position
/******************** System Parameters *******************************/  
   
    #define PWM_FCY_SET             40		        //value in KHz
    #define	BUTTON_FILTER           50              //value used for debouncing
    #define MIPS                    40              //do not change
    #define DEAD_TIME				500				//value in ns; do not change
    
/******************* Hardware Parameters ******************************/

    /* Fixed parameters based on hardware (dsPICDEM MCSM development board)- Do not change!  */    
    #define BUTTON_PIN       		PORTBbits.RB7   //S1 button port pin         
    #define ADC_CURRENT_RANGE       3.3             //Value in Amps; maximum current the ADC input can read
    #define DC_BUS_RESISTOR         28.0            //Value in KOhms; 1:28 is the voltage divider scale in HW
    #define ADC_VOLTAGE             3.3             //value in Volts; voltage used by the ADC as reference 
    #define BOARD_R             	(0.224+DEAD_TIME*PWM_FCY_SET*DC_BUS/1000000) //Value in Ohms; board resistance on the drive path = twice the mosfet ON resistance plus shunt value plus dead-time
    	#define BOARD_R_OHM_x_100   BOARD_R*100L
		#define MOTOR_R_TOT_OHM_x_100  BOARD_R_OHM_x_100  + MOTOR_R_OHM_x_100
/***************** Resultant Parameters - DO NOT TOUCH!! ***************/  
    
    #if (UNIPOLAR==0)
    #define BIPOLAR
    #else
    #undef BIPOLAR
    #endif


    #define TABLE_SIZE	            256             //sinewave look-up table size
    #define TABLE_SIZE_MUL2	        TABLE_SIZE*2    //table size *2
    #define TABLE_SIZE_MUL3	        TABLE_SIZE*3    //table size *3
    #define TABLE_SIZE_MUL4	        TABLE_SIZE*4    //table size *4
    
    //Full Step Modes
    #define FULLSTEP_TWO_PHASE_ON   1               //possible values for stepSize variable               
    #define FULLSTEP_WAVE_DRIVE     0
    
    //Step sizes
    #define ST_FULLSTEP             0               //possible values for stepSize variable
    #define ST_HALFSTEP             1
    #define ST_1_4STEP              2
    #define ST_1_8STEP              3
    #define ST_1_16STEP             4
    #define ST_1_32STEP             5
    #define ST_1_64STEP             6
    #define ST_1_128STEP            7
    #define ST_1_256STEP            8
    
    //define bipolar decay modes                
                                                    //drive = high mosfet + opposite low mosfet
    #define D_FAST                  0               //fast decay = all MOSFETS off
    #define D_SLOW_L_DIODE          1               //slow decay = low diode + opposite low mosfet
    #define D_SLOW_H_DIODE          2               //slow decay = high mosfet + opposite high diode
    #define D_SLOW_L_MOSFET         3               //slow decay = low mosfet + opposite low mosfet
    #define D_SLOW_H_MOSFET         4               //slow decay = high mosfet + opposite high mosfet
    #define D_REVERSE               5               //reverse = low mosfet + opposite high mosfet
   
    //define unipolar decay modes    
    #define D_SLOW                  1               //slow decay for unipolar mode
    
    #define PWM_ACQ_DELAY           30              //delay from PWM interrupt/reload to ADC acquistion start; measured in Tosc periods
                                                    //before ADC acquistion, the delay allows the current to rise through the RC filter
                                                    //delay is 750ns for PWM_ACQ_DELAY  == 30
    //State machine defines
    #define STATE_OFF               1
    #define STATE_RUN               2
    #define STATE_INIT              0
   
    
    //motor and winding direction - do not change - these values are used in different mathematical formulae
    #define FORWARD                 0               //winding forward direction; used by setDir variables
    #define REVERSE                 1               //winding reverse direction; used by setDir variables
    #define MOTOR_FORWARD           1               //rotor is rotating in forward direction
    #define MOTOR_REVERSE          -1               //rotor is rotating in reverse direction
    
    //speed or position control modes
    #define SPEED_CONTROL           0               //speed control mode
    #define POSITION_CONTROL        1               //position control mode
    
    //PI control ON or OFF
    #define ON                      1
    #define OFF                     0
   
    
    #define SYSTEM_CLOCK                (0.000001/MIPS)                             //system clock in ns
    #define PWM_FCY                     (int)((0.5*MIPS*1000/PWM_FCY_SET)-1)        //PWM frequency value to be written in PxTPER
    #define PWM_MAX                     (int)((PWM_FCY+1)*2+1)                        //value corresponding to 100% duty cycle
		#define MINIMUM_DUTY                (int)(PWM_MINIMUM_DUTY * PWM_MAX / 100)     //PWM_MINIMUM_DUTY converted to PWM duty cycle values 

    //1023*32 is the scaling for 10bit ADC left aligned
    #define OPEN_LOOP_CURRENT           (int)((double)RATED_MOTOR_CURRENT*1023*32/ ADC_CURRENT_RANGE)                               
    #define CLOSED_LOOP_CURRENT         (int)(RATED_MOTOR_CURRENT*1023*32/ ADC_CURRENT_RANGE)
    #define CLOSED_LOOP_CURRENT_REDUCTED         (int)(RATED_MOTOR_CURRENT*1023*32*COEFF_REDUCTION_PCT/ ADC_CURRENT_RANGE/100)
		
		#define VAVV_COEFF 100L 

		#define VAVV_COEFF1 1000L 
    #define CONV_SPEED_SLIDER 500L
    #define CONV_MIN_SEC 60
	  
    #ifdef NANOTEC_MOTOR
    #define COEFF_REDUCTION_PCT  10L 
    #else
    #define COEFF_REDUCTION_PCT  10L
    #endif

    
    //convert to duty Cycle values; this duty cycle generates the motor rated current in open loop voltage control mode
    #define OPEN_LOOP_VOLTAGE           (int)(RATED_MOTOR_CURRENT*(MOTOR_R+BOARD_R)*PWM_MAX*COEFF_REDUCTION_PCT/DC_BUS/100)   
    
    //calculate acceleration and deceleration in step values based on the controller rate, system clock and speed scale used in the controller
    #define ACCELERATION_STEPS          (int)(0.001*SPEED_CONTROL_RATE*SYSTEM_CLOCK*ACCELERATION_MAX*TABLE_SIZE*SPEED_PRE_SCALE/(1<<SPEED_POST_SCALE))
    #define DECELERATION_STEPS          (int)(0.001*SPEED_CONTROL_RATE*SYSTEM_CLOCK*DECELERATION_MAX*TABLE_SIZE*SPEED_PRE_SCALE/(1<<SPEED_POST_SCALE))
    
    #define SPEED_CONTROL_RATE_TIMER    (int)(0.001*SPEED_CONTROL_RATE/SYSTEM_CLOCK/8)      //rate of speed controller in system clock cycles controlled by Timer1
                                                                                            //Timer1 input clock is prescaled with a factor of 8
    
    //Speed(step/s) = 1/ (SPEED_PRE_SCALE / speedOut * TABLE_SIZE * TIMER_PERIOD / 2^SPEED_POST_SCALE)
    //an increment of 1 in speedOut results in an increase of 0.4steps/s (0.12RPM) in the motor speed
    #define SPEED_PRE_SCALE             (long)(400000000)  //see the formula above
    #define SPEED_POST_SCALE            10                 //see the formula above
    #define MAXIMUM_SPEED               10000               // 1200 RPM 
    #define MINIMUM_SPEED               49                 //below this POT or RTDM value the motor stops;
                                                           //(TABLE_SIZE * SPEED_PRE_SCALE/MINIMUM_SPEED) value should be smaller than 2^31)
                                                           //reducing this value without changing the SPEED_PRE_SCALE will result in an overflow
    #define POS_OVERFLOW_CHECK          (long)(((double)32768/POS_FIXED_GAIN)*32768/DECELERATION_STEPS)   //used to check if the position gain formula would overflow
    //PI controller parameters based on motor parameters and controller Gain
    #define MOTOR_T                     (0.001/PWM_FCY_SET)             //PI controller rate
    #define PI_PARAM_1                  ((MOTOR_L+MOTOR_T*MOTOR_R/2)*ADC_CURRENT_RANGE*32768/DC_BUS_RESISTOR /ADC_VOLTAGE)
    #define PI_PARAM_2                  ((MOTOR_L-MOTOR_T*MOTOR_R/2)*ADC_CURRENT_RANGE*32768/DC_BUS_RESISTOR /ADC_VOLTAGE)
                                        
   
    #define PI_GAIN_PARAM_COEFF         131072L //8192L//2^17
    //80V = max voltage on Board
  #if defined(BASE_ACTUATOR)    
    #define PI_GAIN_PARAM_1             (int) (2000)//(PI_GAIN*PI_PARAM_1*1000/PI_GAIN_PARAM_COEFF)
    #define PI_GAIN_PARAM_2             (int) (1000)//(PI_GAIN*PI_PARAM_2*1000/PI_GAIN_PARAM_COEFF)
  #else
    #define PI_GAIN_PARAM_1             (int) (2000)//(PI_GAIN*PI_PARAM_1*1000/PI_GAIN_PARAM_COEFF)
    #define PI_GAIN_PARAM_2             (int) (1000)//(PI_GAIN*PI_PARAM_2*1000/PI_GAIN_PARAM_COEFF)
  #endif

    #define VOLTAGE                     (int)((DC_BUS/DC_BUS_RESISTOR/ADC_VOLTAGE)*32768)       //Drive voltege as read by the ADC                                        
                                        //80V corresponds to 28369   -> 24V corresponds to 8511: (24/28/3.3)*32768
    
    #define DC_2_PWM                    (int)((PWM_MAX*32768)/VOLTAGE)    //convert voltage to PWM duty cycle
  
#endif
