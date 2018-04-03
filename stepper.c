/**/
/*============================================================================*/
/**
**      @file      STEPPER.C
**
**      @brief     stepper motor management
**
**      @version   Alfa Color tester
**/
/*============================================================================*/
/**/

/*===== INCLUSIONI ========================================================= */
#include "stepper.h"
#include "MACRO.H"
#include "tables.h"
#include "INPUT.H"
#include "INIT.h"
#include "timermg.H"
#include "color.h"
#include "ram.h"
#include <math.h>


/*====== MACRO LOCALI ====================================================== */
#define RAMP_IDLE 0
#define RAMP_UP 1
#define RAMP_DW 2
#define RAMP_PLATEAU 3
#define NUM_STEP_PER_REVOLUTION 200L
#define CONV_MIN_MSEC 60000L

/*====== TIPI LOCALI ======================================================== */
/*====== VARIABILI LOCALI =================================================== */ 
volatile int stepCount;         //micro-step counter
int stepSize;         /*step mode stepSize = 0 -> FullStep
                                            stepSize = 1 -> Half Step  
                                            stepSize = 2 -> 1/4 Step
                                            stepSize = 3 -> 1/8 Step
                                            stepSize = 4 -> 1/16 Step
                                            stepSize = 5 -> 1/32 Step
                                            stepSize = 6 -> 1/64 Step
                                            stepSize = 7 -> 1/128 Step
                                            stepSize = 8 -> 1/256 Step   */
int stepSizeCount;        //used to increment the microstepping counter stepCount in timer2/3 interrupt
                                        /*  stepSizeCount = 256 -> FullStep
                                            stepSizeCount = 128 -> Half Step  
                                            stepSizeCount = 64  -> 1/4 Step
                                            stepSizeCount = 32  -> 1/8 Step
                                            stepSizeCount = 16  -> 1/16 Step
                                            stepSizeCount = 8   -> 1/32 Step
                                            stepSizeCount = 4   -> 1/64 Step
                                            stepSizeCount = 2   -> 1/128 Step
                                            stepSizeCount = 1   -> 1/256 Step   */  
int fullStepMode;               //controls change from 'full step 2 phase on' and 'full step wave drive'
                                //stepSize is identical in these 2 cases so this flag is needed                                   
int stepSizeCopy;       //copy of stepSize; used to manage step changes from DMCI

int setDir1;          //winding 1 desired driving direction
int setDir2;          //winding 1 desired driving direction
int setDir1HW;                  //copy of setDir1; controls current measurement; reflects the actual hardware current direction through the shunt resistor
int setDir2HW;                  //copy of setDir1; controls current measurement; reflects the actual hardware current direction through the shunt resistor

int voltageOut1,voltageOut2;    //output voltage for each phase
int stepAmplitudeRef1;          //voltage reference for winding 1
int stepAmplitudeRef2;          //voltage reference for winding 2
char directionFlag;       //motor direction; +1 is p; -1 is counter clockwise (depends on motor connection; direction can be reversed  by swapping one motor coil wires eg: M1 with M2)
char directionFlagOld;
int current1, current2;     //measured current for each phase 
volatile int currentMaxAmplitude;        //maximum amplitude of the current of both windings combined over one sinewave period (stepCount from 0 to 1023)

long speedRef;                  //speed reference value input to the speed controller
long speedOut;                  //motor speed; speed controller output
long OldspeedOut;
long speed2Time;                //motor speed converted to timer2/3 periods
int dmciSpeed;                  //motor speed set by DMCI
int potSpeed;                   //motor speed set by POT
                                   
int accelerationRate;           //maximum accelerationRate rate measured in timer counts (Timer2)
int decelerationRate;           //maximum decelerationRate rate measured in timer counts (Timer2)

int ovdPwm1,ovdPwm2;            //temporary variable to hold PWM override values during calculations
int pwmOut1,pwmOut2;        //temporary variable to hold PWM duty cycle values during calculations
 
int dcBus;            //power stage DC voltage scaled to Q15; see description of VOLTAGE for scaling method

int state;            //State machine: current state
int stateCopy;                  //copy of state; used to manage state changes from DMCI
int buttonCounter;              //S1 push buttonCounter used for debouncing

int stepChangeDecayCounter1;    //Counter that controls the decay change period in alternate decay mode; for winding 1
int stepChangeDecayCounter2;    //Counter that controls the decay change period in alternate decay mode; for winding 2
unsigned char decay1,decay2;    //current decay modes for each winding; see decay modes in userparams.h for possible values
int baseDecay;                  //current decay mode used in FIXED_DECAY mode; initial value is set in InitStepper() function
int alternateDecay;             //current decay used in ALTERNATE_DECAY mode; initial value is set in InitStepper() function
int stepChangeDecayNumber1;     //number of PWM periods in which alternate decay is used; for winding 1
int stepChangeDecayNumber2;     //number of PWM periods in which alternate decay is used; for winding 2
int altDecayScale;              //scaling value used to calculate the decay change period

volatile int maxRefValue;                //current or voltage reference; this value scales the look-up table to the desired amplitude
               //current or voltage reference; this value scales the look-up table to the desired amplitude

volatile long position;                  //motor position calculated by counting steps
long Mirror_position;           //copy of position to read position outside Timer 3 interrupt
long positionRef;               //desired motor position
long posError;                  //posError = positionRef - position
unsigned char posReached;
int dmciPositionRef;            //int version of positionRef; set by DMCI
int dmicPosRefScale;            //scale dmciPositionRef to long; positionRef = dmciPositionRef*dmicPosRefScale
int dmciPositionRefCopy;        //copy of dmicPosRefScale; used to detect position reference changes from DMCI

int PI_ref1;                    //current reference for winding 1
long PI_err1;                   //current error for winding 1
int PI_out1;                    //PI controller output (voltage)
long PI_sum1;                   //PI integral accumulator
int PI_ref2;                    //current reference for winding 2
long PI_err2;                   //current error for winding 2
int PI_out2;                    //PI controller output (voltage)
long PI_sum2;                   //PI integral accumulator
int PI_antiWindUp;              //anti-windup value for the PI controller; changes with speed
char resonanceCompensation;     //flag used to compensate the motor resonance in full step modes;

long debug_sum1[N_DEBUG_SAMPLES];
int debug_out1[N_DEBUG_SAMPLES]; 
long debug_err1[N_DEBUG_SAMPLES];
long debug_currentError1[N_DEBUG_SAMPLES];
long debug_prop[N_DEBUG_SAMPLES];
long debug_windup[N_DEBUG_SAMPLES];

long debug_sum2[N_DEBUG_SAMPLES];
int debug_out2[N_DEBUG_SAMPLES]; 
long debug_err2[N_DEBUG_SAMPLES];
long debug_currentError2[N_DEBUG_SAMPLES];

unsigned char count1, count2;

volatile generalFlags uGF;               //general flags for application control
int maxSpeed;   //maximum Speed reference and temporary variable


/*Posizione assoluta corrente in full step*/
long curr_position;

unsigned char statusRamp;
unsigned short speed_ramp;
unsigned char  motion_ramp_direction;

/*Posizione assoluta in full step di inizio rampa di discesa*/
long start_ramp_dw_num_step;

unsigned long StepPositionCount;
int setDir1temp,setDir2temp;
long currentError1, currentError2 ;
long pi_param1, pi_param2;
/*====== COSTANTI LOCALI =================================================== */

/* ===== PROTOTIPI FUNZIONI LOCALI ========================================= */  

void StartStepper(unsigned short speed, unsigned char direction);


/*====== DEFINIZIONE FUNZIONI LOCALI ======================================= */
int SetMaxRefInit(int speed_slider)
/*
*//*=====================================================================*//**
**
**      @brief Set optimal duty cycle depending on speed selected
**
**      @param speed slider
**
**      @retval duty setting
**
**
*//*=====================================================================*//**
*/
{
int maxRef;//=OPEN_LOOP_VOLTAGE;

unsigned long speed; /*[giri/sec] *100*/

/*zeta=MOTOR_R+jomega*MOTOR_L*/
/*omega = 3,14*fstep/2*/
/*mod(Z) = SQRT(MOTOR_R^2+omega^2*MOTOR_L^2)*/
/*Vavv = (RATED_MOTOR_CURRENT * mod(Z)*/

/*1 speed_slider = 0.12 RPM = 0.002 giri/sec : speed giri/sec = speed_slider*0.0002 = speed_slider/500*/
if (speed_slider < 0)
  {
  speed_slider = -speed_slider;
  }
speed = (speed_slider*VAVV_COEFF)/CONV_SPEED_SLIDER; /*giri/sec * 100*/

freq_step = speed*NUM_STEP_PER_REVOLUTION;  /* step/sec * 100*/
omega =  314*freq_step/2; /*x 10^4*/
omega_l = omega*MOTOR_L_H_x_10000/TEN_6_POW; /*x 10^8*/

motor_r =MOTOR_R_TOT_OHM_x_100;
motor_r_quad = motor_r*motor_r; /* x 10^4*/ 
omega_l_quad = omega_l*omega_l;

zeta = sqrt(motor_r_quad + omega_l_quad);

Vavv = zeta*RATED_MOTOR_CURRENT_A_x_100; /**/

  
/*duty[%] = 1.41*CORRECTION_COEFF*Vavv/DC_BUS*/
duty = Vavv*VAVV_COEFF_MOD*CORRECTION_COEFF/(unsigned long)(DC_BUS*TEN_4_POW); 
/*value on PWM register*/
value = PWM_MAX;
value =value*duty/TEN_4_POW;



if (value > PWM_MAX)
  {
  value = PWM_MAX;
  }

maxRef = (int)value;

return (maxRef);
}  

/******************************************************************************
* Function:     StateMachine()
*
* Output:   None
*
* Overview:   The state machine controls the stepper motor states: INIT, OFF and RUN
*               Transitions between states are processed here
*
* Note:     None
*******************************************************************************/

void StepperStateMachine(void)
{
  // #ifdef MEASURE_CHANGE_PAGE_TIME
  // {__asm__ volatile ("DISI #100");}
  // PORTBbits.RB6=1;
  // #endif
  
  IEC0bits.T3IE = 0;      // Disable Timer2/3 interrupt
  Mirror_position = position;
  IEC0bits.T3IE = 1;      // Enable Timer2/3 interrupt
  
  // #ifdef MEASURE_CHANGE_PAGE_TIME
  // PORTBbits.RB6=0;
  // #endif
  
  //RUN state
  if(state == STATE_RUN)              
  {
    
    #ifdef ENABLE_BUTTON_EVENT
    /*ENABLE_BUTTON_EVENT*/    //Button event - cycle between all microstep values and OFF state
    /*ENABLE_BUTTON_EVENT*/   if( buttonCounter > BUTTON_FILTER)  //read buttonCounter state
    /*ENABLE_BUTTON_EVENT*/   { 
    /*ENABLE_BUTTON_EVENT*/       //do not allow on-the-fly step changes above this speed
    /*ENABLE_BUTTON_EVENT*/     //button was pressed and released
    /*ENABLE_BUTTON_EVENT*/     if((BUTTON_PIN == 1) && (speedOut < PI_ANTI_WINDUP_SPEED))
    /*ENABLE_BUTTON_EVENT*/     {
    /*ENABLE_BUTTON_EVENT*/       buttonCounter = 0;          //clear counter
    /*ENABLE_BUTTON_EVENT*/                //since there are two full step modes (two-phase-on and wave),
    /*ENABLE_BUTTON_EVENT*/                //step size is not increased until both modes have been used;
    /*ENABLE_BUTTON_EVENT*/                //Full step two-phase_on (stepSize ==0 and fullStepMode ==1 )
    /*ENABLE_BUTTON_EVENT*/                //Full step wave (stepSize ==0 and fullStepMode ==0 )           
    /*ENABLE_BUTTON_EVENT*/                
    /*ENABLE_BUTTON_EVENT*/                if(fullStepMode == FULLSTEP_TWO_PHASE_ON)         //full step already on
    /*ENABLE_BUTTON_EVENT*/                {
    /*ENABLE_BUTTON_EVENT*/                    fullStepMode = FULLSTEP_WAVE_DRIVE;           //turn off fullStepMode (two-phase-on)
    /*ENABLE_BUTTON_EVENT*/                }
    /*ENABLE_BUTTON_EVENT*/                else                                              //change to microstepping
    /*ENABLE_BUTTON_EVENT*/                {   
    /*ENABLE_BUTTON_EVENT*/                    //if maximum step resolution is reached then return to OFF state
    /*ENABLE_BUTTON_EVENT*/           if(stepSize >= MAX_STEP_RESOLUTION)
    /*ENABLE_BUTTON_EVENT*/               {
    /*ENABLE_BUTTON_EVENT*/                   
    /*ENABLE_BUTTON_EVENT*/             stateCopy = STATE_OFF;                  //trigger a state change to OFF state
    /*ENABLE_BUTTON_EVENT*/             
    /*ENABLE_BUTTON_EVENT*/             fullStepMode = FULLSTEP_TWO_PHASE_ON;   //set stepSize to fullstep with Two-phase on
    /*ENABLE_BUTTON_EVENT*/                        stepSize = ST_FULLSTEP;                 //calculate step Counter increment value
    /*ENABLE_BUTTON_EVENT*/                        stepSizeCopy = stepSize;                //sync variables
    /*ENABLE_BUTTON_EVENT*/                        stepSizeCount = (TABLE_SIZE)>>(stepSize);
    /*ENABLE_BUTTON_EVENT*/               }
    /*ENABLE_BUTTON_EVENT*/           else       // last step is not reached; continue to reduce step size
    /*ENABLE_BUTTON_EVENT*/           {
    /*ENABLE_BUTTON_EVENT*/                stepSize++;                                //   increase stepSize; 
    /*ENABLE_BUTTON_EVENT*/                                                           //   stepSize = 0 -> FullStep
    /*ENABLE_BUTTON_EVENT*/                                                           //   stepSize = 1 -> Half Step  
    /*ENABLE_BUTTON_EVENT*/                                                           //   stepSize = 2 -> 1/4 Step
    /*ENABLE_BUTTON_EVENT*/                                                           //   stepSize = 3 -> 1/8 Step
    /*ENABLE_BUTTON_EVENT*/                                                           //   stepSize = 4 -> 1/16 Step
    /*ENABLE_BUTTON_EVENT*/                                                           //   stepSize = 5 -> 1/32 Step
    /*ENABLE_BUTTON_EVENT*/                                                           //   stepSize = 6 -> 1/64 Step
    /*ENABLE_BUTTON_EVENT*/                                                           //   stepSize = 7 -> 1/128 Step
    /*ENABLE_BUTTON_EVENT*/                                                           //   stepSize = 8 -> 1/256 Step
    /*ENABLE_BUTTON_EVENT*/                        //create copy to detect changes made in DMCI
    /*ENABLE_BUTTON_EVENT*/                        stepSizeCopy = stepSize;
    /*ENABLE_BUTTON_EVENT*/                        
    /*ENABLE_BUTTON_EVENT*/                        //stepSizeCount used to increment the microstepping counter stepCount in timer2/3 interrupt
    /*ENABLE_BUTTON_EVENT*/               stepSizeCount = (TABLE_SIZE)>>(stepSize);   //  stepSizeCount = 256 -> FullStep
    /*ENABLE_BUTTON_EVENT*/                                                            //  stepSizeCount = 128 -> Half Step  
    /*ENABLE_BUTTON_EVENT*/                                                            //  stepSizeCount = 64  -> 1/4 Step
    /*ENABLE_BUTTON_EVENT*/                                                            //  stepSizeCount = 32  -> 1/8 Step
    /*ENABLE_BUTTON_EVENT*/                                                            //  stepSizeCount = 16  -> 1/16 Step
    /*ENABLE_BUTTON_EVENT*/                                                            //  stepSizeCount = 8   -> 1/32 Step
    /*ENABLE_BUTTON_EVENT*/                                                            //  stepSizeCount = 4   -> 1/64 Step
    /*ENABLE_BUTTON_EVENT*/                                                            //  stepSizeCount = 2   -> 1/128 Step
    /*ENABLE_BUTTON_EVENT*/                                                            //  stepSizeCount = 1   -> 1/256 Step           
    /*ENABLE_BUTTON_EVENT*/           }                                                     
    /*ENABLE_BUTTON_EVENT*/                }         
     /*ENABLE_BUTTON_EVENT*/     }
    /*ENABLE_BUTTON_EVENT*/   }
    #endif /*ENABLE_BUTTON_EVENT*/
    
    
    //Detect if DMCI changed the stepsize variable
    if(stepSizeCopy != stepSize )
    {
      //check if new step size is valid
      if(stepSizeCopy <=MAX_STEP_RESOLUTION && stepSizeCopy >= ST_FULLSTEP)
      {
        //do not allow on-the-fly step changes above this speed
        if(speedOut <PI_ANTI_WINDUP_SPEED)
        {             
          stepSize = stepSizeCopy;                            //resync the values
          stepSizeCount = (TABLE_SIZE)>>(stepSize);           //divide the look-up table into the appropriate number of steps      
          //stepSizeCount is used to increment the microstepping counter stepCount in timer2/3 interrupt
        }                                                     
      }
      else        //if new step size is not valid cancel step change
      {
        stepSizeCopy = stepSize;                                //resync the values
      }
    }
    //end of step size change mode
    
    #ifdef BIPOLAR      
    //Detect if DMCI requested a control loop mode change
    if (uGF.currentControlLoop != uGF.currentControlLoopCopy)
    {
      //This section needs to be protected from ADC and Timer2/3 interrupts
      //no motor activity is allowed until transition is made
      IEC0bits.T3IE = 0;      // Disable Timer2/3 interrupt
      IEC0bits.AD1IE = 0;   // Disable ADC interrupt
      
      //switch all PWMs off by override
      P2OVDCON = 0;
      P1OVDCON = 0;
      
      //Disable PWM and reset timers
      P1TCONbits.PTEN = 0;
      P2TCONbits.PTEN = 0;
      P1TMR = 0;
      P2TMR = 0;    
      
      //reset Timer2/3
      T2CONbits.TON = 0;
      TMR2 =0;
      TMR3 =0;
      
      
      if (uGF.currentControlLoop==ON)             //if previous mode was closed loop
      {
        maxRefValue = OPEN_LOOP_VOLTAGE;        //set the reference to the open loop maximum PWM cycle
        CalcStep();                             //recalculate the step amplitude based on the new reference   
        uGF.currentControlLoop = OFF;           //re-syncronize the control variable
        uGF.currentControlLoopCopy = OFF;   
      }
      else                                        //if previous mode was open loop (Fixed voltage or Fixed current)
      {
        maxRefValue = CLOSED_LOOP_CURRENT;      //set the reference to the open loop maximum PWM cycle
        InitPI();                               //reset PI internal state        
        CalcStep();                             //recalculate the step amplitude based on the new reference                
        uGF.currentControlLoop = ON;            //re-syncronize the control variable
        uGF.currentControlLoopCopy = ON;
      }
      
      //restart PWM if in RUN mode
      DISICNT = 10;   //disable all  interrupts with priority below 7 for 10 instruction cycles
      P1TMR = 0; //sync PWM1 and PWM2 timers
      P2TMR = 1;
      P1TCONbits.PTEN = 1; //start PWM1
      P2TCONbits.PTEN = 1; //start PWM2
      
      //restart Timer2/3 and enable ADC and Timer2/3 interrupts
      IEC0bits.T3IE = 1;      // Enable Timer2/3 interrupt
      IEC0bits.AD1IE = 1;   // Enable ADC interrupt               
      T2CONbits.TON = 1;      // Start 32-bit Timer            
    }
    //end of control loop mode change
    #endif  //#ifdef BIPOLAR    
    
    //check if a new state has been requested by DMCI or Button
    if(state!=stateCopy )
    {
      state = STATE_OFF;  //the only valid state is OFF          
      stateCopy = state;  //sync variables
      
      //This section needs to be protected from ADC and Timer2/3 interrupts
      //no motor activity is allowed until transition is made
      IEC0bits.T3IE = 0;      // Disable Timer2/3 interrupt
      IEC0bits.AD1IE = 0;   // Disable ADC interrupt
      
      LATBbits.LATB12 = 0;	//set pwm1 pins 1 and 2 as outputs at zero
      LATBbits.LATB13 = 0;
      LATBbits.LATB14 = 0;
      LATBbits.LATB15 = 0;
      
      LATCbits.LATC6 = 0;	//set pwm1 pins 3 and pwm2 as outputs at zero
      LATCbits.LATC7 = 0;
      LATBbits.LATB10 = 0;
      LATBbits.LATB11 = 0;
      
      //switch all PWMs off by override
      P2OVDCON = 0;
      P1OVDCON = 0;
      
      //Disable PWM and reset timers
      P1TCONbits.PTEN = 0;
      P2TCONbits.PTEN = 0;
      P1TMR = 0;
      P2TMR = 0;    
      
      //reset Timer2/3
      T2CONbits.TON = 0;
      TMR2 =0;
      TMR3 =0;    
      //reset motor speed
      speedOut = 0;
      
      voltageOut1 = 0;
      voltageOut2 = 0;      
    }
    
    //speed controller exection rate is set by Timer1
    if(uGF.timer1_InterruptFlag)
    {
      SpeedPositionControl();                     //call speed control at each 100us based on Timer 1 (the 100us value may vary slightly)
      uGF.timer1_InterruptFlag = 0;     //clear trigger
    }
  }
  //end of RUN state
  else
    //OFF state
  if (state==STATE_OFF)           
  {
    #ifdef ENABLE_BUTTON_EVENT
      /*ENABLE_BUTTON_EVENT*/   //read buttonCounter state; If button is presed trigger a change to RUN state
      /*ENABLE_BUTTON_EVENT*/ if( buttonCounter > BUTTON_FILTER)
      /*ENABLE_BUTTON_EVENT*/ { 
      /*ENABLE_BUTTON_EVENT*/     if(BUTTON_PIN == 1)             //button was pressed and released
      /*ENABLE_BUTTON_EVENT*/     {
      /*ENABLE_BUTTON_EVENT*/             buttonCounter = 0;          //reset buttonCounter pressed action
      /*ENABLE_BUTTON_EVENT*/            stateCopy = STATE_RUN;      //change to ON state          
      /*ENABLE_BUTTON_EVENT*/     }
      /*ENABLE_BUTTON_EVENT*/ }
    #endif /*ENABLE_BUTTON_EVENT*/
    //check if a new state has been requested
    if(state!=stateCopy)
    {           
      state = STATE_RUN;          //the only valid state is RUN
      state=stateCopy;            //sync variables      
      
      //InitStepperPosition();      
      
      //trigger a Timer1 event; signals the StateMachine to call  SpeedPositionControl()
      uGF.timer1_InterruptFlag =1;
    }
  } 
  //end of the OFF state  
  else
    //INIT state
  if (state==STATE_INIT)      //init state
  {
    //initialize PI and motor variables
    InitControlMode();
    InitStepper();
    InitPI();
    
    state = STATE_OFF;          //switch to OFF state
    stateCopy = state;          //sync variables    
  }
  //end of the INIT state 
}
//end of StateMachine()


/*******************************************************************************
* Function:     SpeedPositionControl()
*
* Output:       None
*
* Overview:     Function includes the speed and position controller
*               It is called periodically based on Timer1 interrupt
*
* Note:         None
*******************************************************************************/

void SpeedPositionControl(void)
{
  long ltemp;     //temporary variable used for speed calculation
  
  //Determine the reference speed source: POT or DMCI
  
  //limit the top speed for fixed current and fixed voltage modes
  if(uGF.currentControlLoop == OFF)
  {
    if(uGF.speedSource == POT_REF_SPEED)    //speed reference source is POT
    {
      maxSpeed = potSpeed>>5;               //potSpeed limited to 123 RPM (maxspeed = 1024)
    }
    else                                    //speed reference source is DMCI                                       
    {
      if(dmciSpeed > MAXIMUM_SPEED)                  //dmciSpeed limited to 300 RPM 
        maxSpeed = MAXIMUM_SPEED;                    //refer to the Stepper_speed_scale.xls for speed scale formula and tables
      else if(dmciSpeed < -MAXIMUM_SPEED)
        maxSpeed = -MAXIMUM_SPEED;
      else
        maxSpeed = dmciSpeed;
    }
  }
  //limit the top speed for closed loop PI control based on step size to avoid small timer values
  else
  {
    //assign speed based on source
    if(uGF.speedSource == POT_REF_SPEED)    //speed reference source is POT
    {
      maxSpeed = potSpeed;
    }
    else                                    //speed reference source is DMCI                                       
    {
      maxSpeed = dmciSpeed; 
    }
    
    //limit max pot speed to 2400 RPM
    if(maxSpeed > 20000)                //maximum speed is limited to 2400 RPM (maxspeed = 20000)
      maxSpeed = 20000;           //refer to the Stepper_speed_scale.xls for speed scale formula and tables
    else
      if(maxSpeed < -20000)     
      maxSpeed = -20000;
    
    //limit the speed based on step size; minimum step time is aprox 19us
    //there is no speed limitation for full, half or quarter step
    if((abs(speedOut)>>2)>(32767>>(stepSize)))
    {
      if(maxSpeed>0)
      {
        maxSpeed = 32767>>(stepSize-2);
        if(speedOut > maxSpeed)
          speedOut = maxSpeed;    
      }    
      else
      {    
        maxSpeed = -(32767>>(stepSize-2));
        if(speedOut < maxSpeed)
          speedOut = maxSpeed;
      }
    }//end of speed limitation in Closed loop PI control
    
  }
  
  //end of reference speed assignment
  
  
  //detect changes form Position Control to Speed Control
  if (uGF.positionControlCopy != uGF.positionControl)
  {
    uGF.positionControl = uGF.positionControlCopy;
    //reset position when entering or leaving Position control mode
    //position = 0;
  }
  
  // Position control OFF - set the maximum speed as defined by the POT or DMCI
  if(uGF.positionControl == SPEED_CONTROL)    
  {
    speedRef = maxSpeed;
  }
  //Position Controller
  else
  {
    //detect a position change from DMCI
    //if (dmciPositionRefCopy != dmciPositionRef)
    //{
    //copy the new position reference
    //dmciPositionRef = dmciPositionRefCopy;  
    
    //calculate the position reference from DMCI and scale it to long
    //positionRef = (long)dmciPositionRef*dmicPosRefScale;
    //if(uGF.currentControlLoop == OFF)
    //{
    /*Update maxRefValueInit depending on maxSpeed*/
    //maxRefValueInit =  SetMaxRefInit(dmciSpeed);  
    //maxRefValue = maxRefValueInit;
    //}    
    //}
    
    //calculate the position error
    IEC0bits.T3IE = 0;      // Disable Timer2/3 interrupt
    posError = positionRef - position;
    IEC0bits.T3IE = 1;      // Enable Timer2/3 interrupt
    //if the error is zero or less than the current step size then stop the motor   
    if((posError < stepSizeCount) && (posError > -stepSizeCount))
    {
      if(enablePosreachead)
      {
        posReached = TRUE; 
        //maxRefValue = OPEN_LOOP_VOLTAGE;        
      }
      else
      {
        posReached = FALSE;
      }
      speedRef = 0;
      
    }
    else    //if the error is nonzero set the speed reference to the speed controller
    {       
      posReached = FALSE;
      //maximum speed should be positive
      if(maxSpeed<0)
        maxSpeed = -maxSpeed;
      
      ///make sure an overflow in the gain calculation formula cannot occur
      ltemp = (long)(abs(speedOut))*POS_OVERFLOW_CHECK;
      //if in overflow area set the speed Reference to maximum 
      if (posError > ltemp)
      {
        speedRef = maxSpeed;
      }
      else
        if (posError <- ltemp)
        {
          speedRef = -maxSpeed;
        }           
        else        //deceleration point reached; switch to P controller
        { 
          long posGain;
          
          //calculate gain for the P controller based on current speed and deceleration rate
          posGain =((long)POS_FIXED_GAIN*decelerationRate/((long)abs(speedOut)));
          
          //make sure the gain is nonzero
          if(posGain==0)  
            posGain =1;
          speedRef = (posError*posGain)>>15;
          
          //limit the speed between the maximum and minimum speed range
          if (speedRef > maxSpeed)
            speedRef = maxSpeed;
          else if(speedRef <- maxSpeed)
            speedRef = -maxSpeed;
          else if(speedRef <= MINIMUM_SPEED && speedRef>0)
            speedRef = MINIMUM_SPEED+1;
          else if(speedRef >=- MINIMUM_SPEED && speedRef<0)
            speedRef = -MINIMUM_SPEED-1;
        }           
    }
  }
  //end of Position Controller
  
  //Speed controller
  
  //calculate speed error (values are in long int format)
  ltemp = (speedRef - speedOut);
  
  //speed controller for positive direction
  if(speedOut >0)
  { 
    if(ltemp > accelerationRate)                //acceleration required
    {
      speedOut +=  accelerationRate;
    }
    else                                        //deceleration required
      if( ltemp < -decelerationRate)
      {
        speedOut -=  decelerationRate;
      }
      else                                    //reached reference speed 
      {
        speedOut = speedRef;
      }       
  }
  //negative direction
  else
  {
    if(ltemp < -accelerationRate)                //acceleration required
    {
      speedOut -=  accelerationRate;
    }
    else                                        //deceleration required
      if( ltemp > decelerationRate)
      {
        speedOut +=  decelerationRate;    
      }
      else                                    //reached ref speed 
      {
        speedOut = speedRef;
      }               
  }//end of speed control 
  //end of Speed controller
  
  #ifdef BIPOLAR
  //resonance compensation active only in full step at slow speeds
  if(stepSize == ST_FULLSTEP)
  {
    //define temp variables and preload them with values used  the position control mode 
    int min=200, max=900;
    
    //use maxspeed as temp variable
    maxSpeed = abs((int)speedOut);
    resonanceCompensation = OFF;
    
    //check speed range for resonance compensation
    //values are experimental; disable the compensation and check at which speeds the motor resonates
    if(uGF.positionControl == SPEED_CONTROL)
    {
      if(fullStepMode == FULLSTEP_TWO_PHASE_ON)
      {          
        min=300;        //low speed limit; experimental value
        max=750;        //high speed limit; experimental value
      }
      else
      {
        min=460;        //low speed limit; experimental value
        max=600;        //high speed limit; experimental value
      }
    }    
    //test if abs(speedOut) is in the resonance range; maxspeed loaded previously with abs(speedOut)
    if(maxSpeed<max && maxSpeed>min)
      resonanceCompensation = ON;         //turn ON resonance compensation    
  }
  else
  {
    resonanceCompensation = OFF;                //turn OFF resonance compensation  
  }
  //end of resoanance compensation    
  
  //update the anti-windup value based on the new speed
  if(speedOut < PI_ANTI_WINDUP_SPEED && speedOut > -PI_ANTI_WINDUP_SPEED )
  {
    //value for low speeds
    PI_antiWindUp =  PI_ANTI_WINDUP_GAIN1;
  }
  else
  {
    //value for low high speeds
    PI_antiWindUp =  PI_ANTI_WINDUP_GAIN2;
  }
  //end of anti-windup calculation
  #endif  //#ifdef BIPOLAR    
  
  IEC0bits.T3IE = 0;      // Disable Timer2/3 interrupt to prevent inconsistent data to be processed
  
  //convert from speed to time SpeedPositionControl -> speedOutLong
  //Speed(step/s) = 1/ (SPEED_PRE_SCALE / speedOut * TABLE_SIZE * TIMER_PERIOD / 2^SPEED_POST_SCALE)
  if(speedOut > MINIMUM_SPEED)
  {
    ltemp = SPEED_PRE_SCALE/speedOut;   //calculate first part of formula
    T2CONbits.TON = 1;                  //restart Timer2/3 if stopped
    directionFlag = MOTOR_FORWARD;      //speed is positive so the motor turns in the positive direction
    if(uGF.currentControlLoop == ON)
    {
        maxRefValue = CLOSED_LOOP_CURRENT;
    }  
  }
  else if(speedOut < -MINIMUM_SPEED)
  {
    ltemp = SPEED_PRE_SCALE/speedOut;   //calculate first part of formula
    T2CONbits.TON = 1;                  //restart Timer2/3 if stopped
    directionFlag = MOTOR_REVERSE;      //speed is positive so the motor turns in the positive direction
    if(uGF.currentControlLoop == ON)
    {
        maxRefValue = CLOSED_LOOP_CURRENT;
    }  
  }
  else    //stop if speed is too low
  {     
    T2CONbits.TON = 0;                  // Stop 32-bit Timer
    TMR2 =0;                            // reset timer counters
    TMR3 =0;
    ltemp = SPEED_PRE_SCALE/MINIMUM_SPEED;   //calculate first part of formula
    if(uGF.currentControlLoop == OFF)
    {
      maxRefValue = OPEN_LOOP_VOLTAGE;
    }
    else
    {
      maxRefValue = CLOSED_LOOP_CURRENT_REDUCTED;
    }  
      
    CalcStep();
  }
  
  if ((uGF.currentControlLoop == OFF) && (speedOut != OldspeedOut))
  {
    OldspeedOut = speedOut;
    /*Update maxRefValueInit depending on maxSpeed*/
    maxRefValueInit =  SetMaxRefInit(speedOut);  
    enableSpeedUpdating = TRUE;
  }  
  
  
  //calculate step time based on step size and then apply postscale
  //speedOutLong is always positive
  speed2Time = (directionFlag*ltemp*stepSizeCount)>>SPEED_POST_SCALE;        //value to be written to PR3:PR2 pair
  
  //limit Timer2/3 value to 12.5us (half of PWM frequency)
  if (speed2Time < PWM_FCY)
  {
    speed2Time = PWM_FCY;
  }
  IEC0bits.T3IE = 1;      // Enable Timer2/3 interrupt
}
//end of SpeedPositionControl()


/******************************************************************************
* Function:     CalcStep()
*
* Output:   None
*
* Overview:   The function calculates the reference values for each step/microstep
*               by reconstructing the driving waveform from the look-up table
*
* Note:     None
*******************************************************************************/

void CalcStep()
{
  //define temporary variables for calculation purposes
  long stepChangeRate;
  int voltageOut1temp,voltageOut2temp;
  int stepAmplitudeRef1temp,stepAmplitudeRef2temp;
  // int setDir1temp,setDir2temp;
  
  
  //save previous winding direction; it is used to calculate the amount of step change
  setDir1temp=setDir1;
  setDir2temp=setDir2;
  
  //use temporary values for calculations; Should an interrupt occur during calculation the data will be consitent
  voltageOut1temp = voltageOut1;
  voltageOut2temp = voltageOut2;
  stepAmplitudeRef1temp = stepAmplitudeRef1;
  stepAmplitudeRef2temp = stepAmplitudeRef2;
  
  //The look-up table contains data from 0 to 90 degrees of a cosine scaled with 32767
  
  if(stepCount<TABLE_SIZE_MUL2)       //0 to 180 degrees; stepCount(0 to 511)
  {
    //0 to 90 degrees; stepCount(0 to 255)
    if(stepCount<TABLE_SIZE)    
    { 
      //adjust table(0-90 degrees) to 0-90 degrees -> no change
      stepAmplitudeRef1temp = (int)(((long)sineTable[stepCount]*maxRefValue)>>15);              //scale with maxRefValue and then back to int
      voltageOut1temp = stepAmplitudeRef1temp;                                        
      setDir1temp = FORWARD;                                                              //set winding direction to forward
      
      if(fullStepMode==FULLSTEP_WAVE_DRIVE || stepSize>ST_FULLSTEP)                       //no change needed here for FullStep two-phase-on mode
      {
        if(directionFlag == MOTOR_FORWARD)                                              //second motor winding is driven in positive direction (positive motor speed)
        {
          //adjust table(0-90 degrees) to 360-270 degrees -> reverse table index; adjust index size between 0-255  
          stepAmplitudeRef2temp = (int)(((long)sineTable[TABLE_SIZE-stepCount-1]*maxRefValue)>>15);   //scale with maxRefValue and then back to int
          voltageOut2temp = stepAmplitudeRef2temp;
          setDir2temp = FORWARD;                                                      //set winding direction to forward
        }
        else                                                                            //second motor winding is driven in negative direction (negative motor speed)
        {
          //adjust table(0-90 degrees) to 180-90 degrees -> reverse table index and reverse table sign
          stepAmplitudeRef2temp = -(int)(((long)sineTable[TABLE_SIZE-stepCount-1]*maxRefValue)>>15);
          //convert negative value to positive PWM duty cycle; winding driven by PWMxLx
          voltageOut2temp = PWM_MAX + (stepAmplitudeRef2temp);
          setDir2temp = REVERSE;                                                      //set winding direction to reverse          
        }
        
      }
    }//end of 0-90 degrees
    //90 to 180 degrees; stepCount(256 to 511)
    else
    {
      if(fullStepMode==FULLSTEP_WAVE_DRIVE || stepSize>ST_FULLSTEP)                       //no change needed here for FullStep two-phase-on mode
      {
        //adjust table(0-90 degrees) to 180-90 degrees -> reverse table index and reverse table sign; adjust index size between 0-255
        stepAmplitudeRef1temp =  - (int)(((long)sineTable[TABLE_SIZE_MUL2-stepCount-1]*maxRefValue)>>15);   //scale with maxRefValue and then back to int
        //convert negative value  to positive PWM duty cycle for open loop control
        voltageOut1temp = PWM_MAX + (stepAmplitudeRef1temp);
        setDir1temp = REVERSE;                                                          //set winding direction to reverse
      }
      if(directionFlag == MOTOR_FORWARD)                                                  //second motor winding is driven in positive direction (positive motor speed)
      {
        //adjust table(0-90 degrees) to 0-90 degrees -> no change; adjust index size between 0-255
        stepAmplitudeRef2temp = (int)(((long)sineTable[stepCount - TABLE_SIZE]*maxRefValue)>>15);     //scale with maxRefValue and then back to int
        voltageOut2temp = stepAmplitudeRef2temp;
        setDir2temp = FORWARD;                                                          //set winding direction to forward
      }
      else                                                                                //second motor winding is driven in negative direction (negative motor speed)
      {
        //adjust table(0-90 degrees) to 180-270 degrees -> reverse table sign; adjust index size between 0-255
        stepAmplitudeRef2temp =  - (int)(((long)sineTable[stepCount-TABLE_SIZE]*maxRefValue)>>15);  //scale with maxRefValue and then back to int
        //convert negative value to positive PWM duty cycle for open loop control
        voltageOut2temp =  PWM_MAX + (stepAmplitudeRef2temp);
        setDir2temp = REVERSE;                                                          //set winding direction to reverse 
      }
    }//end of 90-180 degrees
  }//end of 0-180 degrees
  //180 to 360 degrees; stepCount(512 to 1023)
  else
  {
    if(stepCount<TABLE_SIZE_MUL3)     //180 to 270 degrees; stepCount(512 to 767)
    {
      //adjust table(0-90 degrees) to 180-270 degrees -> reverse table sign; adjust index size between 0-255
      stepAmplitudeRef1temp = - (int)(((long)sineTable[stepCount - TABLE_SIZE_MUL2]*maxRefValue)>>15);    //scale with maxRefValue and then back to int
      //convert negative value to positive PWM duty cycle for open loop control
      voltageOut1temp = PWM_MAX + (stepAmplitudeRef1temp);
      setDir1temp = REVERSE;                                                              //set winding direction to reverse
      
      if(fullStepMode==FULLSTEP_WAVE_DRIVE || stepSize>ST_FULLSTEP)                       //no change needed here for FullStep two-phase-on mode
      {
        if(directionFlag == MOTOR_FORWARD)                                              //second motor winding is driven in positive direction (positive motor speed)
        {
          //adjust table(0-90 degrees) to 180-90 degrees -> reverse table index and reverse table sign; adjust index size between 0-255                 
          stepAmplitudeRef2temp = - (int)(((long)sineTable[TABLE_SIZE_MUL3-stepCount-1]*maxRefValue)>>15);  //scale with maxRefValue and then back to int
          //convert negative value to positive PWM duty cycle for open loop control
          voltageOut2temp = PWM_MAX + (stepAmplitudeRef2temp);
          setDir2temp = REVERSE;                                                      //set winding direction to reverse
        }
        else                                                                            //second motor winding is driven in negative direction (negative motor speed)
        {
          //adjust table(0-90 degrees) to 360-270 degrees -> reverse table index; adjust index size between 0-255  
          stepAmplitudeRef2temp = (int)(((long)sineTable[TABLE_SIZE_MUL3-stepCount-1]*maxRefValue)>>15);  //scale with maxRefValue and then back to int
          voltageOut2temp = stepAmplitudeRef2temp;
          setDir2temp = FORWARD;                                                      //set winding direction to forward          
        }   
      } 
    }//end of 180-270 degrees 
    //270 to 360 degrees; stepCount(768 to 1023)
    else
    {
      if(fullStepMode==FULLSTEP_WAVE_DRIVE || stepSize>ST_FULLSTEP)                       //no change needed here for FullStep two-phase-on mode
      {
        //adjust table(0-90 degrees) to 360-270 degrees -> reverse table index; adjust index size between 0-255  
        stepAmplitudeRef1temp = (int)(((long)sineTable[TABLE_SIZE_MUL4-stepCount-1]*maxRefValue)>>15);  //scale with maxRefValue and then back to int
        voltageOut1temp = stepAmplitudeRef1temp;
        setDir1temp = FORWARD;                                                          //set winding direction to forward
        
      }
      if(directionFlag == MOTOR_FORWARD)                                                  //second motor winding is driven in positive direction (positive motor speed)
      {
        //adjust table(0-90 degrees) to 180-270 degrees -> reverse table sign; adjust index size between 0-255
        stepAmplitudeRef2temp = - (int)(((long)sineTable[stepCount-TABLE_SIZE_MUL3]*maxRefValue)>>15);  //scale with maxRefValue and then back to int
        //convert negative value to positive PWM duty cycle for open loop control
        voltageOut2temp =  PWM_MAX + (stepAmplitudeRef2temp);
        setDir2temp = REVERSE;                                                          //set winding direction to reverse
      }
      else                                                                                //second motor winding is driven in negative direction (negative motor speed)
      {
        //adjust table(0-90 degrees) to 0-90 degrees -> no change; adjust index size between 0-255
        stepAmplitudeRef2temp = (int)(((long)sineTable[stepCount - TABLE_SIZE_MUL3]*maxRefValue)>>15);    //scale with maxRefValue and then back to int
        voltageOut2temp = stepAmplitudeRef2temp;
        setDir2temp = FORWARD;                                                          //set winding direction to forward      
      }
    }//end of 270-360 degrees
  }
  //end of 180-360 degrees
  
  #ifdef BIPOLAR
  //Alternate decay procedure; calculate if a decay change to alternateDecay is needed
  //This mode is not needed in closed loop PI control
  if(uGF.currentControlLoop == OFF && uGF.decayMode == ALTERNATE_DECAY)
  {
    //calculate the amount of step decrease for winding 1; stepChangeRate = |OldValue|-|NewValue|
    stepChangeRate = stepAmplitudeRef1*(1-2*setDir1)-stepAmplitudeRef1temp*(1-2*setDir1temp);
    if(stepChangeRate > 0)                  //if step amplitude is decreasing
    {
      //calculate the number of decay periods for which the decay is changed
      //for D_FAST, the default value for altDecayScale is 4
      stepChangeDecayNumber1 = stepChangeRate>>altDecayScale;  
      if(stepChangeDecayNumber1>0)        //Number of decay cycles should be higher than zero   
      {
        if(stepAmplitudeRef1temp == 0)        //maintain fast decay as long as possible if the step amplitude is zero
          stepChangeDecayNumber1 = 32767;
        decay1 = alternateDecay;        //default value is D_FAST  
        stepChangeDecayCounter1 = 1;    //start counter to signal a decay change in ADC interrupt
      }    
    }   
    else                                    //if step amplitude is increasing use base Decay
    {
      stepChangeDecayNumber1 = 0;         //set the number of decay change periods to 0
      decay1 = baseDecay;                 //default is D_SLOW_L_MOSFET  
    } 
    
    //calculate the amount of step decrease for winding 2; stepChangeRate = |OldValue|-|NewValue|
    stepChangeRate = stepAmplitudeRef2*(1-2*setDir2)-stepAmplitudeRef2temp*(1-2*setDir2temp);
    if(stepChangeRate > 0)                  //if step amplitude is decreasing
    {
      //calculate the number of decay periods for which the decay is changed
      //for D_FAST, the default value for altDecayScale is 4
      stepChangeDecayNumber2 = stepChangeRate>>altDecayScale;
      if(stepChangeDecayNumber2>0)        //Number of decay cycles should be higher than zero 
      {
        if(stepAmplitudeRef2temp == 0)        //maintain fast decay as long as possible if the step amplitude is zero 
          stepChangeDecayNumber2 = 32767;
        decay2 = alternateDecay;        //default value is D_FAST
        stepChangeDecayCounter2 = 1;    //start counter to signal a decay change in ADC interrupt
      }    
    }
    else                                    //if step amplitude is increasing use base Decay
    {
      stepChangeDecayNumber2 = 0;         //set the number of decay change periods to 0
      decay2 = baseDecay;                 //default is D_SLOW_L_MOSFET     
    } 
  }
  // no alternate decay used; set base decay
  else
  {
    //set default decay
    decay1 = baseDecay;                     //default value is D_SLOW_L_MOSFET
    decay2 = baseDecay;
  }      
  //end of alternate decay procedure
  #endif  //#ifdef BIPOLAR
  
  
  //disable ADC interrupt to prevent inconsistent data being processed in the controller
  IEC0bits.AD1IE = 0;   // disable ADC interrupt
  //load the lastest data; ADC interrupt disabled during this update
  voltageOut1 = voltageOut1temp;
  voltageOut2 = voltageOut2temp;
  stepAmplitudeRef1 = stepAmplitudeRef1temp;
  stepAmplitudeRef2 = stepAmplitudeRef2temp;
  
  //setDirX values are updated only in fixed current and fixed voltage modes
  //in closed loop mode the PI has control of the winding driving direction
  if(uGF.currentControlLoop == OFF)
  {
    setDir1=setDir1temp;
    setDir2=setDir2temp;
    CalcDecay();            //calculate PxOVDCON register values for the new decay mode and winding directions
  }    
  IEC0bits.AD1IE = 1;   // enable ADC interrupt
  
}
//end of CalcStep()


/******************************************************************************
* Function:     CalcDecay()
*
* Output:   None
*
* Overview:   The function uses a look-up table to setup the Override registers content 
*               for the selected decay mode
*
* Note:     None
*******************************************************************************/

void CalcDecay()
{
  int temp; 
  
  #ifdef BIPOLAR
  
  //bipolar motor configuration
  temp = 0;
  //calculate Override values for winding 2  
  temp = decayTableBipolar[setDir2][decay2];
  //write values for PWM2 pair; extract bits 2 and 3 that correspond to PWM2
  ovdPwm2 = temp >> 2;                    //this will be loaded into P2OVDCON
  //calculate and write Override values for PWM1
  //add bits 0 and 1 from temp variable and place them in the PWM1H3 and PWM1L3 location (left shift by 4 positions)
  ovdPwm1 = decayTableBipolar[setDir1][decay1]+(temp<<4); //this will be loaded into P1OVDCON
  
  #else
  
  //unipolar motor configuration    
  if(decay1>D_SLOW) 
    decay1 = D_SLOW;
  if(decay2>D_SLOW) 
    decay2 = D_SLOW;
  //calculate Override values for winding 2
  //the table is offset with 6 positions for negative direction (setDir2==REVERSE)
  temp = decayTableUnipolar[decay2];
  //write values for PWM2 pair; extract bits 2 and 3 that correspond to PWM2
  ovdPwm2 = temp >> 2;
  //calculate and write Override values for PWM1
  //add bits 0 and 1 from temp variable and place them in the PWM1H3 and PWM1L3 location
  ovdPwm1 = decayTableUnipolar[decay1]+(temp<<4);        
  #endif
}
//end of CalcDecay()


/******************************************************************************
* Function:     SetPWM()
*
* Output:   None
*
* Overview:   The function loads the PWM duty cycle registers and Override registers with 
previously calculated values
*
* Note:     None
*******************************************************************************/

void SetPWM()
{
  
  //load PWM override registers according to the decay settings calculated in the CalcDecay() function
  P2OVDCON = ovdPwm2;
  P1OVDCON = ovdPwm1;
  
  #ifdef BIPOLAR      //load duty cycles for bipolar configuration
  
  P1DC1 = pwmOut1;
  P1DC2 = pwmOut1;
  
  P1DC3 = pwmOut2;
  P2DC1 = pwmOut2;
  #else               //load duty cycles for unipolar configuration
  
  
  if(setDir1==FORWARD)        //drive winding at M1
  {
    //reverse duty cycle since PWM1L1 is driving the winding
    P1DC1 = PWM_MAX-pwmOut1;
    //winding driven by PWM1L2 needs to be off
    P1DC2 = 0;
  } 
  else                        //drive winding at M2
  {
    //winding driven by PWM1L1 needs to be off, so PWM1H1 is high
    P1DC1 = PWM_MAX;
    //reverse duty cycle since PWM1L2 is driving the winding
    P1DC2 = PWM_MAX-pwmOut1;
  }
  
  if(setDir2==FORWARD)        //drive winding at M3
  {
    //reverse duty cycle since PWM1L3 is driving the winding
    P1DC3 = PWM_MAX-pwmOut2;
    //winding driven by PWM2L1 needs to be off
    P2DC1 = 0;
  } 
  else                        //drive winding at M4
  {
    //winding driven by PWM1L3 needs to be off, so PWM1H3 is high
    P1DC3 = PWM_MAX;
    //reverse duty cycle since PWM2L1 is driving the winding
    P2DC1 = PWM_MAX-pwmOut2;
  }  
  #endif  
}
//end of SetPWM()

/******************************************************************************
* Function:     CalcPI1(int currentReference,int currentMeasurement)
*
* Output:   PI_out1 
*
* Overview:   This function implements PI control for Winding 1 current
*               The discrete PI controller has the following format:
*                                 PI_PARAM_1 * z - PI_PARAM_2
*                H(z) = PI_GAIN * ---------------------------
*                                             z - 1 
*
* Note:     None
*******************************************************************************/
int CalcPI1(int currentReference,int currentMeasurement)
{
  // long currentError;
  
 
  
    //calculate the current currentErroror
  currentError1 = (((long)currentReference - currentMeasurement));
  
  
    //calculate the controller equation, including anti-windup
  PI_sum1 = PI_sum1+((currentError1 * PI_GAIN_PARAM_1 - PI_err1 * PI_GAIN_PARAM_2)/1000)-(((PI_sum1-PI_out1) * PI_antiWindUp)>>15);

  //limit the controller output to the available drive voltage
  if(PI_sum1 > dcBus)
    PI_out1 = dcBus;
  else if(PI_sum1 < -dcBus)
    PI_out1 = -dcBus;
  else
    PI_out1 = (int)PI_sum1;
    
    //resonance compensation active only in full step at slow speeds
    if(resonanceCompensation == ON)
    {
        PI_out1 = PI_out1>>2;       //reduce output magnitude to allow more time for slow decay 
    }

   
 PI_err1 = currentError1;         //save the error for use in the next cycle
    
  return PI_out1;                 //controller voltage output
}


/******************************************************************************
* Function:     CalcPI2(int currentReference,int currentMeasurement)
*
* Output:   PI_out2 
*
* Overview:   This function implements PI control for Winding 2 current
*               The discrete PI controller has the following format:
*                                 PI_PARAM_1 * z - PI_PARAM_2
*                H(z) = PI_GAIN * ---------------------------
*                                             z - 1 
*
* Note:     None
*******************************************************************************/
int CalcPI2(int currentReference,int currentMeasurement)
{
  // long currentError;
  
  
  
    //calculate the current currentErroror
  currentError2 = (((long)currentReference - currentMeasurement));
    
    //calculate the controller equation, including anti-windup
  PI_sum2 = PI_sum2+((currentError2 * PI_GAIN_PARAM_1 - PI_err2 * PI_GAIN_PARAM_2)/1000/*>>13*/)-(((PI_sum2-PI_out2) * PI_antiWindUp)>>15);
  pi_param1 = PI_GAIN_PARAM_1;
  pi_param2 = PI_GAIN_PARAM_2;
    //limit the controller output to the available drive voltage
  if(PI_sum2 > dcBus)
    PI_out2 = dcBus;
  else if(PI_sum2 < -dcBus)
    PI_out2 = -dcBus;
  else
    PI_out2 = (int)PI_sum2;

    //resonance compensation active only in full step at slow speeds
    if(resonanceCompensation == ON)
    {
        PI_out2 = PI_out2>>2;       //reduce output magnitude to allow more time for slow decay 
    }
    

    #ifdef DEBUG_RISPOSTA_GRADINO
    if ((StatusTimer(T_AIR_OVERRIDEN) == T_ELAPSED) && (count2 < N_DEBUG_SAMPLES))
    {                
      debug_sum2[count2] = PI_sum2;
      debug_out2[count2] = PI_out2;
      debug_err2[count2] = PI_err2;
      debug_currentError2[count2] = currentError2; 
      count2++;
    }
    #endif
        PI_err2 = currentError2;         //save the error for use in the next cycle
    
  return PI_out2;                 //controller voltage output
}

void StartStepper(unsigned short speed, unsigned char direction)
/*
*//*=====================================================================*//**
**
**      @brief Start stepper with speed (SPEED_CONTROL)
**
**      @param speed [RPM]
**
**      @param direction 
** 
**      @retval void
**
**
*//*=====================================================================*//**
*/
{
unsigned long speed_conversion;
int speed_slider;

speed_conversion = (speed*CONV_SPEED_SLIDER)/CONV_MIN_SEC;

speed_slider = (int)speed_conversion;
if (direction == REVERSE)
{
  speed_slider= -speed_slider;
  directionFlag = MOTOR_REVERSE;
}
else
{
directionFlag = MOTOR_FORWARD;
}  

dmciSpeed = speed_slider;
stateCopy = STATE_RUN;
uGF.positionControlCopy = SPEED_CONTROL;
}  

void InitStartStepperRamp(void)
/*
*//*=====================================================================*//**
**
**      @brief Start stepper with speed (SPEED_CONTROL)
**
**      @param speed [RPM]
**
**      @param direction 
** 
**      @retval void
**
**
*//*=====================================================================*//**
*/
{
  speed_ramp = 0;
  statusRamp = RAMP_UP;
  StartTimer(T_COUNT_RAMP);
}  

void StartStepperRamp(unsigned short speed, unsigned char direction, unsigned short delta_ramp_up)
/*
*//*=====================================================================*//**
**
**      @brief Start stepper with speed (SPEED_CONTROL)
**
**      @param speed [RPM]
**
**      @param direction 
** 
**      @retval void
**
**
*//*=====================================================================*//**
*/
{
unsigned long speed_conversion;
int speed_slider;

if (StatusTimer(T_COUNT_RAMP) == T_ELAPSED)
{
  switch (statusRamp)
  {
  case RAMP_IDLE:
    break;
    
  case RAMP_UP:
    speed_ramp = speed_ramp + delta_ramp_up;   
    if (speed_ramp >=  speed)
    {
      speed_ramp = speed;
      statusRamp = RAMP_PLATEAU;
    }
    break;
  
  case RAMP_PLATEAU:
    break;   
  
  case RAMP_DW:
    break;   
  } 
  StartTimer(T_COUNT_RAMP);
}

speed_conversion = (speed_ramp*CONV_SPEED_SLIDER)/CONV_MIN_SEC;

speed_slider = (int)speed_conversion;
if (direction == REVERSE)
  {
  speed_slider= -speed_slider;
  }  

dmciSpeed = speed_slider;
stateCopy = STATE_RUN;
uGF.positionControlCopy = SPEED_CONTROL;
}  

void InitMoveStepperRamp(unsigned short speed, 
                         unsigned short delta_up,
                         unsigned short delta_dw, 
                         long n_step_motion)
/*
*//*=====================================================================*//**
**
**      @brief Init variables to manage ramp
**
**      @param set_point speed set point [msec]
** 
**      @param delta_dw  delta speed decrement [RPM]
** 
**      @param number motion stepper full step
**
**      @retval void
**
**
*//*=====================================================================*//**
*/
{
  unsigned short speed_step;
  unsigned short n_step_rump_dw;
  
  motion_ramp_direction = getDirection(n_step_motion);  
  
  if (n_step_motion == POS_0) 
  {
    n_step_motion = POS_0 + DELTA_RIGHT_POS_0;
  } 
  
  speed_step = speed;
  //Calcolo numero di step della rampa di discesa
  n_step_rump_dw = 0;
  if (delta_dw)
  {
    while (speed_step >= delta_dw)
    {
      speed_step = speed_step - delta_dw;
      n_step_rump_dw+= (speed_step*NUM_STEP_PER_REVOLUTION)*(T_BASE*DELAY_COUNT_RAMP)/CONV_MIN_MSEC;
    } 
  }
  
  //Calcolo posizione di inizio rampa di discesa
  if (motion_ramp_direction == FORWARD)
  {
    start_ramp_dw_num_step =n_step_motion - n_step_rump_dw;
  }
  else
  {
    start_ramp_dw_num_step = n_step_motion + n_step_rump_dw;
  }
  
  if (delta_up)
  {
    speed_ramp = 0;
    statusRamp = RAMP_UP;
    StartTimer(T_COUNT_RAMP);
  }
  else
  {
    speed_ramp = speed;
    statusRamp = RAMP_PLATEAU;     
  }  

}  


void MoveStepperRamp(unsigned short speed, 
                     long n_step_motion,
                     unsigned short delta_up,
                     unsigned short delta_dw)
/*
*//*=====================================================================*//**
**
**      @brief Move the stepper to n_step_motion position with ramp
**
**      @param speed [RPM]
** 
**      @param n_step_motion [full step] 
**
**      @param delta_up [full step] 
**
**      @param delta_dw [full step] 
**
**      @param n_step_rump_dw [full step starting decrease ramp] 
**
**      @retval void
**
*//*=====================================================================*//**
*/
{
  unsigned long speed_conversion;
  int speed_slider;
  
  if (n_step_motion == POS_0) 
  {
    n_step_motion = POS_0 + DELTA_RIGHT_POS_0;
  }  
  
  switch (statusRamp)
  {
  case RAMP_IDLE:
    break;
    
  case RAMP_UP:
    if (StatusTimer(T_COUNT_RAMP) == T_ELAPSED)
    {
      speed_ramp = speed_ramp + delta_up;   
      if (speed_ramp >=  speed)
      {
        speed_ramp = speed;
        statusRamp = RAMP_PLATEAU;
      }
      else
      {    
        StartTimer(T_COUNT_RAMP);
      }
    }
    break;
    
  case RAMP_PLATEAU:
    speed_ramp = speed;
    
    IEC0bits.T3IE = 0;      // Disable Timer2/3 interrupt
    curr_position = position;
    IEC0bits.T3IE = 1;   // Enable Timer2/3 interrupt    
    curr_position = curr_position/TABLE_SIZE;
    if (( (motion_ramp_direction == FORWARD) && (curr_position >= start_ramp_dw_num_step)) ||
      (  (motion_ramp_direction == REVERSE) && (curr_position < start_ramp_dw_num_step)))
    {
      statusRamp = RAMP_DW;
      StartTimer(T_COUNT_RAMP);
    }
    break;
    
  case RAMP_DW:
    if (StatusTimer(T_COUNT_RAMP) == T_ELAPSED)
    {
      if (speed_ramp >= delta_dw)
      {
        speed_ramp -= delta_dw;      
        StartTimer(T_COUNT_RAMP);
      }
      else
      {
        speed_ramp = 0;
        statusRamp = RAMP_IDLE;
        StopTimer(T_COUNT_RAMP);
      }
    }
    break;   
  } 
  
  speed_conversion = (speed_ramp*CONV_SPEED_SLIDER)/CONV_MIN_SEC;
  
  speed_slider = (int)speed_conversion;
  
  // Num_Count_Position (stepCount unit, 1 revolution = 200 FULL STEPS, 
  // 1 FULL STEP = 256 stepCount-> 1 revolution = 200*256 =
  // 51200 stepCounts) with respect of absolute zero position
  
  positionRef = n_step_motion*TABLE_SIZE;
  
  stateCopy = STATE_RUN;
  dmciSpeed = speed_slider;
  if (dmciSpeed <= MINIMUM_SPEED)
  {
    dmciSpeed = MINIMUM_SPEED+1;
  } 
  
  uGF.positionControlCopy = POSITION_CONTROL; 
  enablePosreachead= TRUE;
}  

void StopStepper(void)
/*
*//*=====================================================================*//**
**
**      @brief Stop the stepper, maintaining currents on windings
**
**      @param void 
** 
**      @retval void
**
**
*//*=====================================================================*//**
*/
{
dmciSpeed = 0;
speedOut = 0;
//maxRefValue = OPEN_LOOP_VOLTAGE;
StopTimer(T_COUNT_RAMP);
}  

void MoveStepper(unsigned short speed, long Num_Step_Position)
/*
*//*=====================================================================*//**
**
**      @brief Move the stepper to Num_Count_Position position
**
**      @param speed [RPM]
** 
**      @param Num_Step_Position [full step] 
** 
**      @retval void
**
**
*//*=====================================================================*//**
*/
{
unsigned long speed_conversion;
int speed_slider;

speed_conversion = (speed*CONV_SPEED_SLIDER)/CONV_MIN_SEC;

speed_slider = (int)speed_conversion;

if (Num_Step_Position == POS_0) 
  {
    Num_Step_Position = POS_0 + DELTA_RIGHT_POS_0;
  } 
  
// Num_Count_Position (stepCount unit, 1 revolution = 200 FULL STEPS, 
// 1 FULL STEP = 256 stepCount-> 1 revolution = 200*256 =
// 51200 stepCounts) with respect of absolute zero position

positionRef = Num_Step_Position*TABLE_SIZE;

stateCopy = STATE_RUN;
dmciSpeed = speed_slider;
if (dmciSpeed <= MINIMUM_SPEED)
{
dmciSpeed = MINIMUM_SPEED+1;
} 
//speedOut = 0;

uGF.positionControlCopy = POSITION_CONTROL; 
enablePosreachead= TRUE;
}  

void MoveStepperToZero(unsigned short speed, unsigned char direction, unsigned short NumStep)
/*
*//*=====================================================================*//**
**
**      @brief Move the stepper of NumStep (POSITION_CONTROL)
**
**      @param speed [RPM]
**
**      @param direction 
** 
**      @param NumStep (stepCount unit, 1 revolution = 200 FULL STEPS, 
**             1 FULL STEP = 256 stepCount-> 1 revolution = 200*256 =
**             51200 stepCounts)
** 
**      @retval void
**
**
*//*=====================================================================*//**
*/
{
unsigned long speed_conversion;
int speed_slider;

speed_conversion = (speed*CONV_SPEED_SLIDER)/CONV_MIN_SEC;

speed_slider = (int)speed_conversion;
positionRef = NumStep;
if (direction == REVERSE)
  {
  speed_slider= -speed_slider;
  positionRef = -positionRef;
  }  

dmciSpeed = speed_slider;
speedOut = 0;

uGF.positionControlCopy = POSITION_CONTROL;
enablePosreachead= TRUE;
IEC0bits.T3IE = 0;      // Disable Timer2/3 interrupt
position = 0;
IEC0bits.T3IE = 1;      // Enable Timer2/3 interrupt
}  

unsigned char isStepperPositionReached(void)
/*
*//*=====================================================================*//**
**
**      @brief Checks if position is reached
**
**      @param void 
** 
**      @retval TRUE/FALSE
**
**
*//*=====================================================================*//**
*/
{
unsigned char ret = FALSE;
if (uGF.positionControl == POSITION_CONTROL)
  {
  ret = posReached;
  }  
return(ret);
}


void setStepperPositionReached(unsigned char value)
/*
*//*=====================================================================*//**
**
**      @brief set position reached
**
**      @param unsigned char 
** 
**      @retval void 
**
**
*//*=====================================================================*//**
*/
{
  posReached = value;
  enablePosreachead = value;
}

void PowerOffStepper(void)
/*
*//*=====================================================================*//**
**
**      @brief power off stepper motor
**
**      @param void
** 
**      @retval void 
**
**
*//*=====================================================================*//**
*/
{
 stateCopy = STATE_OFF;
}


void resetStepPositionCount(void)
/*
*//*=====================================================================*//**
**
**      @brief set position to 0 (absolute)
**
**      @param  void
** 
**      @retval void 
**
**
*//*=====================================================================*//**
*/
{
IEC0bits.T3IE = 0;      // Disable Timer2/3 interrupt
StepPositionCount = 0;
IEC0bits.T3IE = 1;      // Enable Timer2/3 interrupt
}

unsigned long getStepPositionCount(void)
/*
*//*=====================================================================*//**
**
**      @brief set position to 0 (absolute)
**
**      @param  void
** 
**      @retval void 
**
**
*//*=====================================================================*//**
*/
{
IEC0bits.T3IE = 0;      // Disable Timer2/3 interrupt
return (StepPositionCount);
IEC0bits.T3IE = 1;      // Enable Timer2/3 interrupt
}

long getPositionRef(void)
/*
*//*=====================================================================*//**
**
**      @brief set position to 0 (absolute)
**
**      @param  void
** 
**      @retval void 
**
**
*//*=====================================================================*//**
*/
{
return (positionRef);
}

long getStepPosition(void)
/*
*//*=====================================================================*//**
**
**      @brief return position (in step) from 0 (absolute)
**
**      @param  void
** 
**      @retval void 
**
**
*//*=====================================================================*//**
*/
{
  IEC0bits.T3IE = 0;      // Disable Timer2/3 interrupt
  curr_position = position;
  IEC0bits.T3IE = 1;      // Enable Timer2/3 interrupt    
  curr_position = curr_position/TABLE_SIZE;   
  return (curr_position);
}

unsigned char getDirection(long n_step_motion)
/*
*//*=====================================================================*//**
**
**      @brief Init variables to manage ramp
**
**      @param number motion stepper full step
**
**      @retval motion direction
**
**
*//*=====================================================================*//**
*/
{

  if (getStepPosition() >= n_step_motion)
  {
    return(REVERSE);
  } 
  else
  {
    return(FORWARD);
  }  
}  

void setAbsZeroPosition(void)
/*
*//*=====================================================================*//**
**
**      @brief set position to 0 (absolute)
**
**      @param  void
** 
**      @retval void 
**
**
*//*=====================================================================*//**
*/
{
IEC0bits.T3IE = 0;      // Disable Timer2/3 interrupt
position = 0;
IEC0bits.T3IE = 1;      // Enable Timer2/3 interrupt
}


void initZeroPosition(void)
/*
*//*=====================================================================*//**
**
**      @brief initialize position and position reference
**
**      @param  void
** 
**      @retval void 
**
**
*//*=====================================================================*//**
*/
{
//initialize position and position reference
	IEC0bits.T3IE = 0;      // Disable Timer2/3 interrupt
	position = 0;                 //motor actual position
	IEC0bits.T3IE = 1;      // Enable Timer2/3 interrupt
	positionRef = 0;              //desired motor position
}


