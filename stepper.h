/**/
/*============================================================================*/
/**
**      @file      STEPPER.H
**
**      @brief     Header file relativo a STEPPER.C
**
**      @version   Alfa Color tester
**/
/*============================================================================*/
/**/
#ifndef STEPPER_H       //avoid double inclusion
#define STEPPER_H


#include "p33FJ32MC204.h"
#include "dsp.h"
#include "init.h"
#include "debug.h"
#include "RTDM.h"
#include "macro.h"
#include "userparams.h"

#define  STEPPER_DTIVER_STANDBY LATAbits.LATA10

#define Nop()    {__asm__ volatile ("nop");}
    //general flags
    typedef 
        struct{
            unsigned speedSource		    :1;         //Sets the speed reference source for the speed controller;
            unsigned decayMode  		    :1;         //choose between fixed decay and alternate decay in Fixed Voltage/Current modes
            unsigned controlMode			:1;         //Switches between Fixed Voltage and Fixed Current modes
            unsigned timer1_InterruptFlag   :1;         //Flag used to trigger the State Machine that a Timer1 interrupt has occured
            unsigned positionControl        :1;         //Switches between Speed and Position control modes
            unsigned positionControlCopy    :1;         //Flag used by DMCI to set the positionControl flag
            unsigned currentControlLoop     :1;         //Switches between Closed loop PI current control and Fixed Voltage/Current modes
            unsigned currentControlLoopCopy :1;         //Flag used by DMCI to set the currentControlLoop flag
            unsigned               		    :8;         //unused
    }generalFlags; 

/* Globals */

    extern volatile int stepCount;					//micro-step counter
    extern int stepSize;					//step mode /
    extern int stepSizeCount;				//used to increment the microstepping counter stepCount in timer2/3 interrupt                                         
    extern int fullStepMode;                //controls change from 'full step 2 phase on' and 'full step wave drive'                                       //stepSize is identical in these 2 cases so this flag is needed                                   
    extern int stepSizeCopy;				//copy of stepSize; used to manage step changes from DMCI  
    extern int setDir1;					    //winding 1 desired driving direction
    extern int setDir2;					    //winding 1 desired driving direction
    extern int setDir1HW;                   //copy of setDir1; controls current measurement; reflects the actual hardware current direction through the shunt resistor
    extern int setDir2HW;                   //copy of setDir1; controls current measurement; reflects the actual hardware current direction through the shunt resistor
    extern int voltageOut1,voltageOut2;     //output voltage for each phase
    extern int stepAmplitudeRef1;           //voltage reference for winding 1
    extern int stepAmplitudeRef2;           //voltage reference for winding 2
    extern char directionFlag;				//motor direction; +1 is p; -1 is counter clockwise (depends on motor connection; direction can be reversed  by swapping one motor coil wires eg: M1 with M2)
    extern char directionFlagOld;
    extern int current1, current2;			//measured current for each phase     
    extern volatile int currentMaxAmplitude;         //maximum amplitude of the current of both windings combined over one sinewave period (stepCount from 0 to 1023)
    extern long speedRef;                   //speed reference value input to the speed controller
    extern long speedOut;                   //motor speed; speed controller output
		extern long OldspeedOut;                   //motor speed; speed controller output
    extern long speed2Time;                 //motor speed converted to timer2/3 periods
    extern int dmciSpeed;                   //motor speed set by DMCI
    extern int potSpeed;                    //motor speed set by POT                                     
    extern int accelerationRate;            //maximum accelerationRate rate measured in timer counts (Timer2)
    extern int decelerationRate;            //maximum decelerationRate rate measured in timer counts (Timer2)
    extern int pwmOut1,pwmOut2;		        //temporary variable to hold PWM duty cycle values during calculations  
    extern int dcBus;						//power stage DC voltage; see description of VOLTAGE for scaling method
    extern int state;						//State machine: current state
    extern int stateCopy;                   //copy of state; used to manage state changes from DMCI
    extern int buttonCounter;               //S1 push buttonCounter used for debouncing 
    extern int stepChangeDecayCounter1;     //Counter that controls the decay change period in alternate decay mode; for winding 1
    extern int stepChangeDecayCounter2;     //Counter that controls the decay change period in alternate decay mode; for winding 2
    extern unsigned char decay1,decay2;     //current decay modes for each winding; see decay modes in userparams.h for possible values
    extern int baseDecay;                   //current decay mode used in FIXED_DECAY mode; initial value is set in InitStepper() function
    extern int alternateDecay;              //current decay used in ALTERNATE_DECAY mode; initial value is set in InitStepper() function
    extern int stepChangeDecayNumber1;      //number of PWM periods in which alternate decay is used; for winding 1
    extern int stepChangeDecayNumber2;      //number of PWM periods in which alternate decay is used; for winding 2
    extern int altDecayScale;               //scaling value used to calculate the decay change period
    
    extern volatile int maxRefValue;                 //current or voltage reference; this value scales the look-up table to the desired amplitude

    extern volatile long position;                   //motor position calculated by counting steps
    extern long Mirror_position;           //copy of position to read position outside Timer 3 interrupt
    extern long positionRef;                //desired motor position
    extern long posError;                   //posError = positionRef - position
    extern int dmciPositionRef;             //int version of positionRef; set by DMCI
    extern int dmicPosRefScale;             //scale dmciPositionRef to long; positionRef = dmciPositionRef*dmicPosRefScale
    extern int dmciPositionRefCopy;         //copy of dmicPosRefScale; used to detect position reference changes from DMCI
    
    extern int PI_ref1;                     //current reference for winding 1
    extern long PI_err1;                    //current error for winding 1
    extern int PI_out1;                     //PI controller output (voltage)
    extern long PI_sum1;                    //PI integral accumulator
    extern int PI_ref2;                     //current reference for winding 2
    extern long PI_err2;                    //current error for winding 2
    extern int PI_out2;                     //PI controller output (voltage)
    extern long PI_sum2;                    //PI integral accumulator
    extern int PI_antiWindUp;               //anti wind-up value for the PI controller; changes with speed
    extern char resonanceCompensation;      //flag used to compensate the motor resonance in full step modes;
        
    extern volatile generalFlags uGF;                //general flags for application control
  		extern unsigned long StepPositionCount;
  		extern int setDir1temp,setDir2temp;
/* Functions */

void CalcStep();                                            //calculate the open loop duty cycle/closed loop current reference values
void CalcDecay(void);                                       //write decay mode values to PWM Override registers
void SetPWM(void);                                          //write values to PWM registers
void StateMachine(void);                                    //state machine that controls the software operation
void SpeedPositionControl(void);                            //position and speed controller called periodically after T2 interrupts
int CalcPI1(int currentReference,int currentMeasurement);   //implements PI controller for winding 1
int CalcPI2(int currentReference,int currentMeasurement);   //implements PI controller for winding 1
void StepperStateMachine(void);
extern void MoveStepper(unsigned short speed, long Num_Step_Position);
extern void MoveStepperToZero(unsigned short speed, unsigned char direction, unsigned short NumStep);
extern void StartStepper(unsigned short speed, unsigned char direction);
extern void StopStepper(void);
extern unsigned char isStepperPositionReached(void);
extern void setStepperPositionReached(unsigned char value);
extern void PowerOffStepper(void);
extern void setAbsZeroPosition(void);
extern void initZeroPosition(void);
extern void MoveStepperRamp(unsigned short speed,long n_step_motion,unsigned short delta_up, unsigned short delta_dw);  
extern void InitMoveStepperRamp(unsigned short set_point, unsigned short delta_up, unsigned short delta_dw, long n_step_motion);
extern void InitStartStepperRamp(void);
extern void StartStepperRamp(unsigned short speed, unsigned char direction, unsigned short delta_ramp_up);
extern unsigned char getDirection(long n_step_motion);
extern unsigned long getStepPositionCount(void);
extern void resetStepPositionCount(void);
extern long getPositionRef(void);
extern long getStepPosition(void);
#endif
