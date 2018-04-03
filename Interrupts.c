/**/
/*============================================================================*/
/**
**      @file      interrupts.c
**
**      @brief     Interrupt routines
**
**      @version   Alfa color tester
**/
/*============================================================================*/
/**/

#include "stepper.h"
#include "timermg.h"
#include "macro.h"
#include "ram.h"


int BufferCurrent[20];

/******************************************************************************
* Function:     _T1Interrupt()
*
* Output:		None
*
* Overview:		A Timer1 overflow event flag is set (uGF.timer1_InterruptFlag)
*               that signals the StateMachine to start Speed Control
*
* Note:			None
*******************************************************************************/

/* Timer 1 Interrupt */
void __attribute__((__interrupt__,auto_psv)) _T1Interrupt(void)
{
	IFS0bits.T1IF = 0; 		            //Clear Timer 1 Interrupt Flag  
	uGF.timer1_InterruptFlag = 1;		//enable speed and position controller trigger flag
	TimeBase++;
}

/******************************************************************************
* Function:  _T3Interrupt()
*
* Output:		None
*
* Overview:	32bit Timer2/3 interrupt controls the step time duration
*
* Note:			None
*******************************************************************************/

void __attribute__((__interrupt__,auto_psv)) _T3Interrupt(void)
{
  
	IFS0bits.T3IF = 0; 		// Clear Timer 1 Interrupt Flag
	
	PORTCbits.RC3 = 1;
	
	//load 32 bit timer LSW with low word of speedOutLong
	PR2 = (int)(speed2Time);	
	//load 32 bit timer MSW with high word of speedOutLong
	PR3 = (int)(speed2Time >> 16);
	
	//Time critical section -  disable ADC interrupt since it has higher priority than Timer2/3
	
	IEC0bits.AD1IE = 0;		// disable ADC interrupt
	
	//make sure the new limit value to be written to Timer2/3 is lower than the current value of the timer
	if(PR3 <= TMR3)
	{
	  //if the new timer limit value is lower than teh current timer counter then reset the counters
	  if(PR2<=TMR2)
	  {
	    TMR2=0;
	    TMR3=0;
	  }
	}
	
	IEC0bits.AD1IE = 1;		// enable ADC interrupt
	
	if (directionFlag != directionFlagOld)
	{
    if (directionFlagOld)
    {
      stepCount = 1023-stepCount;
    }  
    directionFlagOld = directionFlag;
  }  
  
  stepCount += stepSizeCount;                 //calculate new microstep counter value based on step size
  
  if(StepCountVolume <= NUM_STEP_PER_ML)
  {
    StepCountVolume += stepSizeCount;   
  }
  else
  {
    StepCountVolume = 0;
  }  
  
  StepPositionCount+= stepSizeCount; 
  
  if(stepCount >= TABLE_SIZE_MUL4)            //if the step counter has reached the end of one sinewave period then reset it
	{
	  
	  //if stepCount has overflown then a step size change has been made. Add the position difference.
	  if((stepCount-TABLE_SIZE_MUL4) >0)
	  {
	    //add lost position
	    position += directionFlag * (stepCount - TABLE_SIZE_MUL4);
	  }
	  
	  //reset micro-stepping counter
	  stepCount = 0;
	  
	  #ifdef BIPOLAR
	  
	  if(uGF.currentControlLoop == OFF)                      //Fixed current mode is not used in closed loop
	  {
	    if(uGF.controlMode == FIXED_CURRENT)                //Fixed current mode enabled
	    {
	      if((currentMaxAmplitude)< OPEN_LOOP_CURRENT)        //Current amplitude is smaller than rated/desired current
	      {
	        //increase the duty cycle only if the motor speed is below the open loop speed limit
	        //if the speed is higher then the motor is in transition from closed loop high speed to open loop
	        if (speedOut <= MAXIMUM_SPEED)                           
	          maxRefValue+= 1;                             //increase the maximum allowed dutycycle
	        if(maxRefValue > PWM_MAX)                        //limit the dutycyle up to PWM_MAX
	          maxRefValue = PWM_MAX;
	      }
	      else                                                 //current amplitude is greater than rated/desired current
	      {
	        maxRefValue-= 1;                                 //decrease the maximum allowed dutycycle
	        if(maxRefValue < OPEN_LOOP_VOLTAGE)              //limit the maximum PWM dutycyle to OPEN_LOOP_VOLTAGE (corresponds to rated/desired motor current)
	        {
	          maxRefValue = OPEN_LOOP_VOLTAGE;
	        }
	      }
	      currentMaxAmplitude = 0;                            //reset the amplitude measurement for another cycle
	    }
	    else                                                    //Fixed Voltage mode enabled
	    {
	      if(maxRefValue > OPEN_LOOP_VOLTAGE)                 //gradually decrease maxVoltage after a change from ...
	      {                                                   //...FIXED_CURRENT mode to FIXED_VOLTAGE to prevent the rotor from stalling
	        maxRefValue-= 10;                               //decrease in steps of 1% of PWM duty cycle
	      }
	      else                                                //limit the PWM dutycyle to the original value of OPEN_LOOP_VOLTAGE (corresponds to rated/desired motor current)
	      {
	        maxRefValue = OPEN_LOOP_VOLTAGE;                 
	      }	
	    }
	  }
	  #endif  //#ifdef BIPOLAR
	  
	}
	  /*Update maxRef based if speedOit change*/
	  if ((uGF.currentControlLoop == OFF) && (enableSpeedUpdating))
	  {
	    enableSpeedUpdating = FALSE;
	    maxRefValue =  maxRefValueInit;
	  } 
	  
	CalcStep();                     //calculate next step amplitude value
	
	//Increment/Decrement the position counter based on the motor turning direction
	if( state==STATE_RUN)
	{
	  position += directionFlag * (TABLE_SIZE >>stepSize);	  
	}
	
}//end of Timer3 interrupt


/******************************************************************************
* Function:     _FLTA1Interrupt()
*
* Output:		None
*
* Overview:		Overcurrent events are processed here
*
* Note:			None
*******************************************************************************/

void __attribute__((__interrupt__,auto_psv)) _FLTA1Interrupt(void)
{
  //in case an overcurrent fault is detected in fixed current mode, reset the maximum dutycyle to the value corresponding to the rated/desired current
  if(uGF.currentControlLoop == OFF)
    maxRefValue = OPEN_LOOP_VOLTAGE;
	IFS3bits.FLTA1IF = 0;       //clear interrupt flag
	IFS4bits.FLTA2IF = 0;       //clear interrupt flag
}

/******************************************************************************
* Function:     _FLTA2Interrupt()
*
* Output:		None
*
* Overview:		Overcurrent events are processed here
*
* Note:			None
*******************************************************************************/

void __attribute__((__interrupt__,auto_psv)) _FLTA2Interrupt(void)
{
	//in case an overcurrent fault is detected in fixed current mode, reset the maximum dutycyle to the value corresponding to the rated/desired current
	if(uGF.currentControlLoop == OFF)
	  maxRefValue = OPEN_LOOP_VOLTAGE;
	IFS3bits.FLTA1IF = 0;       //clear interrupt flag
	IFS4bits.FLTA2IF = 0;       //clear interrupt flag
}



/******************************************************************************
* Function:     _ADC1Interrupt()
*
* Output:		None
*
* Overview:		Current, POT and DC_BUS measurements are done here. 
*             PWM duty cycles are updated in the ADC interrupt
*
* Note:			None
*******************************************************************************/

void __attribute__((__interrupt__,auto_psv)) _ADC1Interrupt(void)
{
	#ifdef MEASURE_CHANGE_PAGE_TIME
  PORTBbits.RB7=1;
  #endif
  //clear interrupt flag
	IFS0bits.AD1IF = 0;
	
	if(state == STATE_RUN)
	{
	  // PWM timer is counting up
	  if(P1TMRbits.PTDIR == 0)    
	  {
	    //read currents when PWMxHx are ON
	    if(setDir2HW==FORWARD)
	      current2 = ADC1BUF2;	//AN1 input
	    if(setDir1HW==FORWARD)
	      current1 = ADC1BUF1;	//AN0 input
	    
	    /* Real- time ADC value can be used although it must filtered; if needed use the following:
	    dcBus = ((int)ADC1BUF3>>1)+16384; //AN2 input */
	    dcBus = VOLTAGE;  
	    
	    //read POT speed
	    potSpeed = ADC1BUF0;	    //AN3 input
	    
	    //Fixed voltage and Fixed current modes
	    if(uGF.currentControlLoop == OFF)
	    {
	      #ifdef BIPOLAR
	      //Calculate the maximum current amplitude of both windings
	      if(uGF.controlMode == FIXED_CURRENT)
	      {
	        if(current1 > currentMaxAmplitude)
	        {
	          currentMaxAmplitude = current1;
	        }
	        if(current2 > currentMaxAmplitude)
	        {
	          currentMaxAmplitude = current2;
	        }
	        
	        if (index_current < 19)
	        {
	          index_current++;
	        }  
	        else
	        {
	          index_current = 0;
	        }  
	        BufferCurrent[index_current]=currentMaxAmplitude;
	      }
	      
	      //Manage alternate decay changes
	      if(uGF.decayMode == ALTERNATE_DECAY)
	      {
	        //winding 1 alternate decay
	        if (stepChangeDecayCounter1 > 0)                            //if the Counter is 0 no action is taken
	        {
	          if(stepChangeDecayCounter1 <= stepChangeDecayNumber1)   //alternate decay periods have not finished, increment counter
	          {
	            stepChangeDecayCounter1++;                          //increment the counter 
	            
	          }
	          else                                                	//alternate decay periods finished, switch to base decay
	          {
	            //reset periods to zero
	            stepChangeDecayNumber1 = 0;
	            decay1 = baseDecay;             //return to the base decay mode                                                                                         
	            stepChangeDecayCounter1 = 0;    //reset the counter to wait for next trigger
	            CalcDecay();                    //calculate PxOVDCON register values for the new decay mode and winding directions
	          }
	        }//end of alternate decay for winding 1
	        
	        //winding 2 alternate decay
	        if (stepChangeDecayCounter2 > 0)                            //if the Counter is 0 no action is taken
	        {
	          if(stepChangeDecayCounter2 <= stepChangeDecayNumber2)   //alternate decay periods have not finished, increment counter
	          {
	            stepChangeDecayCounter2++;                          //increment the counter 
	          }
	          else                                                	//alternate decay periods finished, switch to base decay
	          {
	            //reset periods to zero
	            stepChangeDecayNumber2 = 0;
	            decay2 = baseDecay;             //return to the base decay mode
	            stepChangeDecayCounter2 = 0;    //reset the counter to wait for next trigger
	            CalcDecay();                    //calculate PxOVDCON register values for the new decay mode and winding directions
	          }	
	        }//end of alternate decay for winding 2	
	      }//end of alternate decay 
	      
	      #endif  //#ifdef BIPOLAR
	      
	      //load PWM output values with calculated values
	      pwmOut1 = voltageOut1;
	      pwmOut2 = voltageOut2;
	      
	      //make sure the output values are in the acceptable PWM range
	      if(pwmOut1>PWM_MAX)
	        pwmOut1 = PWM_MAX;
	      
	      if(pwmOut2>PWM_MAX)
	        pwmOut2 = PWM_MAX;
	      
	    }
	    //end of fixed current and fixed voltage modes          
	    //Closed loop PI control (currentControlLoop == ON)
	    #ifdef BIPOLAR
	    else        
	    {      
	      //PI controller for winding 1
	    
	      pwmOut1 = CalcPI1(stepAmplitudeRef1,current1);
	      pwmOut1 = (int)((((long)pwmOut1*DC_2_PWM))>>15);        //convert from voltage scale to PWM duty cycle
	      
	      //make sure the output value is in the acceptable PWM range
	      if(pwmOut1>PWM_MAX)
	        pwmOut1 = PWM_MAX;
	      if(pwmOut1<-PWM_MAX)
	        pwmOut1 = -PWM_MAX;
        
	         
	      //limit the minimum PWM duty cycle and set the appropriate winding driving direction
	      if(pwmOut1 >=0)
	      {
	        setDir1 = FORWARD; 
	        if (pwmOut1<MINIMUM_DUTY)
	          pwmOut1 = MINIMUM_DUTY;
	      }
	      else             //if the output is negative reverse drive direction and recalculate the output
	      {			
	        pwmOut1 = pwmOut1 + PWM_MAX;        
	        setDir1 = REVERSE;	
	        if (pwmOut1>(PWM_MAX-MINIMUM_DUTY))
	          pwmOut1 = PWM_MAX-MINIMUM_DUTY;
	      }    
	      
	      CalcDecay();            //calculate PxOVDCON register values for the new decay mode and winding directions   
	       
	    } 
	    //end of closed loop current control
	    #endif  //#ifdef BIPOLAR
	    
	    //setDir1HW is only changed if the winding is driven in the positive direction
	    // to prevent the current from being read before the HW actually reverses the winding direction
	    if(setDir1==0 )
	      setDir1HW = setDir1; 
	    
	    //setDir2HW is always updated sice setDir2 is changed on the other PWM phase
	    setDir2HW = setDir2;    
	    
	    //Load PWM registers with the calculated values
	    SetPWM();	
    	
	    //DMCI sequence - write data to buffers
      #ifdef RTDM_PROCESS_MSG
      DBG_SnapStart();        
      DBG_SnapUpdate();
      DBG_StateUpdate();
      #endif
      
      //Set ADC trigger on PWM timer counting down, with delay for proper current measurement 
      SEVTCMP = PWM_FCY-PWM_ACQ_DELAY;    //the delay allows the current to rise through the measurement filters
      P1SECMPbits.SEVTDIR = 1;            //set trigger on PWM timer counting down
      
    }	 
    //end of PWM timer counting up
    // PWM timer is counting down
    else
    {
      //read currents when PWMxLx are ON
      if(setDir2HW==REVERSE)
        current2 = -ADC1BUF2;	        //AN1 input
      if(setDir1HW==REVERSE)
        current1 = -ADC1BUF1;		    //AN0 input
      
      //Fixed voltage and Fixed current modes 
      if(uGF.currentControlLoop == OFF)
      {  	
        
        
        //load output values with calculated values
        pwmOut1 = voltageOut1;
        pwmOut2 = voltageOut2;
        
        
        // make sure the output values are in the acceptable PWM range
        if(pwmOut1>PWM_MAX)
          pwmOut1 = PWM_MAX;
        
        if(pwmOut2>PWM_MAX)
          pwmOut2 = PWM_MAX;
      }
      //end of fixed current and fixed voltage modes          
      //Closed loop PI control (currentControlLoop == ON)
      #ifdef BIPOLAR 
      else
      {
        
        //PI controller for winding 2
        pwmOut2 = CalcPI2(stepAmplitudeRef2,current2);
        pwmOut2 = (int)((((long)pwmOut2*DC_2_PWM))>>15);        //convert from voltage scale to PWM duty cycle
        
        //make sure the output value is in the acceptable PWM range
        if(pwmOut2>PWM_MAX)
          pwmOut2 = PWM_MAX;
        if(pwmOut2<-PWM_MAX)
          pwmOut2 = -PWM_MAX;
        
       
        //limit the minimum PWM duty cycle and set the appropriate winding driving direction
        if(pwmOut2 >=0)
        {
          setDir2 = FORWARD; 
          if (pwmOut2<MINIMUM_DUTY)
            pwmOut2 = MINIMUM_DUTY;
        }
        else                //if the output is negative reverse drive direction and recalculate the output
        {			
          pwmOut2 = pwmOut2 + PWM_MAX;        
          setDir2 = REVERSE;	
          if (pwmOut2>(PWM_MAX-MINIMUM_DUTY))
            pwmOut2 = PWM_MAX-MINIMUM_DUTY;
        }	 

        CalcDecay();        //calculate PxOVDCON register values for the new decay mode and winding directions
     
      }
      //end of closed loop current control
      #endif  //#ifdef BIPOLAR
      
      //setDir2HW is only changed if the winding is driven in the positive direction
      // to prevent the current from being read before the HW actually reverses the winding direction
      if(setDir2==1)
        setDir2HW = setDir2; 	
      
      //setDir2HW is always updated sice setDir2 is changed on the other PWM phase
      setDir1HW = setDir1; 
      
      //Load PWM registers with the calculated values
      SetPWM();
      
      //DMCI sequence - write data to buffers
      #ifdef RTDM_PROCESS_MSG
      DBG_SnapStart();        
      DBG_SnapUpdate();
      DBG_StateUpdate();
      #endif
      //Set ADC trigger on PWM timer counting up, with delay for proper current measurement 
      SEVTCMP = PWM_ACQ_DELAY;            //the delay allows the current to rise through the measurement filters
      P1SECMPbits.SEVTDIR = 0;            //set trigger on PWM timer counting up
    }
    //end of PWM timer counting down
  }	
  #ifdef MEASURE_CHANGE_PAGE_TIME
  PORTBbits.RB7=0;
  #endif
}
//end of ADC interrupt


