#ifndef DEBUG_H
#define DEBUG_H
/**********************************************************************
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
******************************************************************************/

#include "stepper.h"


#define SNAPSHOT
#define RTDM
#define DATA_BUFFER_SIZE 160    //data buffer length
#define SNAPDELAY	10          // In number of PWM Interrupts (25us)
    #define	SNAP1		current1
    #define	SNAP2		current2
	#define	SNAP3	    stepAmplitudeRef1 
    #define	SNAP4		stepAmplitudeRef2

#ifdef RTDM
    #undef SNAPSHOT
#endif

#undef SNAP_TRIGGERED

#ifdef RTDM

    typedef struct DMCIFlags{
    		    unsigned Recorder : 1;	// Flag needs to be set to start buffering data
    			unsigned unused : 15;  
    } DMCIFLAGS;
extern    DMCIFLAGS DMCIFlags;
    
#endif

#ifdef	SNAPSHOT
    typedef struct SNAPFlags{
    		    unsigned DoSnap : 1;	// Flag needs to be set to start buffering data
    		    unsigned SnapDone:1;    // Flag indicating snap completed
    			unsigned unused : 14;  
    } SNAPFLAGS;
extern    SNAPFLAGS SNAPFlags;

#endif

void DBG_Init(void);
void DBG_SyncComm(void);
void DBG_SnapStart(void);
void DBG_SnapUpdate(void);
void DBG_StateUpdate(void);
void DBG_IDLECounter(void);

#endif
