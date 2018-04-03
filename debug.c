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

/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*                    include files                                           */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/

#include "debug.h"
#include "RTDM.h"



/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*                    typedef definitions                                     */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/



/******************************************************************************/
/******************************************************************************/
/*                                                                            */
/*                    global variables                                        */
/*                                                                            */
/******************************************************************************/
/******************************************************************************/
#ifdef RTDM
    DMCIFLAGS DMCIFlags;
    int RecorderBuffer1[DATA_BUFFER_SIZE];  //Buffer to store the data samples for the DMCI data viewer Graph1
    int RecorderBuffer2[DATA_BUFFER_SIZE];	//Buffer to store the data samples for the DMCI data viewer Graph2
    int RecorderBuffer3[DATA_BUFFER_SIZE];	//Buffer to store the data samples for the DMCI data viewer Graph3
    int RecorderBuffer4[DATA_BUFFER_SIZE];	//Buffer to store the data samples for the DMCI data viewer Graph4
    
    int * PtrRecBuffer1 = &RecorderBuffer1[0];	//Tail pointer for the DMCI Graph1
    int * PtrRecBuffer2 = &RecorderBuffer2[0];	//Tail pointer for the DMCI Graph2
    int * PtrRecBuffer3 = &RecorderBuffer3[0];	//Tail pointer for the DMCI Graph3
    int * PtrRecBuffer4 = &RecorderBuffer4[0];	//Tail pointer for the DMCI Graph4
    int * RecBuffUpperLimit = RecorderBuffer4 + DATA_BUFFER_SIZE -1;	//Buffer Recorder Upper Limit
    int	SnapCount = 0;
    int SnapShotDelayCnt = 0;
    int SnapShotDelay = SNAPDELAY;

#endif


#ifdef	SNAPSHOT
    SNAPFLAGS SNAPFlags;
    int	SnapBuf1[DATA_BUFFER_SIZE];
    int SnapBuf2[DATA_BUFFER_SIZE];
    int SnapBuf3[DATA_BUFFER_SIZE];
    int SnapBuf4[DATA_BUFFER_SIZE];		
    int	SnapCount = 0;  
    int SnapShotDelayCnt = 0;
    int SnapShotDelay = SNAPDELAY;

#endif

/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: DBG_Init                                                    */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Initialize the Debugger functionality                         */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void DBG_Init(void)
{
    /* call RTDM specific if defined */
#ifdef RTDM
	RTDM_Start();	//RTDM start function
					// Overview: 
					// Here is where the RTDM code initilizes the UART to be used to
					// exchange data with the host PC
					// Note:	
					// Some processors may have 2 UART modules, that is why it is required to
					// specify wich UART module is going to be used by RTDM	
					
    DMCIFlags.Recorder =1;
#endif

#ifdef SNAPSHOT
    
    SNAPFlags.DoSnap = 0;
    SNAPFlags.SnapDone = 0;
    
#endif

}
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: DBG_SyncComm                                                */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Sychronize the communication between the debugger funct       */
/* running on the embedded to the host                                        */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void DBG_SyncComm(void)
{
#ifdef RTDM

	RTDM_ProcessMsgs();	//RTDM start function

#endif
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: DBG_SnapStart                                              */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Snap start - enable the data acquisition                      */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void DBG_SnapStart(void)
{
#ifdef SNAPSHOT
    SNAPFlags.DoSnap = 1;
	SNAPFlags.SnapDone = 0;
    
    #ifdef  SNAP_TRIGGERED
	    SnapCount = 0;
	#endif   
#endif
    
}    
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: DBG_SnapUpdate                                              */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Snap the values sent to host for debug purposes               */
/* update the internal debbuger variables (snap buffers) at certain timing    */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void DBG_SnapUpdate(void)
{
#ifdef	SNAPSHOT
    if((SNAPFlags.DoSnap == 1)&&( SNAPFlags.SnapDone!=1 ))
    {
            if (SnapShotDelayCnt >= SnapShotDelay)
            {
                 SnapShotDelayCnt = 0;
    		     SnapBuf1[SnapCount] = SNAP1;
    		     SnapBuf2[SnapCount] = SNAP2;  
    		     SnapBuf3[SnapCount] = SNAP3;  
    		     SnapBuf4[SnapCount] = SNAP4;  
    		     SnapCount++;    
    	         if(SnapCount >= DATA_BUFFER_SIZE)
                 {
					SnapCount = 0;
					SnapShotDelayCnt = 0;
	                SNAPFlags.SnapDone=1;
	                SNAPFlags.DoSnap = 0;				
	             }
    	         
    	         #ifndef SNAP_TRIGGERED
    	         //used for visibility of the start/end of a recording
    	         SnapBuf1[SnapCount] = 0;
    		     SnapBuf2[SnapCount] = 0;
    		     SnapBuf3[SnapCount] = 0;
    		     SnapBuf4[SnapCount] = 0;
    	         #endif
            }
    }        
#endif                 

#ifdef RTDM
    	if(DMCIFlags.Recorder)
    	{
        	    //filling the buffer
    		if(SnapShotDelayCnt >= SnapShotDelay)
    		{
    			SnapShotDelayCnt = 0;
    			*PtrRecBuffer1++ 	= SNAP1;
    			*PtrRecBuffer2++	= SNAP2;
    			*PtrRecBuffer3++	= SNAP3;
    			*PtrRecBuffer4++	= SNAP4;
    			
    			//if buffers are full reset the buffer index and disable recording
    			if(PtrRecBuffer4 > RecBuffUpperLimit)
    			{
    				PtrRecBuffer1 = RecorderBuffer1;
    				PtrRecBuffer2 = RecorderBuffer2;
    		        PtrRecBuffer3 = RecorderBuffer3;
    		        PtrRecBuffer4 = RecorderBuffer4;
    		        DMCIFlags.Recorder = 0;
    		    }   
    		}
    	}
#endif

}
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/******************************************************************************/
/* Function name: DBG_StateUpdate                                             */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Update the debugger's internal state variables, on which the  */
/* data snap depends or the communcation between the embedded and host.       */
/******************************************************************************/
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void DBG_StateUpdate(void)
{
#if  defined (RTDM)  || defined (SNAPSHOT)
     /* delay count update for both RTDM & SNAP */
     SnapShotDelayCnt++;
#endif
}
