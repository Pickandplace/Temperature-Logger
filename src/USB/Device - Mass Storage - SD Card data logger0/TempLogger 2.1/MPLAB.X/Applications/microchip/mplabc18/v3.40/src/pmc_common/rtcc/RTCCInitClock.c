 
#include "rtcc.h"

#if defined (RTCC_V1) 
 /*********************************************************************
 Function:        void RtccInitClock(void)

 PreCondition:    None
                  
 Input:           None
 
 Output:          None
 
 Side Effects:    Enables the secondary oscillator from Timer1
 
 Overview:        The function initializes the RTCC device. It starts the RTCC clock,
                  sets the RTCC Off and disables RTCC write. Disables the OE.
                   
 Note:            None
 ********************************************************************/
void RtccInitClock(void)
{
   // enable the Secondary Oscillator
#if defined (RTCC_SFR_V1)
	T1CONbits.SOSCEN = 1;
#else   
   T1CONbits.TMR1CS0 = 0;
   T1CONbits.TMR1CS1 = 1;
   T1CONbits.T1OSCEN = 1;
#endif

   RTCCFG = 0x0;
   RTCCAL = 0x00;
   if(mRtccIsOn())
   {
      if(!mRtccIsWrEn())
      {
          RtccWrOn();
      }
       mRtccOff();
   }
   
   mRtccWrOff();
}

//#else
////#warning "Selected device does not support this function"
#endif

