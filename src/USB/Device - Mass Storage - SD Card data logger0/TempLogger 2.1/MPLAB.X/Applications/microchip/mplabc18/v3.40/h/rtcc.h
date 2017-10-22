#ifndef __RTCC_H
#define __RTCC_H

/******************************************************************************
 *
 *                  RTCC PERIPHERAL LIBRARY HEADER FILE
 *
 ******************************************************************************
 * FileName:        rtcc.h
 * Dependencies:    See include below
 * Processor:       PIC18
 * Compiler:        MPLAB C18
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 * The software supplied herewith by Microchip Technology Incorporated
 * (the �Company�) for its PICmicro� Microcontroller is intended and
 * supplied to you, the Company�s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *****************************************************************************/

#include "p18f26j50.h"
#include "pconfig.h"
#include "GenericTypeDefs.h"
#include <stdio.h>
//This preprocessor conditional statement is to avoid unintended linking for unsuppported devices.
#if defined (RTCC_V1)


//typedef enum _BOOL { FALSE = 0, TRUE } BOOL;	// Undefined size


typedef union _WORD_VAL
{
    unsigned int Val;
    unsigned char v[2];
    struct
    {
        unsigned char LB;
        unsigned char HB;
    } byte;
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
        unsigned char b8:1;
        unsigned char b9:1;
        unsigned char b10:1;
        unsigned char b11:1;
        unsigned char b12:1;
        unsigned char b13:1;
        unsigned char b14:1;
        unsigned char b15:1;
    } bits;
} WORD_VAL, WORD_BITS;


// RTCC definitions
typedef enum
{
    RTCCFG_MASK_RTCEN      =   0x80, 
    RTCCFG_MASK_FRZ        =   0x40,
    RTCCFG_MASK_RTCWREN    =   0x20,
    RTCCFG_MASK_RTCSYNC    =   0x10,
    RTCCFG_MASK_HALFSEC    =   0x08,
    RTCCFG_MASK_RTCOE      =   0x04,
    RTCCFG_MASK_RTCPTR     =   0x03
}RTCCFG_MASK;

#define RTCCAL_MASK_CAL        0xFF

//Alarm definitions
typedef enum
{
    ALRMCFG_MASK_ALRMEN    =   0x80,
    ALRMCFG_MASK_CHIME     =   0x40,
    ALRMCFG_MASK_AMASK     =   0x3c,
    ALRMCFG_MASK_ALRMPTR   =   0x03
}ALRMCFG_MASK;

#define ALRMRPT_MASK_ARPT      0xFF

// accessing the RTCC/Alarm Value Register Window pointer bits
typedef enum
{
    RTCCPTR_MASK_SECMIN     =   0x00,
    RTCCPTR_MASK_HRSWEEK    =   0x01,
    RTCCPTR_MASK_DAYMON     =   0x02,
    RTCCPTR_MASK_YEAR       =   0x03   
}RTCCPTR_MASK;


// union/structure for read/write of time into the RTCC device
typedef union
{ 
    struct
    {
        unsigned char    rsvd;       // reserved for future use
        unsigned char    sec;        // BCD codification for seconds, 00-59
        unsigned char    min;        // BCD codification for minutes, 00-59
        unsigned char    hour;       // BCD codification for hours, 00-24
    }f;                     // field access
    unsigned char        b[4];       // BYTE access
    unsigned int         w[2];       // 16 bits access
    unsigned long         l;          // 32 bits access
}rtccTime;

// union/structure for read/write of date into the RTCC device
typedef union
{
    struct
    {
        unsigned char    wday;       // BCD codification for day of the week, 00-06
        unsigned char    mday;       // BCD codification for day of the month, 01-31
        unsigned char    mon;        // BCD codification for month, 01-12
        unsigned char    year;       // BCD codification for years, 00-99
    }f;                     // field access
    unsigned char        b[4];       // BYTE access
    unsigned int         w[2];       // 16 bits access
    unsigned long         l;          // 32 bits access   
}rtccDate;


// union/structure for read/write of time and date from/to the RTCC device
typedef union
{ 
    struct
    {
        unsigned char    year;       // BCD codification for year, 00-99
        unsigned char    rsvd;       // reserved for future use
        unsigned char    mday;       // BCD codification for day of the month, 01-31
        unsigned char    mon;        // BCD codification for month, 01-12
        unsigned char    hour;       // BCD codification for hours, 00-24
        unsigned char    wday;       // BCD codification for day of the week, 00-06   
        unsigned char    sec;        // BCD codification for seconds, 00-59 
        unsigned char    min;        // BCD codification for minutes, 00-59
    }f;                     // field access
    unsigned char        b[8];       // BYTE access
    unsigned int         w[4];       // 16 bits access
    unsigned long         l[2];       // 32 bits access
}rtccTimeDate;



// valid values of alarm repetition for the RTCC device
typedef enum
{
    RTCC_RPT_HALF_SEC,      // repeat alarm every half second
    RTCC_RPT_SEC,           // repeat alarm every second
    RTCC_RPT_TEN_SEC,       // repeat alarm every ten seconds
    RTCC_RPT_MIN,           // repeat alarm every minute
    RTCC_RPT_TEN_MIN,       // repeat alarm every ten minutes
    RTCC_RPT_HOUR,          // repeat alarm every hour          
    RTCC_RPT_DAY,           // repeat alarm every day
    RTCC_RPT_WEEK,          // repeat alarm every week
    RTCC_RPT_MON,           // repeat alarm every month
    RTCC_RPT_YEAR           // repeat alarm every year (except when configured for Feb 29th.) 
}rtccRepeat;


#define MAX_MIN         	(0x59)/* BCD codification for minutes, 00-59 */
#define MAX_SEC         	(0x59) /* BCD codification for seconds, 00-59 */
#define MAX_WDAY        	(0x6)/* BCD codification for day of the week, 00-06 */
#define MAX_HOUR        	(0x24)/* BCD codification for hours, 00-24 */
#define MAX_MON         	(0x12)/* BCD codification for month, 01-12 */
#define MIN_MON         	(0x1)/* BCD codification for month, 0-1 */
#define MAX_MDAY        	(0x31) /* BCD codification for day of the month, 01-31 */
#define MIN_MDAY        	(0x1)/* BCD codification for day of the month, 0-1 */
#define MAX_YEAR        	(0x99)/* BCD codification for year, 00-99 */

 /*********************************************************************
 * Macro:           mRtccIsWrEn()
 * PreCondition:    None
 * Side Effects:    None
 * Overview:   Returns the value of RTCCFG.RTCWREN.  
 * Input:           None
 * Output:          Returns the value of RTCCFG.RTCWREN.         
 * Note:            None
 ********************************************************************/
#define mRtccIsWrEn()   (RTCCFGbits.RTCWREN)

 /*********************************************************************
 * Macro:           mRtccWrOff() 
 * PreCondition:    None
 * Side Effects:    None
 * Overview:   Macro to to clear the RTCCFG.RTCWREN.
 * Input:           None
 * Output:          Macro to to clear the RTCCFG.RTCWREN.
 * Note:            None
 ********************************************************************/
#define mRtccWrOff()   (RTCCFGbits.RTCWREN = 0)

 /*********************************************************************
 * Macro:           mRtccIsOn() 
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        Macro to turn the RTCC on. 
 * Input:           None
 * Output:          value of RTCCFG.RTCEN 
 * Note:            None
 ********************************************************************/
#define mRtccIsOn()   (RTCCFGbits.RTCEN)

 /*********************************************************************
 * Macro:           mRtccOff() 
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        Macro to turn the RTCC off.
 * Input:           None
 * Output:          None 
 * Note:            None
 ********************************************************************/
#define mRtccOff()   (RTCCFGbits.RTCEN=0)

 /*********************************************************************
 * Macro:           mRtccOn() 
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        Macro helper to turn the RTCC on.
 * Input:           None
 * Output:          None 
 * Note:            Setting RTCCFG.RTCEN to 1 clears the RTCWREN, RTCSYNC,
 *                  HALFSEC, RTCOE and the ALRMCFG register.
 *                  TODO: check this is TRUE for PIC24F too.
 ********************************************************************/
#define mRtccOn()   (RTCCFGbits.RTCEN=1)

 /*********************************************************************
 * Macro:           mRtccIsAlrmEnabled() 
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        Inline helper.
 * Input:           None
 * Output:          value of ALRMCFG.ALRMEN
 * Note:            None
 ********************************************************************/
#define mRtccIsAlrmEnabled()   (ALRMCFGbits.ALRMEN)

 /*********************************************************************
 * Macro:           mRtccAlrmEnable() 
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        Macro to set the ALRMCFG.ALRMEN.
 * Input:           None
 * Output:          None 
 * Note:            None
 ********************************************************************/
#define mRtccAlrmEnable()   (ALRMCFGbits.ALRMEN=1)

 /*********************************************************************
 * Macro:           mRtccAlrmDisable()
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        Macro to clear the ALRMCFG.ALRMEN.
 * Input:           None
 * Output:          None 
 * Note:            None
 ********************************************************************/
#define mRtccAlrmDisable()   (ALRMCFGbits.ALRMEN=0)

 /*********************************************************************
 * Macro:           mRtccIsSync()
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        Macro read the status of the sync signal.
 * Input:           None
 * Output:          TRUE if sync asserted, FALSE otherwise.
 * Note:            None
 ********************************************************************/
#define mRtccIsSync()   (RTCCFGbits.RTCSYNC)

 /*********************************************************************
 * Macro:           mRtccWaitSync()
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        Macro to wait until RTCCFG.RTCSYNC signals read/write is safe.
 * Input:           None
 * Output:          None 
 * Note:            In order to be sure that the write/read op is safe, interrupts 
 *                  should be disabled or kept very short (worst case scenario, sync
 *                  can be asserted for 32 RTCC clocks,i.e. almost 1ms, so it's not 
 *                  advisable to disable the interrupts for such a long period
 *                  of time. Care must be taken under these circumstances).
 ********************************************************************/
#define mRtccWaitSync()   while((BOOL)RTCCFGbits.RTCSYNC)

 /*********************************************************************
 * Macro:           mRtccIs2ndHalfSecond() 
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        Macro to read the half-second status bit.
 * Input:           None
 * Output:          TRUE if it's the second half period of a second, FALSE otherwise. 
 * Note:            None
 ********************************************************************/
#define mRtccIs2ndHalfSecond()   (RTCCFGbits.HALFSEC)

 /*********************************************************************
 * Macro:           mRtccClearRtcPtr()
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        Macro to clear the RTCC Value Register Window Pointer bits.
 * Input:           None
 * Output:          None 
 * Note:            It sets the pointer to the min/sec slot
 ********************************************************************/
#define mRtccClearRtcPtr()   (RTCCFG&=~RTCCFG_MASK_RTCPTR)

 /*********************************************************************
 * Macro:           mRtccSetRtcPtr(mask)
 * PreCondition:    mask has to be valid member of the RTCCPTR_MASK
 * Side Effects:    None
 * Overview:        Macro to set the RTCC Value Register Window Pointer
 *                  bits to the desired value.
 * Input:           mask : enum RTCCPTR_MASK defined in "Rtcc.h" , value of the
 *                         pointer mask 
 * Output:          None 
 * Note:            macro mRtccClearRtcPtr() must be called prior to this macro
 ********************************************************************/
#define mRtccSetRtcPtr(mask)   (RTCCFG|=mask)

 /*********************************************************************
 * Macro:           mRtccSetAlrmPtr(mask)
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        Macro to set the Alarm Value Register Window Pointer bits.
 * Input:           None
 * Output:          None
 * Note:            It sets the pointer to the slot beyond the ALRMDAY
 *                  macro mRtccClearAlrmPtr() must be called prior to this macro
********************************************************************/
#define mRtccSetAlrmPtr(mask)   (ALRMCFG |= mask)

 /*********************************************************************
 * Macro:           mRtccClearAlrmPtr() 
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        Macro to clear the Alarm Value Register Window Pointer bits.
 * Input:           None
 * Output:          None
 * Note:            It sets the pointer to the slot beyond the ALRMDAY
 ********************************************************************/
#define mRtccClearAlrmPtr()   (ALRMCFG&=~ALRMCFG_MASK_ALRMPTR)


 /*********************************************************************
 * Macro :          mRtccGetChimeEnable()
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        The Macro  returns the chime alarm of the RTCC device.
 * Input:           None
 * Output:          current status of the alarm chime 
 * Note:            None
 ********************************************************************/
#define mRtccGetChimeEnable()   (ALRMCFGbits.CHIME)

 /*********************************************************************
 * Macro :          mRtccGetCalibration() 
 * PreCondition:    None 
 * Side Effects:    None
 * Overview:        The Macro  returns the value that the RTCC uses in the
 *                  auto-adjust feature, once every minute.
 *                  The calibration value is a signed 10 bits value, [-512, +511].
 * Input:           None 
 * Output:          Current value of the RTCC calibration field.
 * Note:            None
 ********************************************************************/
#define mRtccGetCalibration()   (RTCCAL)

 /*********************************************************************
 * Macro :          mRtccSetClockOe(enable)
 * PreCondition:    None 
 * Side Effects:    None
 * Overview:        The Macro enables/disables the Output Enable pin of the RTCC. 
 * Input:           enable - the desired status of the Output Enable pin 
 * Output:          None
 * Note:            None
 ********************************************************************/
#define mRtccSetClockOe(enable)   (RTCCFGbits.RTCOE=enable)

 /*********************************************************************
 * Macro :          mRtccGetClockOe()
 * PreCondition:    None 
 * Side Effects:    None
 * Overview:        The Macro  returns the enabled/disabled status of the Output
 *                  Enable pin of the RTCC. 
 * Input:           None 
 * Output:          1 if Output Enable is enabled, 0 otherwise.
 * Note:            None
 ********************************************************************/
#define mRtccGetClockOe()   (RTCCFGbits.RTCOE)

 /*********************************************************************
 * Macro :          mRtccSetInt(enable)
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        This Macro  enables/disables the RTCC event interrupts.
 * Input:           enable - enable/disable the interrupt
 * Output:          None
 * Note:            None.
 ********************************************************************/
#define mRtccSetInt(enable)   (PIE3bits.RTCIE=enable)

/*******************************************************************
Macro       : mRtcc_Clear_Intr_Status_Bit

Include     : Rtcc.h 

Description : Macro to Clear RTCC Interrupt Status bit 

Arguments   : None 

Remarks     : None 
*******************************************************************/
#define mRtcc_Clear_Intr_Status_Bit     (PIR3bits.RTCCIF = 0)

 /*********************************************************************
 * Macro:           mRtccGetAlarmRpt()
 * PreCondition:    None 
 * Side Effects:    None
 * Overview:        The macro returns the current RTCC alarm repeat rate.
 * Input:           None 
 * Output:          The value of the current alarm repeat rate. 
 * Note:            None
 ********************************************************************/
#define mRtccGetAlarmRpt()   (ALRMCFGbits.AMASK)

 /*********************************************************************
 * Macro:           mRtccGetAlarmRptCount()
 * PreCondition:    None 
 * Side Effects:    None
 * Overview:        The macro reads the RTCC alarm repeat counter.
 * Input:           None 
 * Output:          the current alarm repeat count 
 * Note:            None
 ********************************************************************/
#define mRtccGetAlarmRptCount()   (ALRMRPT)

 /*********************************************************************
 * Macro:           mRtccGetCalibration() 
 * PreCondition:    None
 * Side Effects:    None
 * Overview:        The macro returns the value that the RTCC uses in the
 *                  auto-adjust feature, once every minute.
 *                  The calibration value is a signed 10 bits value, [-512, +511]. 
 * Input:           None 
 * Output:          Current value of the RTCC calibration field.
 * Note:            None
 ********************************************************************/
#define mRtccGetCalibration()   (RTCCAL)
 
extern void RtccInitClock(void) ;
 
extern void RtccReadAlrmDate(rtccDate* pDt) ;
 
extern void  RtccReadAlrmTime(rtccTime* pTm);
 
extern void RtccReadAlrmTimeDate(rtccTimeDate* pTD) ;
 
extern void  RtccReadDate(rtccDate* pDt) ;

extern void RtccReadTime(rtccTime* pTm) ;
 
extern void RtccReadTimeDate(rtccTimeDate* pTD) ;

extern void RtccSetAlarmRpt(rtccRepeat rpt, BOOL dsblAlrm) ;
 
extern void RtccSetAlarmRptCount(unsigned char rptCnt, BOOL dsblAlrm) ;

extern void RtccSetCalibration(int drift) ;
 
extern void RtccSetChimeEnable(BOOL enable, BOOL dsblAlrm) ;
 
extern BOOL RtccWriteAlrmDate(const rtccDate* pDt) ;
 
extern BOOL RtccWriteAlrmTime(const rtccTime* pTm) ;

extern BOOL  RtccWriteAlrmTimeDate(const rtccTimeDate* pTD) ;
 
extern BOOL  RtccWriteDate(const rtccDate* pDt , BOOL di);
 
extern BOOL  RtccWriteTime(const rtccTime* pTm , BOOL di) ;

extern BOOL  RtccWriteTimeDate(const rtccTimeDate* pTD , BOOL di) ;

extern void  RtccWrOn(void);

//#else		//This preprocessor conditional statement is to avoid unintended linking for unsuppported devices.
//#error "Selected device does not support this Module"
#endif

//*********** These declarations are of simulated RTCC (using timer1) functions.**************(Note: place this outside #if def RTCC_V1_1) generic,available for all devices *********  
//void Open_RTCC(void);
//void Close_RTCC(void);
//unsigned char update_RTCC(void);

#endif /* __RTCC_H */
