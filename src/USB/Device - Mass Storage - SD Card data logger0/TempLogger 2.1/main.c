/********************************************************************
 * Copyright (C) 2017 Jean Wlodarski
 * 
 * KaZjjW at gmailcom
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * The original code is from Microchip, under the Microchip license.
********************************************************************/

/** INCLUDES *******************************************************/

#include <stdio.h>

#include "USB/USB.h"
#include "HardwareProfile.h"
#include "MDD File System/SD-SPI.h"
#include "MDD File System/FSIO.h"
#include "i2c.h"
#include "./USB/usb_function_msd.h"
#include "rtcc.h"


/** CONFIGURATION **************************************************/

#pragma config WDTEN = OFF          //WDT disabled (enabled by SWDTEN bit)
#pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input)
#pragma config STVREN = ON          //stack overflow/underflow reset enabled
#pragma config XINST = OFF          //Extended instruction set disabled
#pragma config CPUDIV = OSC1        //No CPU system clock divide
#pragma config CP0 = OFF            //Program memory is not code-protected
#pragma config OSC = HSPLL          //HS oscillator, PLL enabled, HSPLL used by USB
#pragma config FCMEN = OFF          //Fail-Safe Clock Monitor disabled
#pragma config IESO = ON           //Two-Speed Start-up disabled
#pragma config WDTPS = 32768        //1:32768
#pragma config DSWDTOSC = INTOSCREF //DSWDT uses INTOSC/INTRC as clock
#pragma config RTCOSC = T1OSCREF    //RTCC uses T1OSC/T1CKI as clock

#pragma config DSBOREN = OFF        //Zero-Power BOR disabled in Deep Sleep
#pragma config DSWDTEN = OFF        //Disabled
#pragma config DSWDTPS = 8192       //1:8,192 (8.5 seconds)
#pragma config IOL1WAY = OFF        //IOLOCK bit can be set and cleared
#pragma config MSSP7B_EN = MSK7     //7 Bit address masking
#pragma config WPFP = PAGE_1        //Write Protect Program Flash Page 0
#pragma config WPEND = PAGE_0       //Start protection at page 0
#pragma config WPCFG = OFF          //Write/Erase last page protect Disabled
#pragma config WPDIS = OFF          //WPFP[5:0], WPEND, and WPCFG bits ignored 
#pragma config T1DIG = ON           //Sec Osc clock source may be selected
#pragma config LPT1OSC = ON        //Low power Timer1 mode





/** VARIABLES ******************************************************/

#pragma udata

extern DISK gDiskData;         // Global structure containing device information.
//#pragma udata grp1
//unsigned char SampleASCII[40];
//#pragma udata grp2
unsigned char TimeDateASCII[20];
BYTE lengthASCII;
BYTE WriteEnable;
rtccTimeDate TimeDate;
BYTE TimeState;
char Temp_Precision[34]="00061319253138445056636975818894"; //Array for the temperature value conversion
char SampleASCII[33];
DWORD LogBufferPtr, tempPtr;         //Pointer for the current sector of the drive.
WORD LogInnerBufferPtr;     //Pointer inside the buffer with the Temperature logs

BYTE State;
BYTE FullOS;

long periode;
extern  BYTE gDataBuffer[MEDIA_SECTOR_SIZE];
WORD LoggerID; //Used to store the logger ID (from 0 to 65535)
unsigned char idString[7]; //Contains the logger ID string

int BATT_POWER_SENSE;
BYTE StatePeriod;
WORD i,WordPtr1,WordPtr2;
int j;
WORD LedFreq, LedCounter;
BYTE_VAL STATUS_FLAG;

int OneWay; //used to make an exitable function while in the SW0 interrupt.

/* Standard Response to INQUIRY command stored in ROM 	*/
const ROM InquiryResponse inq_resp = {
	0x00,		// peripheral device is connected, direct access block device
	0x80,           // removable
	0x04,	 	// version = 00=> does not conform to any standard, 4=> SPC-2
	0x02,		// response is in format specified by SPC-2
	0x20,		// n-4 = 36-4=32= 0x20
	0x00,		// sccs etc.
	0x00,		// bque=1 and cmdque=0,indicates simple queueing 00 is obsolete,
			// but as in case of other device, we are just using 00
	0x00,		// 00 obsolete, 0x80 for basic task queueing
	{'P','c','k','P','l','a','c','e'
    },
	// this is the T10 assigned Vendor ID
	{'M','a','s','s',' ','S','t','o','r','a','g','e',' ',' ',' ',' '
    },
	{'0','0','0','1'
    }
};

extern const rom char STATUS_TIME_UPDATED;


/** PRIVATE PROTOTYPES *********************************************/
static void InitializeSystem(void);
void USBDeviceTasks(void);
void ProcessIO(void);

void YourHighPriorityISRCode(void);
void YourLowPriorityISRCode(void);
void USBCBSendResume(void);
void FillBootSector(void);
void FillFAT(void);
void FillFAT_msd(void);
void FillDirectory(void);
void FillSlack(void);
void FillMBR(void);
void FillDirectoryMSD(WORD LogFileSize);
void DtASCII(unsigned char * tabASCII);
void StartOneShotTemp(void);
int OneShotTempWait(void);
void OneShotTempRead(unsigned char* TempASCII);
void RtccSetNextAlarm(void);
void AcquisitionRoutine(void);
void StartLogging(void);
void StopLogging(void);
void AddFAT(WORD FatSector);
void MiniInit(void);
void EEPROM_Sleep(void);
void DelayLED(void);
extern void CloseSPIM( void );
void SlowClock(void);
void FastClock(void);
void EraseFileSystem(void);
void Delay2OSC10ms(void);
/** VECTOR REMAPPING ***********************************************/
#if defined(__18CXX)
	//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
	//the reset, high priority interrupt, and low priority interrupt
	//vectors.  However, the current Microchip USB bootloader 
	//examples are intended to occupy addresses 0x00-0x7FF or
	//0x00-0xFFF depending on which bootloader is used.  Therefore,
	//the bootloader code remaps these vectors to new locations
	//as indicated below.  This remapping is only necessary if you
	//wish to program the hex file generated from this project with
	//the USB bootloader.  If no bootloader is used, edit the
	//usb_config.h file and comment out the following defines:
	#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
	//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
	#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
	#else	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
	#endif
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	extern void _startup (void);        // See c018i.c in your C18 compiler dir
	#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
	void _reset (void)
	{
	    _asm goto _startup _endasm
	}
	#endif
	#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
	     _asm goto YourHighPriorityISRCode _endasm
	}
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
	     _asm goto YourLowPriorityISRCode _endasm
	}
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	//Note: If this project is built while one of the bootloaders has
	//been defined, but then the output hex file is not programmed with
	//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
	//As a result, if an actual interrupt was enabled and occured, the PC would jump
	//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
	//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
	//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
	//would effective reset the application.
	
	//To fix this situation, we should always deliberately place a 
	//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
	//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
	//hex file of this project is programmed with the bootloader, these sections do not
	//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
	//programmed using the bootloader, then the below goto instructions do get programmed,
	//and the hex file still works like normal.  The below section is only required to fix this
	//scenario.
	#pragma code HIGH_INTERRUPT_VECTOR = 0x08
	void High_ISR (void)
	{
	     _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#pragma code LOW_INTERRUPT_VECTOR = 0x18
	void Low_ISR (void)
	{
	     _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#endif	//end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

	#pragma code
	
	
	//These are your actual interrupt handling routines.
	#pragma interrupt YourHighPriorityISRCode
	void YourHighPriorityISRCode()
	{
          //  static BYTE SampleLength;
    /** USB INTERRUPT  ***************************************************/
         if((!LOGGING_ON)&&(USB_POWER))
          USBDeviceTasks();
          /** END USB INTERRUPT  *
    /**************************************************************************/
        
       /** RTCC Alarm ***************************************************/

          if(PIR3bits.RTCCIF)
            {
            PIR3bits.RTCCIF = 0; //Erase the RTCC interrupt flag
            if( LOGGING_ON  )
                {
                
                AcquisitionRoutine();
                mLED_LP_Off()
                }
            
            }//END RTCC Alarm
    /**************************************************************************/


    /** USB POWER restored  ***************************************************/
          if(USB_POWER_INT_EN == 1)
          {
          if(USB_POWER_FLAG == 1)
          {
              USB_POWER_FLAG = 0;
              if(( LOGGING_ON ==1) )
                {
                                    //Do stuff to stop the logging
                  LOGGING_STOPPED = 1;
                  LOGGING_ON =0;
            
                  PIE3bits.RTCCIE = 0;
                  mRtccAlrmDisable();
         
                    mLED_LP_On()

                   FastClock();
                    StopLogging();
                    //mLED_1_Off()
                    Reset();

                }
              else
              {
                  //if((!PERIOD_OK) || (!TIME_OK))
                    Reset();
              }


          }
          }
/** END USB POWER restored INTERRUPT  *
    /**************************************************************************/





    /** SW1 BUTTON PRESSED  ***************************************************/
          if(SW_PRESSED_INT_EN == 1)
          {
          if(SW_PRESSED_FLAG == 1)
            {
            //SW1 Pressed
              SW_PRESSED_FLAG = 0;
              if(!USB_POWER)
              {
                  SlowClock();
                  OneWay = 1;
                  while(OneWay == 1) //This while is used to be able to exit from the if() statements. It could be a while(1) but Like that, let's say it's safer.
                  {
                      OneWay = 0;
                      if((LOGGING_STOPPED)&&(LOGGING_ON))
                      {
                          //Restart the logging
                          LOGGING_STOPPED = 0;//Just in case
                          LOGGING_ON =1;
                          SlowClock();
                          //Flash the LED three times.
                          DelayLED();
                          mLED_LP_On()
                          DelayLED();
                          mLED_LP_Off()
                          DelayLED();
                          mLED_LP_On()
                          DelayLED();
                          mLED_LP_Off()
                          DelayLED();
                          mLED_LP_On()
                          DelayLED();
                          mLED_LP_Off()
                          PIE3bits.RTCCIE = 1;
                          AcquisitionRoutine();
                          mLED_LP_Off()
                          break;
                      }

                      if ((LOGGING_ON)&&(!LOGGING_STOPPED))
                      {//STOP LOGGING
                          //We were logging, we want to stop it.
                          //Do stuff to stop the logging
                          LOGGING_STOPPED = 1;
                          LOGGING_ON =1;
                          //Make the LED flash two times
                          DelayLED();
                          DelayLED();
                          mLED_LP_On()
                          DelayLED();
                          mLED_LP_Off()
                          DelayLED();
                          mLED_LP_On()
                          DelayLED();
                          mLED_LP_Off()
                          PIE3bits.RTCCIE = 0;
                          mRtccAlrmDisable();
                          break;

                      }//End Stop Logging

                      //START LOGGING//The user pressed the button to start the logging
                      if((PERIOD_OK) && (TIME_OK)&&(!LOGGING_ON)&&(!LOGGING_STOPPED))
                      {
                          LOGGING_ON =1;
                          LOGGING_STOPPED = 0;//Just in case
                          if(USB_POWER)
                              USBDeviceDetach();
                          SlowClock();
                          //Flash the LED three times.
                          DelayLED();
                          mLED_LP_On()
                          DelayLED();
                          mLED_LP_Off()
                          DelayLED();
                          mLED_LP_On()
                          DelayLED();
                          mLED_LP_Off()
                          DelayLED();
                          mLED_LP_On()
                          DelayLED();
                          mLED_LP_Off()
                          StartLogging();
                          AcquisitionRoutine();
                          mLED_LP_Off()
                          break;

                       }//End Start Logging
                  }
                  INTCON3bits.INT1IP = 1;
                  USB_POWER_FLAG = 0;
                  USB_POWER_INT_EDGE = 1;
                  USB_POWER_INT_EN = 1;

                  SW_PRESSED_FLAG = 0;
                  SW_PRESSED_INT_EN = 1;
                  SW_PRESSED_INT_EDGE = 1;
                  INTCONbits.GIE = 1;
                  WDTCONbits.REGSLP = 1;
                  OSCCONbits.IDLEN = 0;
                  DSCONHbits.DSEN = 0;
                  Sleep();
                  }//No USB Power

                }//
            }/** END SW1 BUTTON PRESSED  *
/**************************************************************************/
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
      //  #if defined(USB_INTERRUPT)

           // if(PIE3bits.RTCCIE == 0)

      //  #endif
	
	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
	
	}	//This return will be a "retfie", since this is in a #pragma interruptlow section 

#elif defined(__C30__)
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
        /*
         *	ISR JUMP TABLE
         *
         *	It is necessary to define jump table as a function because C30 will
         *	not store 24-bit wide values in program memory as variables.
         *
         *	This function should be stored at an address where the goto instructions 
         *	line up with the remapped vectors from the bootloader's linker script.
         *  
         *  For more information about how to remap the interrupt vectors,
         *  please refer to AN1157.  An example is provided below for the T2
         *  interrupt with a bootloader ending at address 0x1400
         */
//        void __attribute__ ((address(0x1404))) ISRTable(){
//        
//        	asm("reset"); //reset instruction to prevent runaway code
//        	asm("goto %0"::"i"(&_T2Interrupt));  //T2Interrupt's address
//        }
    #endif
#endif




/** DECLARATIONS ***************************************************/
#pragma code

/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
#if defined(__18CXX)
void main(void)
#else
int main(void)
#endif
{
unsigned int i;
BOOL initResults;
BYTE printData[6];

char TempASCII[8];
WORD_VAL w;
unsigned short fl;
char volname[11]="LOGGER";
DWORD x;
extern long periode;

MiniInit();
//Any initialization should be done here
fl=0;
STATUS_FLAG.Val = 0x00;
LedCounter = 0;
while(1)
{
  if(!USB_POWER)
    {
    //No USB Power, we enter sleep mode
    mLED_1_Off();
    mLED_LP_Off()

    SlowClock();
     EEPROM_Sleep();
    //Switch off the USB and all peripherials too.
    UCONbits.SUSPND = 0;
    UCONbits.USBEN = 0;
    UCFGbits.UPUEN = 0;
    UCFGbits.UTRDIS = 0;

    ADCON0bits.ADON = 0;
    CVRCONbits.CVREN = 0;
    CM1CONbits.CON = 0;
    T0CONbits.TMR0ON = 0;
    T3CONbits.TMR3ON = 0;
    INTCONbits.GIE = 0;
    //Switch interrupt
    if((PERIOD_OK) && (TIME_OK)&&(LOGGING_ON == 0))
        {SW_PRESSED_FLAG = 0;
        SW_PRESSED_INT_EDGE = 1;
        SW_PRESSED_INT_EN = 1;} //Clear and enable switch interrupt
    //Usb Power interrupt
    INTCON3bits.INT1IP = 1;
    USB_POWER_INT_EDGE = 1;
    USB_POWER_FLAG = 0;
    USB_POWER_INT_EN = 1;
    //Interrupts ON
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

    //Put EEPROM to sleep mode
   

    //Put the Temperature sensor to sleep mode
    TEMP_SCK_TRIS = INPUT_PIN;
    TEMP_SDA_TRIS = INPUT_PIN;
    OpenI2C(MASTER, SLEW_OFF);// Initialize I2C module
    StartI2C();
    putcI2C(I2C_TEMP_ADDRESS_W);
    putcI2C(I2C_TEMP_CONF_REG);
    putcI2C(I2C_TEMP_TEMP_CON2_SDW); //Shut the sensor down
    putcI2C(I2C_TEMP_TEMP_CON1);
    StopI2C();
    CloseI2C();
    TEMP_SCK_TRIS = INPUT_PIN;
    TEMP_SDA_TRIS = INPUT_PIN;
    ADCON0bits.ADON = 0;
    ANCON1bits.VBGEN = 0;

    T0CONbits.TMR0ON = 0;
    T1CONbits.TMR1ON = 0;
    T2CONbits.TMR2ON = 0;
    T3CONbits.TMR3ON = 0;
    T4CONbits.TMR4ON = 0; //At some point in the dev, I had problems with the PIC current in sleep mode, so I tried the shut off
                            //all the peripherals.


        while(!USB_POWER)
        {
        WDTCONbits.REGSLP = 1;
        OSCCONbits.IDLEN = 0;
        DSCONHbits.DSEN = 0;

        Sleep();
        }
   
    }


    if(USB_POWER)
     {
        //Here, we're powered by USB
        mLED_1_On()
        InitializeSystem();
        mLED_1_Off()

#if defined USE_ATMEL_FLASH //The SST flash memory doesn't have a sleep mode command
        ENABLE_MSSP();
         SD_CS = 0;
         Nop();
         Nop();
         Nop();
         WriteSPIM( DEEP_PWR_DOWN_RESUME  );
         SD_CS = 1;

#endif

         while(sw1)//Reset the Filesystem if the logger plugged in USB while button pressed
         {
             EraseFileSystem();
             while(sw1);

         }

        USBDeviceInit();
        USBDeviceAttach();
        SW_PRESSED_FLAG = 0;
        SW_PRESSED_INT_EDGE = 1;
        SW_PRESSED_INT_EN = 1;
        while(USB_POWER)
        {
            //Main loop on USB
            ProcessIO();
            if(!USB_POWER)
                break;
            USBDeviceTasks();
            if(!USB_POWER)
                break;
            if((LedFreq != 0)&&(PERIOD_OK) && (TIME_OK))
              {
              if(LedCounter > LedFreq)
                {
                 if(!mLED_1)
                 {mLED_1_On()}//Flash the LED according to the period entered in config.txt
                 else
                 {mLED_1_Off()}
                 LedCounter = 0;
                }
                 LedCounter++;
              }
        

        }

     }


}//End While(1)

}//end main

/********************************************************************
 * Function:        void MiniInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This is a minimal init function with low power
 *                  and slow clock, in the case where the PIC is started on
 *                  battery power
 *
 *
 * Note:            None
 *******************************************************************/
void MiniInit(void)
{
    //Test if we go to the bootloader firmware of if we execute the main program
    //To enter the bootloader, we use the PGC/PGD pins:
    //Short circuit these pins while holding SW1

    TRISBbits.TRISB6 = INPUT_PIN; //PGC
    LATBbits.LATB7 = 0;
    TRISBbits.TRISB7 = OUTPUT_PIN; //PGD
/*    TRISBbits.TRISB1=1;
    if(PORTBbits.RB1 == 0)
    {
        if(PORTBbits.RB6 == 0)
        {
            LATBbits.LATB7 = 1;
            Nop();
            if(PORTBbits.RB6 == 1)
            {
                LATBbits.LATB7 = 0;
                Nop();
                if(PORTBbits.RB6 == 0)
                {
                    TRISBbits.TRISB7 = INPUT_PIN; //PGD
                    LATBbits.LATB6 = 1;
                    TRISBbits.TRISB6 = OUTPUT_PIN; //PGC
                    Nop();
                    if(PORTBbits.RB7 == 1)
                        _asm goto 0x001C _endasm;//Enter the bootloader

                }
            }
        }
    }*/

    //Switch the low freq OSC mode (T1OSC).
    SlowClock();
    INTCONbits.GIE = 0;
    T3CONbits.TMR3ON = 0;

    //All I/O as input
    TRISA = 0xFF;
    TRISB = 0xFF;
    TRISC = 0xFF;
    //No Pull-ups on portB
    INTCON2bits.RBPU = 1;

    mLED_LP_Off()
    mLED_1_Off()


   // TRISBbits.TRISB3 = INPUT_PIN;   //NC pin
    TRISBbits.TRISB6 = INPUT_PIN;   //PGC
    TRISBbits.TRISB7 = INPUT_PIN;   //PGD
   
    ANCON0 = 0b11111111; //DIGITAL input
    ANCON1 = 0b00011111;
    USB_POWER_TRIS = INPUT_PIN;

    //INITIALIZING THE REPROGRAMMABLE PINS FOR SPI EEPROM AND I2C TEMP SENSOR

    EECON2 = 0x55;      //Unlock the RPP
    EECON2 = 0xAA;
    PPSCONbits.IOLOCK = 0;

    //EEPROM SDI on RC7-RP18 (18: 0x12)  //SDO2:9
    RPOR18 = 0x09;
    //EEPROM SCK  on RC6-RP17 (clock output)
    RPOR17=0x0A;
    // EEPROM SCK on RC6-RP17 (clock input)
    RPINR22=0x11;
    //EEPROM DSO on RB2-RP5 SDI:21
    RPINR21=0x05;
    //RP1 on INT1 (USB POWER)
    RPINR1 = 0x01;


    EECON2 = 0x55;
    EECON2 = 0xAA;
    PPSCONbits.IOLOCK = 1;  //Re-Lock the RPP


    USB_POWER_INT_EN = 0;
    //Switch as input
    mInitSwitch1();
       


    //Put the temp Sensor in sleep mode
    TEMP_SCK_TRIS = INPUT_PIN;
    TEMP_SDA_TRIS = INPUT_PIN;
    OpenI2C(MASTER, SLEW_OFF);// Initialize I2C module
    StartI2C();
    putcI2C(I2C_TEMP_ADDRESS_W);
    putcI2C(I2C_TEMP_CONF_REG);
    putcI2C(I2C_TEMP_TEMP_CON2_SDW); //Shut the sensor down
    putcI2C(I2C_TEMP_TEMP_CON1);
    StopI2C();
    CloseI2C();

 

}

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralized initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
    BYTE InterruptEnableSave,fl,resultats,tmp;
    unsigned char Temp_Conf1,Temp_Conf2;
    WORD pll_startup_counter = 0x600;
    extern const rom char Config0[0xFF];
    extern const rom char Instr0[0xFF];
    extern const  rom char LOG_NOT_STARTED;
    extern int BATT_POWER_SENSE;
    extern const rom char Config1[0xFF];
    static char volname[11]="LOGGER";
    BYTE swFlag = 0;

    
    mInitAllLEDs();
    //Switch the T1OSC and configure PIC for fast mode OSC (PLL)
    FastClock();  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
   
    T3CONbits.TMR3ON = 0;

    LogBufferPtr = FIRST_DATA_SECTOR +2; //Pointer for the current sector of the drive.
    MDD_SDSPI_InitIO();
    WriteEnable = TRUE;

    //Variable init
    LedFreq = 0;
    StatePeriod = 0;
    TimeState = 0;
    FullOS = 0;

    while (sw1)
    {
    EraseFileSystem();
    break;
    }

    LOW_BATT = 0;

    TimeDate.f.hour = 0;
    TimeDate.f.mday = 0;
    TimeDate.f.min = 0;
    TimeDate.f.mon = 0;
    TimeDate.f.sec = 0;
    TimeDate.f.wday = 0;
    TimeDate.f.year = 0;
    
    LoggerID=0;

    //Get the logger ID if there's one
    //The extra configuration bytes are in the last 4KB sector of the EEPROM
    //This space can be used to store data and state/config variables.
    MDD_SDSPI_SectorRead(0x6AF, gDataBuffer);
    if((gDataBuffer[0]!=0xFF)&&(gDataBuffer[1]!=0xFF))
    {
        LoggerID = gDataBuffer[0];
        LoggerID = LoggerID << 8;
        LoggerID |= gDataBuffer[1];
        ultoa(LoggerID,idString);

    }
}



/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks. Microchip original source legacy
 *
 * Note:            None
 *******************************************************************/
void ProcessIO(void)
{
   // User Application USB tasks
    //if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

    MSDTasks();
    if(!USB_POWER)
        return;

    if(TIME_READ_FROM_FILE) //The config file has been saved and we got the OS time&date
    {
        TimeDate.f.wday = 0;
        if (RtccWriteTimeDate(&TimeDate,0) )
        {
         TIME_OK = 1;
         DtASCII(TimeDateASCII);
         RtccInitClock();
         RtccWrOn();
         mRtccOn();
         RtccWrOn();
        }
        else
            TimeState = TIME_UPDATE_FAILED;
    }

    if(TIME_OK)
        RtccReadTimeDate(&TimeDate);

    if(!USB_POWER)
        return;


    if(PERIOD_OK)
    {
        PERIOD_OK = 1;
        if(periode < 60)
            LedFreq = LED_SECOND;
        if((periode >= 60) && (periode<3600))
            LedFreq = LED_MINUTE;
        if(periode >= 3600)
            LedFreq = LED_HOUR;

    }
    else
        LedFreq = 0;
         
     
}//end ProcessIO


// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:
	
	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	

}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *					In this example the interrupt is only used when the device
 *					goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 Function:        void USBCB_SOF_Handler(void)

 Description:     The USB host sends out a SOF packet to full-speed
                  devices every 1 ms. This interrupt may be useful
                  for isochronous pipes. End designers should
                  implement callback routine as necessary.

 PreCondition:    None
 
 Parameters:      None
 
 Return Values:   None
 
 Remarks:         None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckMSDRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    #if (MSD_DATA_IN_EP == MSD_DATA_OUT_EP)
        USBEnableEndpoint(MSD_DATA_IN_EP,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    #else
        USBEnableEndpoint(MSD_DATA_IN_EP,USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
        USBEnableEndpoint(MSD_DATA_OUT_EP,USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    #endif

    USBMSDInit();
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function will only be able to wake up the host if
 *                  all of the below are true:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior, 
 *                  as a USB device that has not been armed to perform remote 
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *                  
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are 
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex: 
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup. 
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in 
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager 
    //properties page for the USB device, power management tab, the 
    //"Allow this device to bring the computer out of standby." checkbox 
    //should be checked).
    if(USBGetRemoteWakeupStatus() == TRUE) 
    {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();
            
            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;  //So we don't execute this code again, 
                                        //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at 
            //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yeilds
            //5ms+ total delay since start of idle).
            delay_count = 3600U;        
            do
            {
                delay_count--;
            }while(delay_count);
            
            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            }while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            //Add application specific callback task or callback function here if desired.
            //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
            //FEATURE (endpoint halt) request on an application endpoint which was 
            //previously armed (UOWN was = 1).  Here would be a good place to:
            //1.  Determine which endpoint the transaction that just got terminated was 
            //      on, by checking the handle value in the *pdata.
            //2.  Re-arm the endpoint if desired (typically would be the case for OUT 
            //      endpoints).

            //Check if the host recently did a clear endpoint halt on the MSD OUT endpoint.
            //In this case, we want to re-arm the MSD OUT endpoint, so we are prepared
            //to receive the next CBW that the host might want to send.
            //Note: If however the STALL was due to a CBW not valid condition, 
            //then we are required to have a persistent STALL, where it cannot 
            //be cleared (until MSD reset recovery takes place).  See MSD BOT 
            //specs v1.0, section 6.6.1.
            if(MSDWasLastCBWValid() == FALSE)
            {
                //Need to re-stall the endpoints, for persistent STALL behavior.
    			USBStallEndpoint(MSD_DATA_IN_EP, IN_TO_HOST);
      			USBStallEndpoint(MSD_DATA_OUT_EP, OUT_FROM_HOST);                 
            }
            else
            {   
                //Check if the host cleared halt on the bulk out endpoint.  In this
                //case, we should re-arm the endpoint, so we can receive the next CBW.
                if((USB_HANDLE)pdata == USBGetNextHandle(MSD_DATA_OUT_EP, OUT_FROM_HOST))
                {
                    USBMSDOutHandle = USBRxOnePacket(MSD_DATA_OUT_EP, (BYTE*)&msd_cbw, MSD_OUT_EP_SIZE);
                }    
            }    
            break;
        default:
            break;
    }      
    return TRUE; 
}

/*******************************************************************
 * Function: Contains of the MBR to be copied in the EEPROM
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    Erases gDataBuffer
 *
 * Overview:
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void FillMBR(void)
{
memset(gDataBuffer,0x00,0x200);

//gDataBuffer[0x1BE] =  0x00;//Status - 0x80 (bootable), 0x00 (not bootable), other (error)
gDataBuffer[0x1BF] =  0x01;//Cylinder-head-sector address of first sector in partition
//gDataBuffer[0x1C0] =  0x00;
//gDataBuffer[0x1C1] =  0x00;//0x02;
gDataBuffer[0x1C2] =  0x01;//Partition type - 0x01 = FAT12 up to 2MB
//gDataBuffer[0x1C3] =  0x00;//Cylinder-head-sector address of last sector in partition
//gDataBuffer[0x1C4] =  0x00;
gDataBuffer[0x1C5] =  0x42;
gDataBuffer[0x1C6] =  0x01;//Logical Block Address (LBA) of first sector in partition
//gDataBuffer[0x1C7] =  0x00;
//gDataBuffer[0x1C8] =  0x00;
//gDataBuffer[0x1C9] =  0x00;
gDataBuffer[0x1CA] =  0x7F;//(BYTE)PARTITION_SIZE;//0x3F ;//Length of partition in sectors (MBR sits at LBA = 0, and is not in the partition.)
gDataBuffer[0x1CB] =  0x06;//(BYTE)(PARTITION_SIZE >>8);//0x00;
//gDataBuffer[0x1CC] =  0x00;//(BYTE)(PARTITION_SIZE >>16);//0x00;
//gDataBuffer[0x1CD] =  0x00;//0x00;


gDataBuffer[0x1FE] =  0x55;
gDataBuffer[0x1FF] =  0xAA;
}


/*******************************************************************
 * Function: Contains of the Boot Sector to be copied in the EEPROM
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:      Erases gDataBuffer
 *
 * Overview:
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void FillBootSector(void)
{
    WORD x;
    int p;
  memset(gDataBuffer,0x00,0x200);
    gDataBuffer[0x00] = 0xEB; //Jump instruction
    gDataBuffer[0x01] = 0X3C;
    gDataBuffer[0x02] = 0X90;
    gDataBuffer[0x03] = 'M';//OEM Name "MSDOS5.0"
    gDataBuffer[0x04] = 'S';
    gDataBuffer[0x05] = 'D';
    gDataBuffer[0x06] = 'O' ;
    gDataBuffer[0x07] = 'S';
    gDataBuffer[0x08] = '5';
    gDataBuffer[0x09] = '.';
    gDataBuffer[0x0A] = '0';
    gDataBuffer[0x0B] = 0x00;   //Bytes per sector (MEDIA_SECTOR_SIZE)
    gDataBuffer[0x0C] = 0x02;
    gDataBuffer[0x0D] = 0x01;   //Sectors per cluster
    gDataBuffer[0x0E] = RESERVED_SECTORS  ;  //Reserved sector count (usually 1 for FAT12 or FAT16)
    gDataBuffer[0x0F] = 0x00 ;
    gDataBuffer[0x10] = NUMBER_OF_FATS;   //number of FATs
    gDataBuffer[0x11] = 0x00;   //Max number of root directory entries - 16 files allowed ;
    gDataBuffer[0x12] = 0x02;//02
    gDataBuffer[0x13] = 0x80 ;  //total sectors (0x0000 means: use the 4 byte field at offset 0x20 instead)
    gDataBuffer[0x14] = 0x06;
    gDataBuffer[0x15] = 0xF8;   //Media Descriptor
    gDataBuffer[0x16] = SECTORS_PER_FAT;   //Sectors per FAT
    gDataBuffer[0x17] = 0x00;
    gDataBuffer[0x18] = (BYTE)PARTITION_SIZE;//0x3F; //(BYTE)PARTITION_SIZE;////Sectors per track
    gDataBuffer[0x19] = (BYTE)(PARTITION_SIZE >>8);//0x00;
    gDataBuffer[0x1A] = 0xFF;   //Number of heads
    gDataBuffer[0x1B] = 0x00;
    gDataBuffer[0x1C] = HIDDEN_SECTORS;   //Hidden sectors
/*    gDataBuffer[0x1D] = 0x00;
    gDataBuffer[0x1E] = 0x00;
    gDataBuffer[0x1F] = 0x00;
    gDataBuffer[0x20] = 0x00;//(BYTE)PARTITION_SIZE;//0x3F  ; //Total sectors (when WORD value at offset 0x13 is 0x0000)
    gDataBuffer[0x21] = 0x00;//(BYTE)(PARTITION_SIZE >>8);//0x00;//(BYTE)(MDD_INTERNAL_FLASH_PARTITION_SIZE >> 8)  ;
    gDataBuffer[0x22] = 0x00;//(BYTE)(PARTITION_SIZE >>16);//0x00;//(BYTE)(MDD_INTERNAL_FLASH_PARTITION_SIZE >> 16)  ;
    gDataBuffer[0x23] = 0x00;//(BYTE)(MDD_INTERNAL_FLASH_PARTITION_SIZE >> 24)  ;
    gDataBuffer[0x24] = 0x00;			//Physical drive number
    gDataBuffer[0x25] = 0x00;			//Reserved("current head")*/
    
    gDataBuffer[0x26] = 0x29;			//Signature
    gDataBuffer[0x27] = 0x23;
    gDataBuffer[0x28] = 0x01;
    gDataBuffer[0x29] = 0x00;
    gDataBuffer[0x2A] = 0x00;
    gDataBuffer[0x2B] = 'L' ;   //Volume Label (11 bytes) - "LOGGER "
    gDataBuffer[0x2C] = 'O' ;
    gDataBuffer[0x2D] = 'G' ;
    gDataBuffer[0x2E] = 'G' ;
    gDataBuffer[0x2F] = 'E' ;
    gDataBuffer[0x30] = 'R' ;
    gDataBuffer[0x31] = ' ' ;
    gDataBuffer[0x32] = ' ' ;
    gDataBuffer[0x33] = ' ' ;
    gDataBuffer[0x34] = ' ' ;
    gDataBuffer[0x35] = ' ' ;
    //If the logger has an ID, replace the blanks by the number
    if(LoggerID)
    {
        ultoa(LoggerID,idString);

        for(p=0;p<5;p++)
        {
            if(idString[p]==0)
                break;
            gDataBuffer[p+30] = idString[p];

        }
    }






    gDataBuffer[0x36] = 'F' ;   //FAT system "FAT12   "
    gDataBuffer[0x37] = 'A' ;
    gDataBuffer[0x38] = 'T' ;
    gDataBuffer[0x39] = '1' ;
    gDataBuffer[0x3A] = '2' ;
    gDataBuffer[0x3B] = ' ' ;
    gDataBuffer[0x3C] = ' ' ;
    gDataBuffer[0x3D] = ' ' ;
    gDataBuffer[0x3E] = 0x00; //Operating system boot code

    for(x=0x3F;x<=0x1FD;x++)
      gDataBuffer[x] = 0x00;

    gDataBuffer[0x1FE] = 0x55 ;
    gDataBuffer[0x1FF] = 0xAA ;

}
/*******************************************************************
 * Function: Contains of the FAT to be copied in the EEPROM
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:   Erases gDataBuffer
 *
 * Overview:
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void FillFAT(void)
{
    WORD x;
    for(x=0x00;x<0x200;x++)
       gDataBuffer[x] = 0x00;
    
    gDataBuffer[0x00] = 0xF8; //Copy of the media descriptor 0xFF8
    gDataBuffer[0x01] = 0x0F;
    gDataBuffer[0x02] = 0x00;//Partition state

    gDataBuffer[0x03] = 0xFF;//7F
    gDataBuffer[0x04] = 0xFF;//First sector(02C) (config.txt)
    gDataBuffer[0x05] = 0xFF;

    gDataBuffer[0x06] = 0xFF;//second sector(02D) (help.txt)(03 in fat)
    gDataBuffer[0x07] = 0x0F;
    gDataBuffer[0x08] = 0x00;//second sector(02E) (data.csv)(04 in fat)

}

/*******************************************************************
 * Function: Contains of the FAT to be copied in the EEPROM
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    Erases msd_buffer
 *
 * Overview:
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void FillFAT_msd(void)
{
    WORD x;
    for(x=0x00;x<0x200;x++)
       msd_buffer[x] = 0x00;

    msd_buffer[0x00] = 0xF8; //Copy of the media descriptor 0xFF8
    msd_buffer[0x01] = 0x0F;
    msd_buffer[0x02] = 0x00;//Partition state

    msd_buffer[0x03] = 0xFF;//7F
    msd_buffer[0x04] = 0xFF;//First sector(02C) (config.txt)
    msd_buffer[0x05] = 0xFF;

    msd_buffer[0x06] = 0xFF;//second sector(02D) (help.txt)(03 in fat)
    msd_buffer[0x07] = 0x0F;
    msd_buffer[0x08] = 0x00;//second sector(02E) (data.csv)(04 in fat)


}

/*******************************************************************
 * Function: Contains of the File Directory to be copied in the EEPROM
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    Erases gDataBuffer
 *
 * Overview:
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void FillDirectory(void)
{
    int p;
    
    memset(gDataBuffer,0x00,0x200);
    gDataBuffer[0x06] = ' ' ;
    gDataBuffer[0x07] = ' ' ;
    gDataBuffer[0x08] = ' ' ;
    gDataBuffer[0x09] = ' ' ;
    gDataBuffer[0x0A] = ' ' ;
    if(LoggerID)
    {
        ultoa(LoggerID,idString);

        for(p=0;p<5;p++)
        {
            if(idString[p]==0)
                break;
            gDataBuffer[p+6] = idString[p];

        }
    }    
    
    
    gDataBuffer[0x00] = 'L' ;  //Drive Name (11 characters, padded with spaces)
    gDataBuffer[0x01] = 'O' ;
    gDataBuffer[0x02] = 'G' ;
    gDataBuffer[0x03] = 'G' ;
    gDataBuffer[0x04] = 'E' ;
    gDataBuffer[0x05] = 'R' ;
    

    gDataBuffer[0x0B] = 0x08; //specify this entry as a volume label
    gDataBuffer[0x0C] = 0x00; //Reserved
    gDataBuffer[0x0D] = 0x06;  //Create time
    gDataBuffer[0x0E] = 0x28;
    gDataBuffer[0x0F] = 0x78;
    gDataBuffer[0x10] = 0xDE;
    gDataBuffer[0x11] = 0x38;
    gDataBuffer[0x12] = 0xDE;   //Last Access
    gDataBuffer[0x13] = 0x38;
    gDataBuffer[0x14] = 0x00;  //EA-index
    gDataBuffer[0x15] = 0x00;
    gDataBuffer[0x16] = 0x04;  //Last modified time
    gDataBuffer[0x17] = 0x77;
    gDataBuffer[0x18] = 0xDE;
    gDataBuffer[0x19] = 0x38;
/**    gDataBuffer[0x1A] = 0x00;     //First FAT cluster
    gDataBuffer[0x1B] = 0x00;
    gDataBuffer[0x1C] = 0x00;  //File Size (number of bytes)
    gDataBuffer[0x1D] = 0x00;
    gDataBuffer[0x1E] = 0x00;
    gDataBuffer[0x1F] = 0x00;*/

    gDataBuffer[0x20] = 'C' ;  //Drive Name (8 characters, padded with spaces)
    gDataBuffer[0x21] = 'O' ;
    gDataBuffer[0x22] = 'N' ;
    gDataBuffer[0x23] = 'F' ;
    gDataBuffer[0x24] = 'I' ;
    gDataBuffer[0x25] = 'G' ;
    gDataBuffer[0x26] = ' ' ;
    gDataBuffer[0x27] = ' ' ;
    gDataBuffer[0x28] = 'T' ;
    gDataBuffer[0x29] = 'X' ;
    gDataBuffer[0x2A] = 'T' ;
    gDataBuffer[0x2B]= 0x20; //specify this entry as a volume label
    gDataBuffer[0x2C]= 0x00 ;//Reserved
    gDataBuffer[0x2D]= 0x06 ; //Create time
    gDataBuffer[0x2E]= 0x28 ;
    gDataBuffer[0x2F]= 0x78 ;
    gDataBuffer[0x30]= 0xDE ;
    gDataBuffer[0x31]= 0x38 ;
    gDataBuffer[0x32]= 0xDE ;  //Last Access
    gDataBuffer[0x33]= 0x38 ;
    gDataBuffer[0x34]= 0x00 ; //EA-index
    gDataBuffer[0x35]= 0x00 ;
    gDataBuffer[0x36]= 0x04 ; //Last modified time
    gDataBuffer[0x37]= 0x77 ;
    gDataBuffer[0x38]= 0xDE ;
    gDataBuffer[0x39]= 0x38 ;
    gDataBuffer[0x3A]= 0x02 ;    //First FAT cluster//2
    gDataBuffer[0x3B]= 0x00 ;
    gDataBuffer[0x3C]= 0x00 ; //File Size (number of bytes)
    gDataBuffer[0x3D]= 0x02 ;
    gDataBuffer[0x3E]= 0x00 ;
    gDataBuffer[0x3F]= 0x00 ;



    gDataBuffer[0x40] = 'H' ;  //Drive Name (8 characters, padded with spaces)
    gDataBuffer[0x41] = 'E' ;
    gDataBuffer[0x42] = 'L' ;
    gDataBuffer[0x43] = 'P' ;
    gDataBuffer[0x44] = ' ' ;
    gDataBuffer[0x45] = ' ' ;
    gDataBuffer[0x46] = ' ' ;
    gDataBuffer[0x47] = ' ' ;
    gDataBuffer[0x48] = 'T' ;
    gDataBuffer[0x49] = 'X' ;
    gDataBuffer[0x4A] = 'T' ;
    gDataBuffer[0x4B]= 0x21; //specify this entry as a volume label and read-only
    gDataBuffer[0x4C]= 0x00 ;//Reserved
    gDataBuffer[0x4D]= 0x06 ; //Create time
    gDataBuffer[0x4E]= 0x28 ;
    gDataBuffer[0x4F]= 0x78 ;
    gDataBuffer[0x50]= 0xDE ;
    gDataBuffer[0x51]= 0x38 ;
    gDataBuffer[0x52]= 0xDE ;  //Last Access
    gDataBuffer[0x53]= 0x38 ;
    gDataBuffer[0x54]= 0x00 ; //EA-index
    gDataBuffer[0x55]= 0x00 ;
    gDataBuffer[0x56]= 0x04 ; //Last modified time
    gDataBuffer[0x57]= 0x77 ;
    gDataBuffer[0x58]= 0xDE ;
    gDataBuffer[0x59]= 0x38 ;
    gDataBuffer[0x5A]= 0x03 ;    //First FAT cluster//3
    gDataBuffer[0x5B]= 0x00 ;
    gDataBuffer[0x5C]= 0x00 ; //File Size (number of bytes)
    gDataBuffer[0x5D]= 0x02 ;
    gDataBuffer[0x5E]= 0x00 ;
    gDataBuffer[0x5F]= 0x00 ;


    gDataBuffer[0x60] = 'D' ;  //Drive Name (8 characters, padded with spaces)
    gDataBuffer[0x61] = 'A' ;
    gDataBuffer[0x62] = 'T' ;
    gDataBuffer[0x63] = 'A' ;
    gDataBuffer[0x64] = ' ' ;
    gDataBuffer[0x65] = ' ' ;
    gDataBuffer[0x66] = ' ' ;
    gDataBuffer[0x67] = ' ' ;
    gDataBuffer[0x68] = 'C' ;
    gDataBuffer[0x69] = 'S' ;
    gDataBuffer[0x6A] = 'V' ;
    gDataBuffer[0x6B]= 0x21; //specify this entry as a volume label and read-only
    gDataBuffer[0x6C]= 0x00 ;//Reserved
    gDataBuffer[0x6D]= 0x06 ; //Create time
    gDataBuffer[0x6E]= 0x28 ;
    gDataBuffer[0x6F]= 0x78 ;
    gDataBuffer[0x70]= 0xDE ;
    gDataBuffer[0x71]= 0x38 ;
    gDataBuffer[0x72]= 0xDE ;  //Last Access
    gDataBuffer[0x73]= 0x38 ;
    gDataBuffer[0x74]= 0x00 ; //EA-index
    gDataBuffer[0x75]= 0x00 ;
    gDataBuffer[0x76]= 0x05 ; //Last modified time
    gDataBuffer[0x77]= 0x77 ;
    gDataBuffer[0x78]= 0xDE ;
    gDataBuffer[0x79]= 0x38 ;
    gDataBuffer[0x7A]= 0x04 ;    //First FAT cluster//
    gDataBuffer[0x7B]= 0x00 ;
    gDataBuffer[0x7C]= 0x00 ; //File Size (number of bytes)
    gDataBuffer[0x7D]= 0x02 ;
    gDataBuffer[0x7E]= 0x00 ;
    gDataBuffer[0x7F]= 0x00 ;

}
/*******************************************************************
 * Function: Contains of the File Directory to be copied in the EEPROM
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    Erases msd_buffer
 *
 * Overview:
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void FillDirectoryMSD(WORD LogFileSize)
{
    int p;
    
    memset(msd_buffer,0x00,0x200);
    msd_buffer[0x06] = ' ' ;
    msd_buffer[0x07] = ' ' ;
    msd_buffer[0x08] = ' ' ;
    msd_buffer[0x09] = ' ' ;
    msd_buffer[0x0A] = ' ' ;
    if(LoggerID)
    {
        ultoa(LoggerID,idString);
        for(p=0;p<5;p++)
        {
            if(idString[p]==0)
                break;
            msd_buffer[p+6] = idString[p];

        }
    }



    msd_buffer[0x00] = 'L' ;  //Drive Name (11 characters, padded with spaces)
    msd_buffer[0x01] = 'O' ;
    msd_buffer[0x02] = 'G' ;
    msd_buffer[0x03] = 'G' ;
    msd_buffer[0x04] = 'E' ;
    msd_buffer[0x05] = 'R' ;
    


    msd_buffer[0x0B] = 0x08; //specify this entry as a volume label
    msd_buffer[0x0C] = 0x00; //Reserved
    msd_buffer[0x0D] = 0x06;  //Create time
    msd_buffer[0x0E] = 0x28;
    msd_buffer[0x0F] = 0x78;
    msd_buffer[0x10] = 0xDE;
    msd_buffer[0x11] = 0x38;
    msd_buffer[0x12] = 0xDE;   //Last Access
    msd_buffer[0x13] = 0x38;
    msd_buffer[0x14] = 0x00;  //EA-index
    msd_buffer[0x15] = 0x00;
    //msd_buffer[0x16] = 0x04;  //Last modified time
    //msd_buffer[0x17] = 0x77;
    //msd_buffer[0x18] = 0xDE;
    //msd_buffer[0x19] = 0x38;
/*    msd_buffer[0x1A] = 0x00;     //First FAT cluster
    msd_buffer[0x1B] = 0x00;
    msd_buffer[0x1C] = 0x00;  //File Size (number of bytes)
    msd_buffer[0x1D] = 0x00;
    msd_buffer[0x1E] = 0x00;
    msd_buffer[0x1F] = 0x00;
*/
    msd_buffer[0x20] = 'C' ;  //Drive Name (8 characters, padded with spaces)
    msd_buffer[0x21] = 'O' ;
    msd_buffer[0x22] = 'N' ;
    msd_buffer[0x23] = 'F' ;
    msd_buffer[0x24] = 'I' ;
    msd_buffer[0x25] = 'G' ;
    msd_buffer[0x26] = ' ' ;
    msd_buffer[0x27] = ' ' ;
    msd_buffer[0x28] = 'T' ;
    msd_buffer[0x29] = 'X' ;
    msd_buffer[0x2A] = 'T' ;
    msd_buffer[0x2B]= 0x20; //specify this entry as a volume label
    msd_buffer[0x2C]= 0x00 ;//Reserved
    msd_buffer[0x2D]= 0x06 ; //Create time
    msd_buffer[0x2E]= 0x28 ;
    msd_buffer[0x2F]= 0x78 ;
    msd_buffer[0x30]= 0xDE ;
    msd_buffer[0x31]= 0x38 ;
    //msd_buffer[0x32]= 0xDE ;  //Last Access
    //msd_buffer[0x33]= 0x38 ;
//    msd_buffer[0x34]= 0x00 ; //EA-index
//    msd_buffer[0x35]= 0x00 ;
    //msd_buffer[0x36]= 0x04 ; //Last modified time
    //msd_buffer[0x37]= 0x77 ;
    //msd_buffer[0x38]= 0xDE ;
    //msd_buffer[0x39]= 0x38 ;
    msd_buffer[0x3A]= 0x02 ;    //First FAT cluster//2
    msd_buffer[0x3B]= 0x00 ;
    msd_buffer[0x3C]= 0x00 ; //File Size (number of bytes)
    msd_buffer[0x3D]= 0x02 ;
    msd_buffer[0x3E]= 0x00 ;
    msd_buffer[0x3F]= 0x00 ;


    msd_buffer[0x40] = 'H' ;  //Drive Name (8 characters, padded with spaces)
    msd_buffer[0x41] = 'E' ;
    msd_buffer[0x42] = 'L' ;
    msd_buffer[0x43] = 'P' ;
    msd_buffer[0x44] = ' ' ;
    msd_buffer[0x45] = ' ' ;
    msd_buffer[0x46] = ' ' ;
    msd_buffer[0x47] = ' ' ;
    msd_buffer[0x48] = 'T' ;
    msd_buffer[0x49] = 'X' ;
    msd_buffer[0x4A] = 'T' ;
    msd_buffer[0x4B]= 0x21; //specify this entry as a volume label and read-only
    msd_buffer[0x4C]= 0x00 ;//Reserved
    msd_buffer[0x4D]= 0x06 ; //Create time
    msd_buffer[0x4E]= 0x28 ;
    msd_buffer[0x4F]= 0x78 ;
    msd_buffer[0x50]= 0xDE ;
    msd_buffer[0x51]= 0x38 ;
    //msd_buffer[0x52]= 0xDE ;  //Last Access
    //msd_buffer[0x53]= 0x38 ;
    msd_buffer[0x54]= 0x00 ; //EA-index
    msd_buffer[0x55]= 0x00 ;
    //msd_buffer[0x56]= 0x04 ; //Last modified time
    //msd_buffer[0x57]= 0x77 ;
    //msd_buffer[0x58]= 0xDE ;
    //msd_buffer[0x59]= 0x38 ;
    msd_buffer[0x5A]= 0x03 ;    //First FAT cluster//3
    msd_buffer[0x5B]= 0x00 ;
    msd_buffer[0x5C]= 0x00 ; //File Size (number of bytes)
    msd_buffer[0x5D]= 0x02 ;
    msd_buffer[0x5E]= 0x00 ;
    msd_buffer[0x5F]= 0x00 ;


    msd_buffer[0x60] = 'D' ;  //Drive Name (8 characters, padded with spaces)
    msd_buffer[0x61] = 'A' ;
    msd_buffer[0x62] = 'T' ;
    msd_buffer[0x63] = 'A' ;
    msd_buffer[0x64] = ' ' ;
    msd_buffer[0x65] = ' ' ;
    msd_buffer[0x66] = ' ' ;
    msd_buffer[0x67] = ' ' ;
    msd_buffer[0x68] = 'C' ;
    msd_buffer[0x69] = 'S' ;
    msd_buffer[0x6A] = 'V' ;
    msd_buffer[0x6B]= 0x21; //specify this entry as a volume label and read-only
    msd_buffer[0x6C]= 0x00 ;//Reserved
    msd_buffer[0x6D]= 0x06 ; //Create time
    msd_buffer[0x6E]= 0x28 ;
    msd_buffer[0x6F]= 0x78 ;
    msd_buffer[0x70]= 0xDE ;
    msd_buffer[0x71]= 0x38 ;
    msd_buffer[0x72]= 0xDE ;  //Last Access
    msd_buffer[0x73]= 0x38 ;
    msd_buffer[0x74]= 0x00 ; //EA-index
    msd_buffer[0x75]= 0x00 ;
    msd_buffer[0x76]= 0x04 ; //Last modified time
    msd_buffer[0x77]= 0x77 ;
    msd_buffer[0x78]= 0xDE ;
    msd_buffer[0x79]= 0x38 ;
    msd_buffer[0x7A]= 0x04 ;    //First FAT cluster//
    msd_buffer[0x7B]= 0x00 ;
    msd_buffer[0x7C]= (BYTE)LogFileSize ; //File Size (number of sectors)
    msd_buffer[0x7D]= (BYTE)(LogFileSize>>8 );
    msd_buffer[0x7E]= (BYTE)(LogFileSize >>16 );
    msd_buffer[0x7F]= (BYTE)(LogFileSize >>24 );
    //msd_buffer[0x7C]= 0x00 ; //File Size (number of bytes)
    //msd_buffer[0x7D]= 0x02 ;
    //msd_buffer[0x7E]= 0x00 ;
    //msd_buffer[0x7F]= 0x00 ;

}
/*******************************************************************
 * Function: Converts the Time and Date in the global TimeDate var
 *into the loggable ASCII String
 *
 * PreCondition:    None
 *
 * Input: TimeDate (global)
 *
 *
 *
 * Output:          char * tabASCII
 *
 * Side Effects:    None
 *
 * Overview:
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void DtASCII(unsigned char * tabASCII)
{
 tabASCII[0] = ((TimeDate.f.mday & 0xF0)>>4) + 0x30;
 tabASCII[1] = (TimeDate.f.mday & 0x0F) + 0x30;
 tabASCII[2] = '/';
 tabASCII[3] = ((TimeDate.f.mon & 0xF0)>>4) + 0x30;
 tabASCII[4] = (TimeDate.f.mon & 0x0F) + 0x30;
 tabASCII[5] = '/';
 tabASCII[6] = ((TimeDate.f.year & 0xF0)>>4) + 0x30;
 tabASCII[7] = (TimeDate.f.year & 0x0F) + 0x30;
 tabASCII[8] = 0x09;//' ' ;
 tabASCII[9] = ((TimeDate.f.hour & 0xF0)>>4) + 0x30;
 tabASCII[10]= (TimeDate.f.hour & 0x0F) + 0x30;
 tabASCII[11]= ':';
 tabASCII[12]= ((TimeDate.f.min & 0xF0)>>4) + 0x30;
 tabASCII[13]= (TimeDate.f.min & 0x0F) + 0x30;
 tabASCII[14]= ':';
 tabASCII[15]= ((TimeDate.f.sec & 0xF0)>>4) + 0x30;
 tabASCII[16]= (TimeDate.f.sec & 0x0F) + 0x30;
 tabASCII[17]= 0x09;//';';
 tabASCII[18]= 0x00;
}

/*******************************************************************
 * Function: Starts a single temperature conversion while
 *           temperature sensor in Shut Down mode
 *
 *
 * PreCondition:    None
 *
 * Input: none
 *
 *
 *
 * Output:          none
 *
 * Side Effects:    None
 *
 * Overview:
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void StartOneShotTemp(void)
{
    OpenI2C(MASTER, SLEW_OFF);// Initialize I2C module
    //Enabling the single shot conversion
    StartI2C();
    putcI2C(I2C_TEMP_ADDRESS_W);
    putcI2C(I2C_TEMP_CONF_REG);
    putcI2C(I2C_TEMP_TEMP_CON2_1CONV); //Start a single temperature conversion while in Shut Down mode
    putcI2C(I2C_TEMP_TEMP_CON1);
    StopI2C();

}
/*******************************************************************
 * Function: Polls the status ready bit of the temp sensor to see
 *if the one shot temp acq is finished
 *
 * PreCondition:    One Shot temp acq started StartOneShotTemp(void)
 *
 * Input:
 *
 *
 *
 * Output:          0 if conversion finished, 1 otherwise
 *
 * Side Effects:    None
 *
 * Overview:
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
int OneShotTempWait(void)
{
    int tmp;
    static BYTE Temp_Conf1,Temp_Conf2;

    OpenI2C(MASTER, SLEW_OFF);// Initialize I2C module
    StartI2C();
    putcI2C(I2C_TEMP_ADDRESS_W);
    putcI2C(I2C_TEMP_CONF_REG);
    StopI2C();
    StartI2C();
    putcI2C(I2C_TEMP_ADDRESS_R);
    Temp_Conf1 = ReadI2C();
    AckI2C();
    Temp_Conf2 = ReadI2C();
    AckI2C();
    StopI2C();
    CloseI2C();
    if((Temp_Conf1 & 0x80) == 0x80)
        return 0;
    else
        return 1;
    EEPROM_Sleep();
        return 0;
        
}
/*******************************************************************
 * Function: Reads the measured temperature by the sensor and converts it
 *into the loggable string
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          char* tabASCII
 *
 * Side Effects:    None
 *
 * Overview:
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void OneShotTempRead(unsigned char* tabASCII)
{
       static BYTE Temperature0,Temperature1,j;
   extern char Temp_Precision[34];
   static char TempASCII[10];

   
    mLED_LP_On()
    OpenI2C(MASTER, SLEW_OFF);// Initialize I2C module
    StartI2C();
    putcI2C(I2C_TEMP_ADDRESS_W);
    putcI2C(I2C_TEMP_TEMP_REG);
    StopI2C();
    StartI2C();
    putcI2C(I2C_TEMP_ADDRESS_R);
    Temperature0 = ReadI2C();
    AckI2C();
    Temperature1 = ReadI2C();
    AckI2C();
    StopI2C();

    StartI2C();
    putcI2C(I2C_TEMP_ADDRESS_W);
    putcI2C(I2C_TEMP_CONF_REG);
    putcI2C(I2C_TEMP_TEMP_CON2_SDW); //Shut the sensor down
    putcI2C(I2C_TEMP_TEMP_CON1);
    StopI2C();
    CloseI2C();
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;

    Temperature1 = Temperature1>>4;
    btoa(Temperature0,TempASCII);
    for(j=0;j<8;j++)
    {
    tabASCII[18+j] = TempASCII[j];


    if(TempASCII[j]== 0x00)
            {
            TempASCII[j] = VIRGULE_TEMPERATURE;
            TempASCII[j+1] = Temp_Precision[Temperature1*2];
            TempASCII[j+2] = Temp_Precision[(Temperature1*2)+1];
            TempASCII[j+3] = 0x0D;
            TempASCII[j+4] = 0x0A;
            TempASCII[j+5] = 0x00;

            break;
            }
    }
    tabASCII[18+j] = TempASCII[j];
    tabASCII[19+j] = TempASCII[j+1];
    tabASCII[20+j] = TempASCII[j+2];
    tabASCII[21+j] = TempASCII[j+3];
    tabASCII[22+j] = TempASCII[j+4];
    tabASCII[23+j] = TempASCII[j+5];
    

}
/*******************************************************************
 * Function:  Calculates and sets the next RTCC alarm with the current
 *time/date and the logging period
 *
 * PreCondition:    Logging OK and started
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:
 *
 *
 *
 *
 * Note:            There is a case where this function fails:
 *                  At leap years, on the 26/28th of february
 *                  
 *******************************************************************/
void RtccSetNextAlarm(void)
{
    static rtccTimeDate Current;
    static DWORD x;

    if(PERIOD_OK)
    {
        RtccReadTimeDate(&Current);
        x = periode;
        while (x>0)
        {
            x--;
            Current.f.sec++;
            if( (Current.f.sec & 0x0F) >= 0x0A )
            {
                Current.f.sec = Current.f.sec +0x10;
                Current.f.sec = Current.f.sec & 0xF0;
                if ( (Current.f.sec & 0xF0) == 0x60)
                {
                    Current.f.sec = 0x00;
                    Current.f.min++;
                    if( (Current.f.min & 0x0F) >= 0x0A )
                        {
                        Current.f.min = Current.f.min +0x10;
                        Current.f.min = Current.f.min & 0xF0;
                            if ( (Current.f.min & 0xF0) == 0x60)
                            {
                                Current.f.min = 0x00;
                                Current.f.hour++;
                                if( ( (Current.f.hour & 0x0F) >= 0x0A ) || (Current.f.hour == 0x24) )
                                    {
                                    Current.f.hour = Current.f.hour +0x10;
                                    Current.f.hour = Current.f.hour & 0xF0;
                                        if (Current.f.hour  == 0x30)
                                        {
                                            Current.f.hour = 0x00;
                                            Current.f.mday ++;
                                            if( ((Current.f.mday & 0x0F) == 0x0A) || ((Current.f.mday == 0x32)&&( (Current.f.mon  == 1)||(Current.f.mon  == 0x03)||(Current.f.mon  == 0x05)||(Current.f.mon  == 0x07)||(Current.f.mon  == 0x08)||(Current.f.mon  == 0x10)||(Current.f.mon  == 0x12) ))  ||  ((Current.f.mday == 0x31)&&((Current.f.mon== 0x02)||(Current.f.mon  == 0x04)||(Current.f.mon  == 0x06)||(Current.f.mon  == 0x09)||(Current.f.mon  == 0x011)) ))
                                            {
                                                Current.f.mday = Current.f.mday +0x10;
                                                Current.f.mday = Current.f.mday & 0xF0;
                                                if (Current.f.mday  == 0x40)
                                                {
                                                    Current.f.mday = 0x01;//0x00 to 0x01
                                                    Current.f.mon ++;
                                                    if( (Current.f.mon & 0x0F) == 0x0A || (Current.f.mon == 0x13))
                                                    {
                                                        Current.f.mon = Current.f.mon +0x10;
                                                        Current.f.mon = Current.f.mon & 0xF0;
                                                        if (Current.f.mon  == 0x20)
                                                        {
                                                            Current.f.mon = 0x01;
                                                            Current.f.year ++;
                                                        }
                                                    }    



                                                }
                                            }    



                                        }
                                    }


                            }
                        }
                }
            }

        }
    }

    RtccWriteAlrmTimeDate(&Current);
    ALRMRPT = 0x00;
    ALRMCFGbits.AMASK = 0x09;
}
/*******************************************************************
 * Function: Temperature logging routine called at each RTCC alarm
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void AcquisitionRoutine(void)
{
     static int SampleLength;
     static DWORD_VAL FatDWordA,FatDWordB;
     static int tempor;
     int i;
  
    mRtccAlrmDisable();
    StartOneShotTemp();         //Start the temperature acquisition
//    mLED_LP_On()
    RtccReadTimeDate(&TimeDate);    //read the time and date
    DtASCII(SampleASCII);           //Convert it to ASCII
    
    while(OneShotTempWait());   //Wait till the temperature conversion is over
    
    OneShotTempRead(SampleASCII);
    SampleLength = strlen(SampleASCII);

    //We now have our string with time/date and temperature. Write it to the buffer.
    if(!FULL_OS)
    {
        if( (LogInnerBufferPtr + SampleLength) <0x200)
            {//If the current buffer isn't full
            memcpy((void*)(gDataBuffer+LogInnerBufferPtr),(void*)SampleASCII,(int)SampleLength);
            LogInnerBufferPtr = LogInnerBufferPtr + SampleLength;
            }
        else
            {
            //Exit the EEPROM DEEP SLEEP MODE
            MDD_SDSPI_InitIO();

            //We need to make a new entry in the FAT. But first, write the new sector with the temperature logs
            WordPtr2 = 0;
            for (WordPtr1=LogInnerBufferPtr;WordPtr1<0x200;WordPtr1++)
            {
                gDataBuffer[WordPtr1] = SampleASCII[WordPtr2]; //We fill the buffer till the end. After, we'll write the
                //remaining part of the new temperature log to the new buffer
                WordPtr2++;
            }
           
            tempor =  MDD_SDSPI_WriteLowPower(LogBufferPtr, (BYTE*)gDataBuffer);

            
            EEPROM_Sleep();

            LogBufferPtr ++; //Increment the sector ptr for the new data
            LogInnerBufferPtr = SampleLength-WordPtr2;
            memset(gDataBuffer,0x00,0x200);//erasing the buffer
            //Writing the remaining part of the temperature measurement to the new buffer
            WordPtr1 = 0;
            while (WordPtr1 <LogInnerBufferPtr)
            {
                gDataBuffer[WordPtr1] = SampleASCII[WordPtr2];
                WordPtr1++;
                WordPtr2++;
            }

            if(LogBufferPtr > 0x2A6)/////2A9 (681)
                FULL_OS = 1;
            //Re-enter the EEPROM DEEP SLEEP MODE
            

        }//end else (LogInnerBufferPtr + SampleLength) <0x200)
 
    RtccSetNextAlarm();
    mRtccAlrmEnable();
    }//end if(!FULL_OS )
    else {//the drive is full. Signal Error (LED?)
       // mLED_LP_On();
        mLED_LP_Off()
        Sleep();
    }
mLED_LP_Off()
}
/*******************************************************************
 * Function: Configures the logger to start the temp logging
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void StartLogging(void)
{

    INTCONbits.GIE = 0;
    //Prepare the FS for a new temperature acq.
    WriteEnable = 1;
 
    LogInnerBufferPtr = 0x00; //Pointer inside the buffer with the Temperature logs
    LogBufferPtr = FIRST_DATA_SECTOR + 2; //Pointer for the current sector of the drive.
  
    //Disable the USB interrupt
    PIE2bits.USBIE = 0;
    //Perform more energy-saving stuff


    //Enable the RTCC Interrupt
    PIE3bits.RTCCIE = 1;

    //Enter the EEPROM DEEP SLEEP MODE
    EEPROM_Sleep();
   
    memset(gDataBuffer,0x00,0x200);//Erase the temporary buffer

    RtccSetNextAlarm();
    mRtccAlrmEnable();
    LOGGING_ON = 1;
  //  INTCONbits.GIE = 1; //Enable all interrupts
}
/*******************************************************************
 * Function: Stopps the logging and writes the FAT and File Directory
 *to match the recorded temperature samples (this is done once plugged
 *to USB, to save battery power)
 * PreCondition:    Logging started
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:
 *The function scans the EEPROM and searches for the last temperature record.
 * Fills the FAT accordingly.
 *
 *
 *
 * Note:            None
 *******************************************************************/

void StopLogging(void) {
    WORD n,x;
    WORD FatSector,FATEntryPtr;
    WORD CurrentFATPagePtr;
    DWORD LogFileSize;
    DWORD SearchPtr;
    WORD SearchInnerPtr;
    int Found;
    BYTE tmpvar;


    
    MDD_SDSPI_InitIO();
    Found = 0;

    for(SearchPtr = FIRST_DATA_SECTOR+2; SearchPtr <= 1360;SearchPtr++)
    {
        //We will scan the EEPROM data.csv zone to get the last recorded log
        MDD_SDSPI_SectorRead((DWORD)SearchPtr, (BYTE*)msd_buffer);

            tmpvar = msd_buffer[0];
            if(tmpvar == 0xFF)
                break; 
    }

    LogBufferPtr = SearchPtr;

        for (SearchInnerPtr = 0;SearchInnerPtr < 0x200;SearchInnerPtr++)
        {
            tmpvar = gDataBuffer[SearchInnerPtr];
            if(tmpvar == 0)
                break;
        }

    

    LogInnerBufferPtr = SearchInnerPtr;

    for (n = LogInnerBufferPtr; n < 0x200; n++)
        gDataBuffer[n] = 0x00;//Erase the remaining bytes of the buffer
    
    MDD_SDSPI_SectorWrite((DWORD)LogBufferPtr, (BYTE*)gDataBuffer,TRUE);
   
    //See if we need to add a new FAT entry
    if (LogBufferPtr >(FIRST_DATA_SECTOR + 2))
    {
        //We have at least one FAT entry to add

        //First, start with a brand new FAT
        FATEntryPtr = 0;
        CurrentFATPagePtr = FAT_LOCATION;
        FillFAT_msd();
        memset(gDataBuffer,0x00,0x200);//Erase the temporary buffer
        FatSector = FAT_LOCATION + 2;
        while ( (FatSector- FAT_LOCATION - 2) < (LogBufferPtr-FIRST_DATA_SECTOR - 2))
        {
            //For every Sector of data, add a FAT entry
        AddFAT(FatSector);
        if(FullOS==2)
            break;
        FatSector++;
        }

        if(FullOS == 1)
        {
            MDD_SDSPI_SectorWrite((DWORD)FAT_LOCATION, (BYTE*)msd_buffer, TRUE);//FAT(0)
            MDD_SDSPI_SectorWrite((DWORD)(FAT_LOCATION+5), (BYTE*)msd_buffer, TRUE);//And same for FAT(1)
            MDD_SDSPI_SectorWrite((DWORD)FAT_LOCATION+1, (BYTE*)gDataBuffer, TRUE);//FAT(0)
            MDD_SDSPI_SectorWrite((DWORD)(FAT_LOCATION+6), (BYTE*)gDataBuffer, TRUE);//And same for FAT(1)
        }
        if(FullOS == 0)
        {
            MDD_SDSPI_SectorWrite((DWORD)FAT_LOCATION, (BYTE*)msd_buffer, TRUE);//FAT(0)
            MDD_SDSPI_SectorWrite((DWORD)(FAT_LOCATION+5), (BYTE*)msd_buffer, TRUE);//And same for FAT(1)
        }

    }
            //Maybe add a temporization to wait till the EEPROM write is over instead of reading the SPI with EEPROMBusy()
          
    FillDirectory();

    LogBufferPtr = LogBufferPtr - FIRST_DATA_SECTOR - 1;
    LogBufferPtr = (LogBufferPtr << 9);
    LogFileSize = LogBufferPtr;
    gDataBuffer[0x7C] = (BYTE) LogFileSize; //File Size (number of sectors)
    gDataBuffer[0x7D] = (BYTE) (LogFileSize >> 8);
    gDataBuffer[0x7E] = (BYTE) (LogFileSize >> 16);
    gDataBuffer[0x7F] = (BYTE) (LogFileSize >> 24);
    MDD_SDSPI_SectorWrite(DIRECTORY_LOCATION, gDataBuffer, TRUE);


 
}
/*******************************************************************
 * Function: Adds a new entry in the FAT12 
 *
 *
 * PreCondition:    FAT copied in the msd_buffer, FatSector pointed
 *on the current FAT entry (the first entry of the file. It has to
 * be incremented outside this function)
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void AddFAT(WORD FatSector)
{
    static DWORD_VAL FatDWordA, FatDWordB;
    WORD FATEntryPtr,x;
    BYTE Tmp0,Tmp1,Tmp2;
    //Adding the new FAT entry with the 12 bit system. Next time, I'll do a HDD based logger with a serial port.
    //FatSector is our current FAT sector. We want to add a new entry cuz we finished a buffer full of temp logs.

    if (FatSector & 1) //Odd Number
    {
        FATEntryPtr = FatSector - 1;
        FATEntryPtr = FATEntryPtr + (FATEntryPtr >> 1); // *1,5


        if(FATEntryPtr > 0x1FB)
            FullOS = 1;

        if(FATEntryPtr > 0x3FB)
        {
            FullOS = 2;
            MDD_SDSPI_SectorWrite((DWORD)FAT_LOCATION, (BYTE*)msd_buffer, TRUE);//FAT(0)
            MDD_SDSPI_SectorWrite((DWORD)(FAT_LOCATION+5), (BYTE*)msd_buffer, TRUE);//And same for FAT(1)
            MDD_SDSPI_SectorWrite((DWORD)FAT_LOCATION+1, (BYTE*)gDataBuffer, TRUE);//FAT(0)
            MDD_SDSPI_SectorWrite((DWORD)(FAT_LOCATION+6), (BYTE*)gDataBuffer, TRUE);//And same for FAT(1)
            return;
        }

        FatDWordA.w[0] = ((FatSector + 1) & 0xFFF);
        FatDWordA.w[1] = ((FatSector) & 0xFFF);
        //We have:  0x yz 0X YZ and we want:
        //          00 yz Zx XY
        FatDWordB.v[0] = ((FatDWordA.v[1]&0x0F) << 4) | ((FatDWordA.v[0]&0xF0) >> 4);
        FatDWordB.v[1] = ((FatDWordA.v[3]&0x0F)) | ((FatDWordA.v[0]&0x0F) << 4);
        FatDWordB.v[2] = FatDWordA.v[2];
        //The FatDWordB Contains what we want to write to the FAT, for the current Sector and the next one.
        //But we're still using the same buffer, so we have to adapt the variable pointing into that buffer
        //to loop if write to the next FAT page
//        FATEntryPtr &= 0x1FF;


        msd_buffer[FATEntryPtr]   = FatDWordB.v[2];
        msd_buffer[FATEntryPtr+1] = FatDWordB.v[1];
        msd_buffer[FATEntryPtr+2] = FatDWordB.v[0];
        msd_buffer[FATEntryPtr+3] = 0xFF; //And write the end of the file for the next sector entry
        msd_buffer[FATEntryPtr+4] = 0x0F;

    } else //Even Number
    {

        FATEntryPtr = FatSector;
        FATEntryPtr = FATEntryPtr + (FATEntryPtr >> 1); // *1,5
        if(FATEntryPtr > 0x1FD)
            FullOS = 1;
        if(FATEntryPtr > 0x3FD)
        {
            FullOS = 1;
            MDD_SDSPI_SectorWrite((DWORD)FAT_LOCATION, (BYTE*)msd_buffer, TRUE);//FAT(0)
            MDD_SDSPI_SectorWrite((DWORD)(FAT_LOCATION+5), (BYTE*)msd_buffer, TRUE);//And same for FAT(1)
            MDD_SDSPI_SectorWrite((DWORD)FAT_LOCATION+1, (BYTE*)gDataBuffer, TRUE);//FAT(0)
            MDD_SDSPI_SectorWrite((DWORD)(FAT_LOCATION+6), (BYTE*)gDataBuffer, TRUE);//And same for FAT(1)
            return;
        }
         


        //We want to write the next sector address in the current one, plus the end of the file on the next sector
        FatDWordA.w[0] = 0x0FFF; //End of the file for the next sector
        //FatDWordA.w[0] = 0x0F00; //End of the file for the next sector <- No, don't
        FatDWordA.w[1] = ((FatSector + 1) & 0xFFF); //In the current sector: the next sector address.
        //We have:  0x yz 0X YZ and we want:
        //          00 yz Zx XY
        FatDWordB.v[0] = ((FatDWordA.v[1]&0x0F) << 4) | ((FatDWordA.v[0]&0xF0) >> 4);
        FatDWordB.v[1] = ((FatDWordA.v[3]&0x0F)) | ((FatDWordA.v[0]&0x0F) << 4);
        FatDWordB.v[2] = FatDWordA.v[2];
        //The FatDWordB Contains what we want to write to the FAT, for the current Sector and the next one.
        //But we're still using the same buffer, so we have to adapt the variable pointing into that buffer
        //to loop if write to the next FAT page
//        FATEntryPtr &= 0x1FF;

        msd_buffer[FATEntryPtr]   = FatDWordB.v[2];
        msd_buffer[FATEntryPtr+1] = FatDWordB.v[1];
        msd_buffer[FATEntryPtr+2] = FatDWordB.v[0];
        

    }

}
/*******************************************************************
 * Function:    void EEPROM_SLEEP(void)
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:    Puts the EEPROM in Deep sleep mode and/or configures the PIC pins for low energy
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void EEPROM_Sleep(void)
{
    #if defined USE_ATMEL_FLASH
    //Put the ATMEL EEPROM IN SLEEP MODE
    MDD_SDSPI_InitIO();
    SD_CS = 1;
    WriteSPIM( DEEP_PWR_DOWN  );
    #endif

    SPICON1 = 0x00;

    SD_CS_TRIS = 1;
    SPICLOCKTRIS = 1;
    SPIINTRIS = 1;
    SPIOUTTRIS = 1;

    TRISA = 0xFF;
    TRISB = 0xFF;
    TRISC = 0xFF;


    TRISBbits.TRISB3 = INPUT_PIN;   //NC pin
    TRISBbits.TRISB6 = INPUT_PIN;   //PGC
    TRISBbits.TRISB7 = INPUT_PIN;   //PGD
    TRISCbits.TRISC2 = INPUT_PIN;   //LED
    ANCON0 = 0b11111111; //DIGITAL input
    ANCON1 = 0b00011111; //DIGITAL
    mLED_1_Off()



}
/*******************************************************************
 * Function:    void DelayLED(void)
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:    Delay to make the LED flash in Low power mode
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void DelayLED(void)
{
    volatile BYTE    ms;
    
    ms = 0xA0;
    while (ms--)
    {
        Nop();
    }
   // return;
}

/*******************************************************************
 * Function:    void Delay2OSC10ms(void)
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:    10ms Delay  in Low power mode (second OSC, 32.765 KHz) function call and
 * return included (measured:)
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void Delay2OSC10ms(void)
{
    volatile BYTE    ms;

    ms = 0x07;
     while (ms--)
    {
        Nop();
    }
    Nop();
    Nop();
   // return;
}



/*******************************************************************
 * Function:    void SlowClock(void)
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    Configures the I2C and SPI buses to the right speed
 *
 * Overview:    Switches the OSC to T1OSC, 32,768Khz Qz
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void SlowClock(void)
{
    T1CONbits.T1OSCEN = 1;
    OSCCONbits.SCS0 = 1;
    OSCCONbits.SCS1 = 0;
    OSCTUNEbits.PLLEN = 0;  //DISABLE the PLL 
    
    SSP2CON1 = SSP2CON1 & 0b11110000;
    SSP1ADD = 0x01; //I2C speed (Temperature sensor)
}
/*******************************************************************
 * Function:    void FastClock(void)
 *
 *
 * PreCondition:    None
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    Configures the I2C and SPI buses to the right speed
 *
 * Overview:    Switches the OSC to PLL
 *
 *
 *
 *
 * Note:            None
 *******************************************************************/
void FastClock(void)
{
    //T1CONbits.T1OSCEN = 1;
    OSCCONbits.SCS0 = 0;
    OSCCONbits.SCS1 = 0;
    OSCTUNEbits.PLLEN = 1;  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
    Delayms(2);
#if defined USE_ATMEL_FLASH
    SSP2CON1 = SSP2CON1 & 0b11110001;
    SSP2CON1 = SSP2CON1 | 0b00000001;
#endif
#if defined USE_SST_FLASH
    SSP2CON1 = 0b00000001;
#endif
    SSP1ADD = 0x77; //I2C speed (Temperature sensor)

}

/*******************************************************************
 * Function:    void EraseFileSystem(void)
 *
 *
 * PreCondition:    USB POWER present, SW1 pressed > 3 seconds
 *
 * Input:
 *
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:    Overwrites the EEPROM files and filesystem
 *              by the default values
 *
 *
 *
 * Note:       This function can be used to test your EEPROM writing/reading routines, see the commented-out lines
 *******************************************************************/
void EraseFileSystem(void)
{
    BYTE resultats,fl;
    extern const rom char Config0[0xFF];
    extern const rom char Instr0[0xFF];
    extern const  rom char LOG_NOT_STARTED;
    extern const rom char Config1[0xFF];

    WORD i;
MDD_SDSPI_InitIO();
    mLED_1_On();

    INTCONbits.GIE = 0;
    
    MDD_SDSPI_SectorRead(0x6AF, gDataBuffer);
    if((gDataBuffer[0]!=0xFF)&&(gDataBuffer[1]!=0xFF))
    {
        LoggerID = gDataBuffer[0];
        LoggerID = LoggerID << 8;
        LoggerID |= gDataBuffer[1];
        ultoa(LoggerID,idString);

    }
    
    EEPROM_EraseEEPROM();
    if(LoggerID)//We have to rewrite the loggerID (and other data if there are in the future) because we just wiped the EEPROM
    {
         memset(gDataBuffer,0xFF,0x200);
         gDataBuffer[1] = (LoggerID&0x00FF);
         gDataBuffer[0] = ((LoggerID>>8)&0x00FF);
         MDD_SDSPI_SectorWrite(0x6AF,gDataBuffer, TRUE);
    }
MDD_SDSPI_SectorRead(0x6AF, gDataBuffer);


 //   memset(gDataBuffer,0xBB,0x200);
 //   MDD_SDSPI_SectorWrite(0x0000, gDataBuffer, TRUE);

//    Nop();
  //    MDD_SDSPI_SectorRead(0x00, gDataBuffer);


    FillMBR();
    Nop();
    //  memcpypgm2ram ( gDataBuffer, (ROM void*)MASTER_BOOT_RECORD_ADDRESS ,MEDIA_SECTOR_SIZE );
    MDD_SDSPI_SectorWrite(0x0000, gDataBuffer, TRUE);
   //memset(gDataBuffer,0xBB,0x200);
//      MDD_SDSPI_SectorRead(0x00, gDataBuffer);

    FillBootSector();

    MDD_SDSPI_SectorWrite(0x01,gDataBuffer, TRUE);
// MDD_SDSPI_SectorRead(0x01, gDataBuffer);
    //Have to clear all the FAT entries (or it's FF else, value of unwritten EEPROM and the OS thinks it's an error. They must be '00')
    //We first erase the pages after the first FAT page (two times, for each copy of the FAT)
    memset(gDataBuffer,0x00,0x200);
    //  MDD_SDSPI_SectorWrite(FAT_LOCATION, gDataBuffer, TRUE);
     MDD_SDSPI_SectorWrite(FAT_LOCATION+1, gDataBuffer, TRUE);
    MDD_SDSPI_SectorWrite(FAT_LOCATION+2, gDataBuffer, TRUE);
    MDD_SDSPI_SectorWrite(FAT_LOCATION+3, gDataBuffer, TRUE);
    MDD_SDSPI_SectorWrite(FAT_LOCATION+4, gDataBuffer, TRUE);
    //   MDD_SDSPI_SectorWrite(FAT_LOCATION+5, gDataBuffer, TRUE);
    MDD_SDSPI_SectorWrite(FAT_LOCATION+6, gDataBuffer, TRUE);
    MDD_SDSPI_SectorWrite(FAT_LOCATION+7, gDataBuffer, TRUE);
    MDD_SDSPI_SectorWrite(FAT_LOCATION+8, gDataBuffer, TRUE);
    MDD_SDSPI_SectorWrite(FAT_LOCATION+9, gDataBuffer, TRUE);
    //Now, we write the two copies of the FAT
    FillFAT();
    MDD_SDSPI_SectorWrite(FAT_LOCATION, gDataBuffer, TRUE);
    MDD_SDSPI_SectorWrite(FAT_LOCATION+5, gDataBuffer, TRUE);
    //    MDD_SDSPI_SectorRead(FAT_LOCATION, gDataBuffer);
    // MDD_SDSPI_SectorRead(0x06, gDataBuffer);
    //Nop();

    FillDirectory();
    MDD_SDSPI_SectorWrite(DIRECTORY_LOCATION, gDataBuffer, TRUE);
    memset(gDataBuffer,0x00,0x200);

    for (fl=DIRECTORY_LOCATION+1;fl<FIRST_DATA_SECTOR;fl++)
    {
     MDD_SDSPI_SectorWrite(fl, gDataBuffer, TRUE);
    }

    //   MDD_SDSPI_SectorRead(DIRECTORY_LOCATION, gDataBuffer);

    //FillSlack();
    memset(gDataBuffer,0x00,0x200);
    memcpypgm2ram((void*)gDataBuffer,Config0,0xC9);
    //memcpypgm2ram(gDataBuffer + 0x100, Config1,0x100);
    MDD_SDSPI_SectorWrite(FIRST_DATA_SECTOR, gDataBuffer, TRUE);
    //Nop();
    //MDD_SDSPI_SectorRead(FIRST_DATA_SECTOR, gDataBuffer);

    //memset(gDataBuffer,0x00,0x200);
    memcpypgm2ram((void*)gDataBuffer,Instr0,0xFF);
    //memcpypgm2ram(gDataBuffer + 0x100, Config1,0x100);
     MDD_SDSPI_SectorWrite(FIRST_DATA_SECTOR +1, gDataBuffer, TRUE);
    
    mLED_1_Off();
}

/** EOF main.c ***************************************************************/
