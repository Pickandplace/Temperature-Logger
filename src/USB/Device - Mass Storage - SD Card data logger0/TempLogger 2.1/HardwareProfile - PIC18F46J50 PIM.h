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
/********************************************************************
 FileName:     	HardwareProfile - PIC18F46J50 PIM.h
 Dependencies:	See INCLUDES section
 Processor:	PIC18 USB Microcontrollers
 Hardware:	PIC18F46J50 PIM


 THIS SOFTWARE IS PROVIDED IN AN © CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************/

#ifndef HARDWARE_PROFILE_PIC18F26J50_PIM_H
#define HARDWARE_PROFILE_PIC18F26J50_PIM_H

    /*******************************************************************/
    /******** USB stack hardware selection options *********************/
    /*******************************************************************/
    //This section is the set of definitions required by the MCHPFSUSB
    //  framework.  These definitions tell the firmware what mode it is
    //  running in, and where it can find the results to some information
    //  that the stack needs.
    //These definitions are required by every application developed with
    //  this revision of the MCHPFSUSB framework.  Please review each
    //  option carefully and determine which options are desired/required
    //  for your application.

    //#define USE_SELF_POWER_SENSE_IO
    #define tris_self_power     TRISAbits.TRISA0    // Input
    #define self_power          1//PORTAbits.RA0

    //#define USE_USB_BUS_SENSE_IO
    #define tris_usb_bus_sense  TRISAbits.TRISA1    // Input
    #define USB_BUS_SENSE       1

 
    //Uncomment this to make the output HEX of this project 
    //   to be able to be bootloaded using the HID bootloader
	//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER

    /*******************************************************************/
    /******** MDD File System selection options ************************/
    /*******************************************************************/
    #define USE_PIC18

    #define USE_SD_INTERFACE_WITH_SPI

    #define USE_SST_FLASH
    //#define USE_ATMEL_FLASH

    #define TRIS_CARD_DETECT    (PRODL)//TRISBbits.TRISB4    // Input
    #define CARD_DETECT         (PRODL)//PORTBbits.RB4
    
    #define TRIS_WRITE_DETECT   (PRODL)   // Input
    #define WRITE_DETECT        (PRODL)
 
    // Chip Select Signal
    #define SD_CS               LATAbits.LATA5
    #define SD_CS_TRIS          (TRISAbits.TRISA5)

        
    // Write protect signal
    #define SD_WE               (PRODL)
    #define SD_WE_TRIS          (PRODL) 

    // TRIS pins for the SCK/SDI/SDO lines
    #define SPICLOCKTRIS            TRISCbits.TRISC6
    #define SPIINTRIS               TRISBbits.TRISB2
    #define SPIOUTTRIS              TRISCbits.TRISC7

    // Latch pins for SCK/SDI/SDO lines
    #define SPICLOCKLAT         LATCbits.LATC6
    #define SPIINLAT            LATBbits.LATB2
    #define SPIOUTLAT           LATCbits.LATC7

    // Port pins for SCK/SDI/SDO lines
    #define SPICLOCKPORT        PORTCbits.RC6
    #define SPIINPORT           PORTBbits.RB2
    #define SPIOUTPORT          PORTCbits.RC7

    // Registers for the SPI module you want to use
    #define SPICON1             SSP2CON1
    #define SPISTAT             SSP2STAT
    #define SPIBUF              SSP2BUF
    #define SPISTAT_RBF         SSP2STATbits.BF
    #define SPICON1bits         SSP2CON1bits
    #define SPISTATbits         SSP2STATbits

    #define SPI_INTERRUPT_FLAG  PIR3bits.SSP2IF 
    #define SPI_INTERRUPT_FLAG_ASM  PIR3, 7 
    #define SPIENABLE           SSP2CON1bits.SSPEN;Nop();Nop(); //Silicon bug walk around



    #define TEMP_SCK_TRIS       (TRISBbits.TRISB4)
    #define TEMP_SCK_IO		(PORTBbits.RB4)
    #define TEMP_SDA_TRIS	(TRISBbits.TRISB5)
    #define TEMP_SDA_IO		(PORTBbits.RB5)


    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/
    /******** Application specific definitions *************************/
    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/

    /** Board definition ***********************************************/
    //These defintions will tell the main() function which board is
    //  currently selected.  This will allow the application to add
    //  the correct configuration bits as wells use the correct
    //  initialization functions for the board.  These defitions are only
    //  required in the stack provided demos.  They are not required in
    //  final application design.
    #define DEMO_BOARD PIC18F26J50_PIM
    #define PIC18F26J50_PIM
    #define CLOCK_FREQ 48000000
    #define GetSystemClock() CLOCK_FREQ   
    #define GetInstructionClock() GetSystemClock()
    #define RTCC_V1
    /** LED ************************************************************/

//#define DEBUG     //Uncomment this line if you want to debug the PIC and use PGC/PGD as input/output for something else (like LEDs)

#if defined DEBUG
    #define mInitAllLEDs() Nop();
    #define mLED_1          (PRODL)
    #define mLED_1_On() Nop();
    #define mLED_1_Off() Nop();
    #define mLED_1_Toggle() Nop();
    #define mLED_LP_Off() Nop();
    #define mLED_LP_On() Nop();
#else
    #define mLED_1              LATBbits.LATB1
    #define mInitAllLEDs()      LATBbits.LATB1 = 0; TRISBbits.TRISB1 = 0;
    #define mGetLED_1()         mLED_1
    #define mLED_1_On()         TRISBbits.TRISB1 = 0; LATBbits.LATB1 = 1; TRISCbits.TRISC2 = 1;
    #define mLED_1_Off()       TRISCbits.TRISC2 = 0; LATBbits.LATB1 = 0; TRISBbits.TRISB1 = 0;LATBbits.LATB1 = 0;
    #define mLED_LP_On()           TRISCbits.TRISC2 = 0; LATBbits.LATB1 = 0; TRISBbits.TRISB1 = 1;LATCbits.LATC2 = 1;
    #define mLED_LP_Off()        LATCbits.LATC2 = 0;LATBbits.LATB1 = 0; TRISCbits.TRISC2 = 0;   TRISBbits.TRISB1 = 0;
    #define mLED_1_Toggle()     LATBbits.LATB1 = !LATBbits.LATB1;
#endif


    /** SWITCH *********************************************************/
    #define mInitSwitch1()      TRISBbits.TRISB0=1;
    #define mInitAllSwitches()  mInitSwitch1();
    #define sw1                 PORTBbits.RB0



    #define SW_PRESSED_FLAG      INTCONbits.INT0IF
    #define SW_PRESSED_INT_EN    INTCONbits.INT0IE
    #define SW_PRESSED_INT_EDGE  INTCON2bits.INTEDG0

    #define USB_POWER_INT_EN    INTCON3bits.INT1IE
    #define USB_POWER_FLAG      INTCON3bits.INT1IF
    #define USB_POWER_INT_EDGE  INTCON2bits.INTEDG1

    #define INPUT_PIN 1
    #define OUTPUT_PIN 0


    #define USB_POWER                 (PORTAbits.RA1)
    #define USB_POWER_TRIS            (TRISAbits.TRISA1)



    #define TIME_UPDATED        0x01
    #define PERIOD_CHANGED      0x02
    #define BATT_POWER          0x03
    #define LOGGING_STARTED     0x04
    //#define TIME_READ_FROM_FILE 0x05
    #define TIME_UPDATE_FAILED  0x06
    #define ERROR_IN_PERIOD     0x07
    #define PERIOD_READ_OK      0x08
    #define FULL_FAT            0x01

    #define VIRGULE_TEMPERATURE '.'

    #define TIME_OK        STATUS_FLAG.bits.b0  //We read and set up succesfuly the time from a file to the RTCC
    #define BATT_PWR       STATUS_FLAG.bits.b1  //We are on battery power only
    #define LOGGING_ON     STATUS_FLAG.bits.b2  //The logging has been started by the user
    #define PERIOD_OK      STATUS_FLAG.bits.b3  //We succesfully read the period
    #define FULL_OS        STATUS_FLAG.bits.b4  //The FAT/Data is full
    #define LOGGING_STOPPED STATUS_FLAG.bits.b5 //The user stopped the logging
    #define TIME_READ_FROM_FILE STATUS_FLAG.bits.b6
    #define LOW_BATT STATUS_FLAG.bits.b7


    //#define BATT_POWER_SENSE 1 //PORTx for battery power sense

    #define LED_SECOND      0x0400
    #define LED_MINUTE      0x0900
    #define LED_HOUR        0x1500

    #define	I2C_V5
    #define I2C_TEMP_ADDRESS_W	0b10010010
    #define I2C_TEMP_ADDRESS_R	0b10010011
    #define I2C_TEMP_TEMP_REG	0b00000000
    #define I2C_TEMP_CONF_REG	0b00000001
    #define I2C_TEMP_TEMP_LOW	0b00000010
    #define I2C_TEMP_TEMP_HIG	0b00000011
    #define I2C_TEMP_TEMP_CON1	0b10000000	//CR1 CR0 AL EM 0000
    #define I2C_TEMP_TEMP_CON2	0b01100000	//OS R1 R0 F1 F0 POL TM SD
    //CR1,CR2: Conversion rate. 00=0,25Hz 01=1Hz 01=4Hz 11=8Hz
    //AL: Read only, Temperature Alarm bit
    //EM:Ext. Mode

    //SD: Shutdown. 0=ON 1=Shutdown
    //TM: Thermostat mode
    //POL: AL bit polarity
    //F1F0: Consecutive Faults to generate an AL
    //R1R0: Read only, Resolution: 11: 12bits
    //OS: One shot Temperature conversion. 1=Start. 0=in progress 1=Done
    #define I2C_TEMP_TEMP_CON2_SDW          0b01100001	//Enter the Shutdown mode
    #define I2C_TEMP_TEMP_CON2_1CONV	0b11100001	//Start a single conversion

#endif  //HARDWARE_PROFILE_PIC18F26J50_PIM_H
