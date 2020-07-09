/********************************************************************
 FileName:		main.c
 Dependencies:	See INCLUDES section
 Processor:		PIC18 or PIC24 USB Microcontrollers
 Hardware:		The code is natively intended to be used on the following
 				hardware platforms: PICDEMÅEFS USB Demo Board, 
 				PIC18F87J50 FS USB Plug-In Module, or
 				Explorer 16 + PIC24 USB PIM.  The firmware may be
 				modified for use on other USB platforms by editing the
 				HardwareProfile.h file.
 Complier:  	Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:		Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the ìCompanyÅE for its PICÆ Microcontroller is intended and
 supplied to you, the Companyís customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN ìAS ISÅECONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
********************************************************************/

/** INCLUDES *******************************************************/
#include "Compiler.h"
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb.h"
#include "USB/usb_function_generic.h"

#include "emu_ft245_eeprom.h"

#include <spi.h>


/** CONFIGURATION **************************************************/
#if defined(PICDEM_FS_USB)      // Configuration bits for PICDEM FS USB Demo Board (based on PIC18F4550)
        #pragma config PLLDIV   = 5         // (20MHz/4Mhz)
        #pragma config CPUDIV   = OSC1_PLL2   
        #pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
        #pragma config FOSC     = HSPLL_HS
        #pragma config FCMEN    = OFF
        #pragma config IESO     = OFF
        #pragma config PWRT     = OFF
        #pragma config BOR      = ON
        #pragma config BORV     = 3
        #pragma config VREGEN   = ON      //USB Voltage Regulator
        #pragma config WDT      = OFF
        #pragma config WDTPS    = 32768
        #pragma config MCLRE    = ON
        #pragma config LPT1OSC  = OFF
        #pragma config PBADEN   = OFF
//      #pragma config CCP2MX   = ON
        #pragma config STVREN   = ON
        #pragma config LVP      = OFF
//      #pragma config ICPRT    = OFF       // Dedicated In-Circuit Debug/Programming
        #pragma config XINST    = OFF       // Extended Instruction Set
        #pragma config CP0      = OFF
        #pragma config CP1      = OFF
//      #pragma config CP2      = OFF
//      #pragma config CP3      = OFF
        #pragma config CPB      = OFF
//      #pragma config CPD      = OFF
        #pragma config WRT0     = OFF
        #pragma config WRT1     = OFF
//      #pragma config WRT2     = OFF
//      #pragma config WRT3     = OFF
        #pragma config WRTB     = OFF       // Boot Block Write Protection
        #pragma config WRTC     = OFF
//      #pragma config WRTD     = OFF
        #pragma config EBTR0    = OFF
        #pragma config EBTR1    = OFF
//      #pragma config EBTR2    = OFF
//      #pragma config EBTR3    = OFF
        #pragma config EBTRB    = OFF

#else
	#error "Hardware profile is not specific!"
#endif

typedef union DATA_PACKET{
    BYTE  _byte[USBGEN_EP_SIZE];  //For byte access
    WORD  _word[USBGEN_EP_SIZE/2];//For word access(USBGEN_EP_SIZE msut be even)
    DWORD _dword[USBGEN_EP_SIZE/4];
} DATA_PACKET;

//output latch, input port definitions
#define LTCK LATBbits.LATB5		//JTAG: TCK
#define LTMS LATBbits.LATB3		//JTAG: TMS
#define LnCE LATBbits.LATB6		//AS  : nCE
#define LnCS LATBbits.LATB0		//AS  : nCS
#define LTDI LATBbits.LATB1		//JTAG: TDI
#define PTDO PORTBbits.RB4		//JTAG: TDO
#define PADO PORTBbits.RB2		//AS  : DataOut ,PS  :nSTATUS
#define LACT LATCbits.LATC2		//Activitty LED

//tris definitions()
#define TTDO TRISBbits.TRISB4
#define TDTO TRISBbits.TRISB2

BYTE LOPE;					//Virtual output enable

#define BUFC 4

/** VARIABLES ******************************************************/
#pragma udata USB_VARS
volatile BYTE ctrl_trf_buf[8];
volatile BYTE indicator[2];
#pragma udata usb5=0x500
volatile DATA_PACKET INPacket[BUFC];
#pragma udata usb6=0x600
volatile DATA_PACKET OUTPacket[BUFC];
#pragma udata
USB_HANDLE USBGenericOutHandle=0;
USB_HANDLE USBGenericInHandle=0;
BYTE OutLen;		//out packet length
BYTE op;			//out packet number
BYTE jtag_byte;	//byte count fot jtag mode
BYTE aser_byte;	//byte count fot active-serial mode
BYTE read;		//simulteneous reading flag
BYTE ip;			//in packet number
BYTE ic;			//in filled byte count
BYTE wp;			//in current packet number

/** PRIVATE PROTOTYPES *********************************************/
void USBDeviceTasks(void);
void YourHighPriorityISRCode(void);
void YourLowPriorityISRCode(void);

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
	//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
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
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.

	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
	#pragma interruptlow YourHighPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
	
	}	//This return will be a "retfie", since this is in a #pragma interruptlow section 

#endif //of "#if defined(__18CXX)"

//queueing to EP1IN
void enqueue(BYTE d){
	if(ic<2){
		INPacket[wp]._word[0]=0x6031;
		ic=2;
	}
	INPacket[wp]._byte[ic++]=d;
	if(USBGEN_EP_SIZE<=ic){
		//not checked buffer over run
		wp++;
		if (BUFC<=wp) wp=0;
		ic=0;
	}
}

void JTAG_Write(BYTE a){
// 7cycles/bit -> 1.71MHz
//bit 0 (0x01)
	if(  a&0x01 ) LTDI=1;
	if(!(a&0x01)) LTDI=0;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 1 (0x02)
	if(  a&0x02 ) LTDI=1;
	if(!(a&0x02)) LTDI=0;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 2 (0x04)
	if(  a&0x04 ) LTDI=1;
	if(!(a&0x04)) LTDI=0;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 3 (0x08)
	if(  a&0x08 ) LTDI=1;
	if(!(a&0x08)) LTDI=0;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 4 (0x10)
	if(  a&0x10 ) LTDI=1;
	if(!(a&0x10)) LTDI=0;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 5 (0x20)
	if(  a&0x20 ) LTDI=1;
	if(!(a&0x20)) LTDI=0;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 6 (0x40)
	if(  a&0x40 ) LTDI=1;
	if(!(a&0x40)) LTDI=0;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 7 (0x80)
	if(  a&0x80 ) LTDI=1;
	if(!(a&0x80)) LTDI=0;
	LTCK=1;
	Nop();
	LTCK=0;
}

BYTE JTAG_RW(BYTE a){
// 9cycles/bit -> 1.33MHz
	BYTE ret=0;
//bit 0 (0x01)
	if(  a&0x01 ) LTDI=1;
	if(!(a&0x01)) LTDI=0;
	if(PTDO) ret|=0x01;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 1 (0x02)
	if(  a&0x02 ) LTDI=1;
	if(!(a&0x02)) LTDI=0;
	if(PTDO) ret|=0x02;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 2 (0x04)
	if(  a&0x04 ) LTDI=1;
	if(!(a&0x04)) LTDI=0;
	if(PTDO) ret|=0x04;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 3 (0x08)
	if(  a&0x08 ) LTDI=1;
	if(!(a&0x08)) LTDI=0;
	if(PTDO) ret|=0x08;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 4 (0x10)
	if(  a&0x10 ) LTDI=1;
	if(!(a&0x10)) LTDI=0;
	if(PTDO) ret|=0x10;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 5 (0x20)
	if(  a&0x20 ) LTDI=1;
	if(!(a&0x20)) LTDI=0;
	if(PTDO) ret|=0x20;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 6 (0x40)
	if(  a&0x40 ) LTDI=1;
	if(!(a&0x40)) LTDI=0;
	if(PTDO) ret|=0x40;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 7 (0x80)
	if(  a&0x80 ) LTDI=1;
	if(!(a&0x80)) LTDI=0;
	if(PTDO) ret|=0x80;
	LTCK=1;
	Nop();
	LTCK=0;
	return ret;
}

BYTE ASer_RW(BYTE a){
// 9cycles/bit -> 1.33MHz
	BYTE ret=0;
//bit 0 (0x01)
	if(  a&0x01 ) LTDI=1;
	if(!(a&0x01)) LTDI=0;
	if(PADO) ret|=0x01;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 1 (0x02)
	if(  a&0x02 ) LTDI=1;
	if(!(a&0x02)) LTDI=0;
	if(PADO) ret|=0x02;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 2 (0x04)
	if(  a&0x04 ) LTDI=1;
	if(!(a&0x04)) LTDI=0;
	if(PADO) ret|=0x04;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 3 (0x08)
	if(  a&0x08 ) LTDI=1;
	if(!(a&0x08)) LTDI=0;
	if(PADO) ret|=0x08;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 4 (0x10)
	if(  a&0x10 ) LTDI=1;
	if(!(a&0x10)) LTDI=0;
	if(PADO) ret|=0x10;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 5 (0x20)
	if(  a&0x20 ) LTDI=1;
	if(!(a&0x20)) LTDI=0;
	if(PADO) ret|=0x20;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 6 (0x40)
	if(  a&0x40 ) LTDI=1;
	if(!(a&0x40)) LTDI=0;
	if(PADO) ret|=0x40;
	LTCK=1;
	Nop();
	LTCK=0;
//bit 7 (0x80)
	if(  a&0x80 ) LTDI=1;
	if(!(a&0x80)) LTDI=0;
	if(PADO) ret|=0x80;
	LTCK=1;
	Nop();
	LTCK=0;
	return ret;
}

/** DECLARATIONS ***************************************************/
#pragma code

/******************************************************************************
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
	BYTE i;
	BYTE *p;
	BYTE tmp;
	ADCON1=0x0F;
	TRISA=0;
	TRISB=0;
	TRISC=0;
	TTDO=1;
	TDTO=1;
	INTCON2bits.RBPU=0;	//pull-up:on
	T0CON=0b10000000;		//On,Fcy/2
	INTCONbits.TMR0IE=0;	//Polling
	eeprom_init();		//see ft245_eepromv2.c
	USBDeviceInit();
	#ifdef LACT
	LACT=1;
	#endif
	while(1)
	{
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // frequently (such as once about every 100 microseconds) at any
        				  // time that a SETUP packet might reasonably be expected to
        				  // be sent by the host to your device.  In most cases, the
        				  // USBDeviceTasks() function does not take very long to
        				  // execute (~50 instruction cycles) before it returns.

		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
	    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) {
			//Something to do in not connected to host.
		}else{
			if(!USBHandleBusy(USBGenericInHandle)){
				if(wp!=ip){				//send filled packet to host
					USBGenericInHandle=USBGenWrite(1,(BYTE*)&INPacket[ip],USBGEN_EP_SIZE);
					ip++;
					if(BUFC<=ip) ip=0;
					INTCONbits.TMR0IF=1;	//schedule to send indicator			
				}else if(ic){			//send non-filled packet to host
					USBGenericInHandle=USBGenWrite(1,(BYTE*)&INPacket[ip],ic);
					ip++;
					if(BUFC<=ip) ip=0;
					wp=ip;
					ic=0;
					INTCONbits.TMR0IF=1;	//schedule sending indicator
				}else if(INTCONbits.TMR0IF){	//send packet indicator
					TMR0H=5536>>8;	//(65536-5536)/(12000/2)=10ms
					TMR0L=5536&0xFF;
					INTCONbits.TMR0IF=0;
					indicator[0]=0x31;
					indicator[1]=0x60;
					USBGenericInHandle=USBGenWrite(1,(BYTE*)indicator,2);
				}
			}

	         //Check to see if data has arrive.
	         if(!USBHandleBusy(USBGenericOutHandle)){
				#ifdef LACT
				LACT=0;
				#endif
				OutLen=USBHandleGetLength(USBGenericOutHandle);
				p=(BYTE*)&OUTPacket[op];
				op++;
				if(BUFC<=op) op=0;
				USBGenericOutHandle=USBGenRead(2,(BYTE*)&OUTPacket[op],USBGEN_EP_SIZE);
				for(i=0;i<OutLen;i++){
					tmp=p[i];
					if(jtag_byte){
						if(read) enqueue(JTAG_RW(tmp));
						else JTAG_Write(tmp);
						jtag_byte--;
					}else if(aser_byte){
						if(read) enqueue(ASer_RW(tmp));
						else JTAG_Write(tmp);
						aser_byte--;
					}else{
						read=tmp&0x40;
						if(tmp&0x80){		//EnterSerialMode
							if(LnCS)	//nCS=1:JTAG
								jtag_byte=tmp&0x3F;
							else	//nCS=0:ActiveSerial
								aser_byte=tmp&0x3F;
						}else{			//BitBangMode
							if(tmp&0x01) LTCK=1; else LTCK=0;
							if(tmp&0x02) LTMS=1; else LTMS=0;
							if(tmp&0x04) LnCE=1; else LnCE=0;
							if(tmp&0x08) LnCS=1; else LnCS=0;
							if(tmp&0x10) LTDI=1; else LTDI=0;
							if(tmp&0x20) LOPE=1; else LOPE=0;
							if(read) enqueue(PADO<<1|PTDO);
						}
					}
				}
				#ifdef LACT
				LACT=1;
				#endif
			} 
		}     
    }//end while
}//end main


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
	

    #if defined(__C30__)
    #if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        TRISA &= 0xFF3F;
        LATAbits.LATA6 = 1;
        Sleep();
        LATAbits.LATA6 = 0;
    #endif
    #endif
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
            LATAbits.LATA7 = 1;
        
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
            LATAbits.LATA7 = 0;
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
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
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
 *****************************************************************************/
void USBCBCheckOtherReq(void)
{
	BYTE index;
	if(SetupPkt.RequestType==2){	//Vendor request
		if(SetupPkt.DataDir==0){	//0utput
			//Responce by sending zero-length packet
			//I don't know if this way is right, but working:)
			USBEP0SendRAMPtr(ctrl_trf_buf,0,USB_EP0_INCLUDE_ZERO);
			return;
		}
		if(SetupPkt.bRequest==0x90){
			index=(SetupPkt.wIndex<<1)&0x7E;
			ctrl_trf_buf[0]=eeprom[index];
			ctrl_trf_buf[1]=eeprom[index+1];
		}else{
			ctrl_trf_buf[0]=0x36;
			ctrl_trf_buf[1]=0x83;
		}
		USBEP0SendRAMPtr(ctrl_trf_buf,2,USB_EP0_INCLUDE_ZERO);
	}
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
 *****************************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/******************************************************************************
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
 *****************************************************************************/
void USBCBInitEP(void)
{
    USBEnableEndpoint(1,USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    USBEnableEndpoint(2,USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    USBGenericOutHandle = USBGenRead(2,(BYTE*)&OUTPacket[0],USBGEN_EP_SIZE);
    op=0;
    jtag_byte=0;
    aser_byte=0;
    ip=0;
    ic=0;
    wp=0;
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
 *					function should only be called when:
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
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            Interrupt vs. Polling
 *                  -Primary clock
 *                  -Secondary clock ***** MAKE NOTES ABOUT THIS *******
 *                   > Can switch to primary first by calling USBCBWakeFromSuspend()
 
 *                  The modifiable section in this routine should be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of 1-13 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at lest 1 ms but for no more than 15 ms."
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
    
    USBResumeControl = 1;                // Start RESUME signaling
    
    delay_count = 1800U;                // Set RESUME line for 1-13 ms
    do
    {
        delay_count--;
    }while(delay_count);
    USBResumeControl = 0;
}

/** EOF main.c ***************************************************************/
