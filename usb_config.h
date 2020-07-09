/********************************************************************
 FileName:     	usb_config.h
 Dependencies: 	Always: GenericTypeDefs.h, usb_device.h
               	Situational: usb_function_hid.h, usb_function_cdc.h, usb_function_msd.h, etc.
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
 *******************************************************************/

/*********************************************************************
 * Descriptor specific type definitions are defined in: usbd.h
 ********************************************************************/

#ifndef USBCFG_H
#define USBCFG_H

/** INCLUDES *******************************************************/

/** DEFINITIONS ****************************************************/
#define EP0_BUFF_SIZE		8	// Valid Options: 8, 16, 32, or 64 bytes.
								// Using larger options take more SRAM, but
								// does not provide much advantage in most types
								// of applications.  Exceptions to this, are applications
								// that use EP0 IN or OUT for sending large amounts of
								// application related data.
									
#define MAX_NUM_INT             1   // For tracking Alternate Setting
#define USB_MAX_EP_NUMBER	2


//the maximum transfer size of EP1
#define USB_MAX_DATA_SIZE_EP1 3
#define USB_NUM_MESSAGES_EP1 1

#define USB_PING_PONG__NO_PING_PONG         0x00    //0b00
#define USB_PING_PONG__EP0_OUT_ONLY         0x01    //0b01
#define USB_PING_PONG__FULL_PING_PONG       0x02    //0b10
#define USB_PING_PONG__ALL_BUT_EP0          0x03    //0b11

//#define USER_DEVICE_DESCRIPTOR device_dsc
//#define USER_CONFIG_DESCRIPTOR USB_CD_Ptr

//Make sure only one of the below "#define USB_PING_PONG_MODE"
//is uncommented.
//#define USB_PING_PONG_MODE USB_PING_PONG__NO_PING_PONG
#define USB_PING_PONG_MODE USB_PING_PONG__FULL_PING_PONG
//#define USB_PING_PONG_MODE USB_PING_PONG__EP0_OUT_ONLY
//#define USB_PING_PONG_MODE USB_PING_PONG__ALL_BUT_EP0		//NOTE: This mode is not supported in PIC18F4550 family rev A3 devices


#define USB_POLLING

/* Parameter definitions are defined in usb_device.h */
#define UCFG_VAL                _PUEN|_TRINT|_FS|USB_PING_PONG_MODE


/** DEVICE CLASS USAGE *********************************************/
#define USB_SUPPORT_DEVICE
#define USB_USE_GEN

/** ENDPOINTS ALLOCATION *******************************************/

/* Generic */
#define USBGEN_EP_SIZE          64
#define USBGEN_EP_NUM            1

/** DEFINITIONS ****************************************************/

#endif //USBCFG_H
