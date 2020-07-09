#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "./USB/usb_device.h"
#include "emu_ft245_eeprom.h"

BYTE eeprom[128];

void eeprom_init(void){
	static BYTE i;
	static WORD checksum; 
	static BYTE idx;
	eeprom[0]=0;
	eeprom[1]=0;
	eeprom[2]=((ROM BYTE*)&device_dsc.idVendor)[0];	//vid,
	eeprom[3]=((ROM BYTE*)&device_dsc.idVendor)[1];
	eeprom[4]=((ROM BYTE*)&device_dsc.idProduct)[0];	//pid,
	eeprom[5]=((ROM BYTE*)&device_dsc.idProduct)[1];
	eeprom[6]=((ROM BYTE*)&device_dsc.bcdDevice)[0];	//ver
	eeprom[7]=((ROM BYTE*)&device_dsc.bcdDevice)[1];
	eeprom[8]=configDescriptor1[7];	//attr,
	eeprom[9]=configDescriptor1[8];	//pow
	eeprom[10]=0x1C;
	eeprom[11]=0x00;
	eeprom[12]=((ROM BYTE*)&device_dsc.bcdUSB)[0];
	eeprom[13]=((ROM BYTE*)&device_dsc.bcdUSB)[1];
	eeprom[14]=0x86+14;								//0x80+&sd001
	eeprom[15]=USB_SD_Ptr[1][0];						//sizeof(sd001)
	eeprom[16]=0x86+14+USB_SD_Ptr[1][0];					//0x80+&sd002
	eeprom[17]=USB_SD_Ptr[2][0];						//sizeof(sd002)
	eeprom[18]=0x86+14+USB_SD_Ptr[1][0]+USB_SD_Ptr[2][0];	//0x80+&sd003
	eeprom[19]=USB_SD_Ptr[3][0];						//sizeof(sd003)
	idx=20;
	for(i=0;i<USB_SD_Ptr[1][0];i++) eeprom[idx++]=USB_SD_Ptr[1][i];
	for(i=0;i<USB_SD_Ptr[2][0];i++) eeprom[idx++]=USB_SD_Ptr[2][i];
	for(i=0;i<USB_SD_Ptr[3][0];i++) eeprom[idx++]=USB_SD_Ptr[3][i];
	for(i=0;i<4;i++) eeprom[idx++]=((ROM BYTE*)&device_dsc.iManufacturer)[i];
	while(idx<126) eeprom[idx++]=0;
	checksum=0xAAAA;
	for(i=0;i<63;i++){
		checksum^=((WORD*)eeprom)[i];
		if(checksum&0x8000) checksum=(checksum<<1)|1;
		else checksum=(checksum<<1);
	}
	((WORD*)eeprom)[63]=checksum;
}