/*
 * Sniffer.c
 *
 *  Created on: May 1, 2016
 *      Author: NagendraBabu
 */
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "Sniffer.h"
#include "UART.h"
#include "DataHeader.h"
#include "enc28j60.h"

void processThermostatPacket(uint8_t *dataIn)
{
	dataInfo = (void *)dataIn;
	uint8_t msgNo = dataInfo->noOfMsgs;
	if(msgNo == 0)
	{
		putsUart0("\t Zero messages in paket.\r\n");
		return;
	}
	struct _data *currData;
	currData = (void *)&dataInfo->data;
	while(msgNo != 0)
	{
		switch(currData->messagetype)
		{
			case  temp:
				putsUart0("\t Temperature :");
				printInt16(&currData->data,10);
				currData = (void*)(&currData->data +2);
				putcUart0('C');
				break;
			case humidity:
				putsUart0("\t Humidity :");
				printUint16(&currData->data,10);
				putcUart0('%');
				currData = (void*)(&currData->data +2);
				break;
			default:
				putsUart0("\t Invalid Data.");
		}
		msgNo--;
		putsUart0("\r\n");
	}
}
void processWeatherSensorsPacket(uint8_t *dataIn)
{
	dataInfo = (void *)dataIn;
	uint8_t msgNo = dataInfo->noOfMsgs;
	if(msgNo == 0)
	{
		putsUart0("\t Zero messages in paket.\r\n");
		return;
	}
	struct _data *currData;
	currData = (void *)&dataInfo->data;
	while(msgNo != 0)
	{
		switch(currData->messagetype)
		{
			case  temp:
				putsUart0("\t Temperature :");
				printInt16(&currData->data,10);
				putcUart0('C');
				currData = (void*)(&currData->data +2);
				break;
			case humidity:
				putsUart0("\t Humidity :");
				printUint16(&currData->data,10);
				putcUart0('%');
				currData = (void*)(&currData->data +2);
				break;
			case aPressure:
				putsUart0("\t Atmospheric Pressure :");
				printUint16(&currData->data,10);
				putsUart0("kPa");
				currData = (void*)(&currData->data +2);
				break;
			default:
				putsUart0("\t Invalid Data.");
		}
		msgNo--;
		putsUart0("\r\n");
	}
}
void processWeatherServicesPacket(uint8_t *dataIn)
{
	dataInfo = (void *)dataIn;
	uint8_t msgNo = dataInfo->noOfMsgs;
	if(msgNo == 0)
	{
		putsUart0("\t Zero messages in packet.\r\n");
		return;
	}
	int i;
	struct _data *currData;
	currData = (void *)&dataInfo->data;
	while(msgNo != 0)
	{
		switch(currData->messagetype)
		{
			case  temp:
				putsUart0("\t Temperature :");
				printInt16(&currData->data,10);
				putcUart0('C');
				currData = (void*)(&currData->data +2);
				break;
			case humidity:
				putsUart0("\t Humidity Data :");
				printUint16(&currData->data,10);
				putcUart0('%');
				currData = (void*)(&currData->data +2);
				break;
			case bPressure:
				putsUart0("\t Barometric Pressure :");
				printUint16(&currData->data,10);
				putsUart0("kPa");
				currData = (void*)(&currData->data +2);
				break;
			case wSpeed:
				putsUart0("\t Wind speed :");
				printUint16(&currData->data,10);
				putsUart0("miles/hr");
				currData = (void*)(&currData->data +2);
				break;
			case dewPoint:
				putsUart0("\t Dew point :");
				printInt16(&currData->data,10);
				putcUart0('C');
				currData = (void*)(&currData->data +2);
				break;
			case visibility:
				putsUart0("\t Visibility :");
				printUint16(&currData->data,10);
				putsUart0("miles");
				currData = (void*)(&currData->data +2);
				break;
			case wDirection:
				putsUart0("\t Wind Direction :");
				printWindDirection(&currData->data);
				currData = (void*)(&currData->data +1);
				break;
			case serviceArea:
				putsUart0("\t Service Area :");
				i=createUint16_t(&currData->data);
				putsnUart0(&currData->data+2,i);
				putsUart0("\r\n");
				currData = (void*)(&currData->data +3+i);
				break;
			default:
				putsUart0("\t Invalid Data.");
		}
		msgNo--;
		putsUart0("\r\n");
	}
}
void processNewsFeedPacket(uint8_t *dataIn)
{
	dataInfo = (void *)dataIn;
	uint8_t msgNo = dataInfo->noOfMsgs;
	if(msgNo == 0)
	{
		putsUart0("\t Zero messages in paket.\r\n");
		return;
	}
	struct _data *currData;
	currData = (void *)&dataInfo->data;
	int i=0;
	while(msgNo != 0)
	{
		switch(currData->messagetype)
		{
			case  newsFeed:
				putsUart0("\t News :");
				i=createUint16_t(&currData->data);
				putsnUart0(&currData->data +2,i);
				currData = (void*)(&currData->data +3+i);
				break;
			default:
				putsUart0("\t Invalid data.");
		}
		msgNo--;
		putsUart0("\r\n");
	}
}
void processStromAlertPacket(uint8_t *dataIn)
{
	dataInfo = (void *)dataIn;
	uint8_t msgNo = dataInfo->noOfMsgs;
	if(msgNo == 0)
	{
		putsUart0("\t Zero messages in paket.\r\n");
		return;
	}
	struct _data *currData;
	currData = (void *)&dataInfo->data;
	int i=0;
	while(msgNo != 0)
	{
		switch(currData->messagetype)
		{
			case  stormAlert:
				putsUart0("\t Strom Alert :");
				i=createUint16_t(&currData->data);
				putsnUart0(&currData->data +2,i);
				currData = (void*)(&currData->data +3+i);
				break;
			default:
				putsUart0("\t Invalid data.");
		}
		msgNo--;
		putsUart0("\r\n");
	}
}
void processTimeServicesPacket(uint8_t *dataIn)
{
	dataInfo = (void *)dataIn;
	uint8_t msgNo = dataInfo->noOfMsgs;
	if(msgNo == 0)
	{
		putsUart0("\t Zero messages in paket.\r\n");
		return;
	}
	struct _data *currData;
	currData = (void *)&dataInfo->data;
	while(msgNo != 0)
	{
		switch(currData->messagetype)
		{
			case  timeDateService:
				putsUart0("\t Date & Time :");
				printDateTime(&currData->data,true);
				currData = (void*)(&currData->data +9);
				break;
			case sunsetSunriseData:
				putsUart0("\t Sunrise :");
				printDateTime(&currData->data,false);
				currData = (void*)(&currData->data +8);
				putsUart0("\t Sunset :");
				printDateTime(&currData->data,false);
				currData = (void*)(&currData->data +9);
				break;
			default:
				putsUart0("\t Invalid data.");
		}
		msgNo--;
		putsUart0("\r\n");
	}
}
void processElectrictyCostPacket(uint8_t *dataIn)
{
	dataInfo = (void *)dataIn;
	uint8_t msgNo = dataInfo->noOfMsgs;
	if(msgNo == 0)
	{
		putsUart0("\t Zero messages in paket.\r\n");
		return;
	}
	struct _data *currData;
	currData = (void *)&dataInfo->data;
	while(msgNo != 0)
	{
		switch(currData->messagetype)
		{
			case  electricityCost:
				putsUart0("\t Electricity Cost on ");
				printElectricityCost(&currData->data);
				currData = (void*)(&currData->data +57);
				break;
			default:
				putsUart0("\t Invalid data.");
		}
		msgNo--;
		putsUart0("\r\n");
	}
}

void ProcessBatteryChargerPacket(uint8_t *dataIn)
{
	dataInfo = (void *)dataIn;
	uint8_t msgNo = dataInfo->noOfMsgs;
	if(msgNo == 0)
	{
		putsUart0("\t Zero messages in paket.\r\n");
		return;
	}
	struct _data *currData;
	currData = (void *)&dataInfo->data;
	while(msgNo != 0)
	{
		switch(currData->messagetype)
		{
			case  batteryStatus:
				putsUart0("\t Batter Status Info :");
				printBatteryInfo(&currData->data);
				currData = (void*)(&currData->data +25);
				break;
			default:
				putsUart0("\t Invalid data.");
		}
		msgNo--;
		putsUart0("\r\n");
	}

}


//* ************* Helper Functions ********************//
//****************************************************//
void printDateTime(void *ptr,bool isUtc)
{
	uint8_t *dataVal = (void *)ptr;
	char data[5]={'\0'};
	uint16_t year;
	uint8_t month,day,hour,minutes,seconds,utcMinutes;
	int16_t utc,utcHours;
	year = createUint16_t(dataVal);
	dataVal+=2;
	month = (*dataVal++ & 0xFF);
	day = (*dataVal++ & 0xFF);
	hour = (*dataVal++ & 0xFF);
	minutes = (*dataVal++ & 0xFF);
	seconds = (*dataVal++ & 0xFF);
	utc = createUint16_t(dataVal);
	utcHours = utc/60;
	utcMinutes = abs(utc) % 60;
	intTostring(year,data);
	putsUart0(data);putcUart0('-');memset(data,'\0',5);
	intTostring(month,data);
	putsUart0(data);putcUart0('-');memset(data,'\0',5);
	intTostring(day,data);
	putsUart0(data);putcUart0(' ');memset(data,'\0',5);
	intTostring(hour,data);
	putsUart0(data);putcUart0(':');memset(data,'\0',5);
	intTostring(minutes,data);
	putsUart0(data);putcUart0(':');memset(data,'\0',5);
	intTostring(seconds,data);
	putsUart0(data);putcUart0(' ');memset(data,'\0',5);
	intTostring(utcHours,data);
	if(isUtc)
	{
		putsUart0(" UTC ");
		if(utcHours > 0)
			putcUart0('+');
		putsUart0(data);putcUart0(':');memset(data,'\0',5);
		intTostring(utcMinutes,data);
		putsUart0(data);memset(data,'\0',5); 					// TO-DO : UTC offset
	}
}
uint32_t createUint32_t(uint8_t *add)
{
	int i=0;
	uint32_t result=0;;
	for(i=0;i<4;i++)
	{
		result+=(add[i] << (3-i)*8);
	}
	return result;
}
uint16_t createUint16_t(uint8_t *add)
{
	int i=0;
	uint16_t result=0;
	for(i=0;i<2;i++)
	{
		result+=(add[i] << (1-i)*8);
	}
	return result;
}
void printUint16(void *ptr,int iResolution)
{
	uint8_t *dataVal = (void*)ptr;
	char data[6]={'\0'};
	uint16_t inData = ((uint16_t)(*dataVal))<<8 | (uint16_t)((*(dataVal+1))& 0xFF);
	uint16_t data1 = (uint16_t)(inData/iResolution);
	uint16_t data2 = (uint16_t)(inData%iResolution);
	intTostring(data1,data);
	putsUart0(data);
	putcUart0('.');
	memset(data,'\0',6);
	intTostring(data2,data);
	putsUart0(data);
}
void printInt16(void *ptr,int iResolution)
{
	int8_t *dataVal = (void*)ptr;
	char data[6]={'\0'};
	int inData = ((int16_t)(*dataVal))<<8 | (int16_t)((*(dataVal+1))& 0xFF);
	int data1 = (int)(inData/iResolution);
	int data2 = abs(inData);
	data2 = data2 % 10;
	intTostring(data1,data);
	putsUart0(data);
	putcUart0('.');
	memset(data,'\0',6);
	intTostring(data2,data);
	putsUart0(data);
}

void printBatteryInfo(void *ptr)
{
	uint8_t *dataVal = (void*)ptr;
	char res[10]={'\0'};
	uint32_t uCharger_mA = 0x00000000;
	uint32_t uCharger_mV = 0x00000000;
	uint32_t uMax_mA =  0x00000000;
	uint32_t uMax_mV = 0x00000000;
	uint8_t  uChargerStatus = 0x00;
	uint32_t uTimeToFinish = 0x00000000;
	uint32_t uChargePercentage = 0x00000000;
	uint32_t data1,data2;
	//************************* Data Extraction*********************************//
	uCharger_mA = createUint32_t(dataVal);
	dataVal = dataVal+4;
	uCharger_mV = createUint32_t(dataVal);
	dataVal = dataVal+4;
	uMax_mA = createUint32_t(dataVal);
	dataVal = dataVal+4;
	uMax_mV = createUint32_t(dataVal);
	dataVal = dataVal+4;
	uChargerStatus = (*dataVal)&0XFF;
	dataVal = dataVal+1;
	uTimeToFinish = createUint32_t(dataVal);
	dataVal = dataVal+4;
	uChargePercentage = createUint32_t(dataVal);
	dataVal = dataVal+4;
	//********************************** Printing ******************************************//
	intTostring(uCharger_mA,res);
	putsUart0("\r\n\t\t Charger Amperage :");putsUart0(res);putsUart0(" mA");memset(res,strlen(res),'\0');
	intTostring(uCharger_mV,res);
	putsUart0("\r\n\t\t Charger Voltage :");putsUart0(res);putsUart0("mV");memset(res,strlen(res),'\0');
	intTostring(uMax_mA,res);
	putsUart0("\r\n\t\t Max. Amperage :");putsUart0(res);putsUart0("mA");memset(res,strlen(res),'\0');
	intTostring(uMax_mV,res);
	putsUart0("\r\n\t\t Max. Volatge :");putsUart0(res);putsUart0("mV");memset(res,strlen(res),'\0');
	intTostring(uTimeToFinish,res);
	putsUart0("\r\n\t\t Time To Finish full charge :");putsUart0(res);putsUart0("Seconds");memset(res,strlen(res),'\0');
	data1 = (uint32_t)uChargePercentage/1000;
	data2 = (uint32_t)uChargePercentage/1000;
	intTostring(data1,res);
	putsUart0("\r\n\t\t Percentage charge Done :");putsUart0(res);putcUart0('.');memset(res,strlen(res),'\0');
	intTostring(data2,res);
	putsUart0(res);putcUart0('%');memset(res,strlen(res),'\0');
	intTostring(uChargerStatus,res);
	putsUart0("\r\n\t\t Battery Status :");putsUart0(res);
}
void printWindDirection(void *ptr)
{
	uint8_t *dataVal = (void*)ptr;
	switch(*dataVal)
	{
		case 1:
			putsUart0("East \r\n");
			break;
		case 2:
			putsUart0("West \r\n");
			break;
		case 3:
			putsUart0("North \r\n");
			break;
		case 4:
			putsUart0("South \r\n");
			break;
		case 5:
			putsUart0("North-East \r\n");
			break;
		case 6:
			putsUart0("South-East \r\n");
			break;
		case 7:
			putsUart0("South-West \r\n");
			break;
		case 8:
			putsUart0("North-West \r\n");
			break;
		default:
			putsUart0("Invalid Direction.\r\n");
	}
}

void printElectricityCost(void *ptr)
{
	uint8_t *dataVal = (void *)ptr;
	char data[5]={'\0'};
	uint16_t year;
	uint8_t month,day,utcMinutes;
	uint8_t i=0;
	int16_t utc,utcHours;
	year = createUint16_t(dataVal);
	dataVal+=2;
	month = (*dataVal++ & 0xFF);
	day = (*dataVal++ & 0xFF);
	utc = createUint16_t(dataVal);
	dataVal+=2;
	utcHours = utc/60;
	utcMinutes = abs(utc) % 60;
	intTostring(year,data);
	putsUart0(data);putcUart0('-');memset(data,'\0',5);
	intTostring(month,data);
	putsUart0(data);putcUart0('-');memset(data,'\0',5);
	intTostring(day,data);
	putsUart0(data);putcUart0(' ');memset(data,'\0',5);
	putsUart0(" UTC ");
	if(utcHours > 0)
		putcUart0('+');
	intTostring(utcHours,data);
	putsUart0(data);putcUart0(':');memset(data,'\0',5);
	intTostring(utcMinutes,data);
	putsUart0(data);putsUart0(" (in Cents)"); memset(data,'\0',5);
	while(i!=24)
	{
		putsUart0("\r\n\t\t");
		printUint16(dataVal,100);
		dataVal+=2;
		i=i+1;
		/*
		intTostring(i&0xff,output);putsUart0(output);
		putcUart0('-');
		intTostring(i+1&0xff,output);putsUart0(output);
		putcUart0(' ');
		putcUart0('¢');
		putsUart0("\r\n\t\t")*/

	}
}

