/*
 * Sniffer.h
 *
 *  Created on: May 1, 2016
 *      Author: NagendraBabu
 */

#ifndef _SNIFFER_H_
#define _SNIFFER_H_
// Node packet sniffing functions
void processNewsFeedPacket(uint8_t *dataIn);
void processElectrictyCostPacket(uint8_t *dataIn);
void processWeatherSensorsPacket(uint8_t *dataIn);
void processThermostatPacket(uint8_t *dataIn);
void ProcessBatteryChargerPacket(uint8_t *dataIn);
void processWeatherServicesPacket(uint8_t *dataIn);
void processJpegPacket(uint8_t *dataIn);
void processTimeServicesPacket(uint8_t *dataIn);
void processStromAlertPacket(uint8_t *dataIn);
// print helper functions


void printDateTime(void *ptr,bool isUtc);
uint32_t createUint32_t(uint8_t *add);
uint16_t createUint16_t(uint8_t *add);
void printUint16(void *ptr,int iResolution);
void printInt16(void *ptr,int iResolution);
void printElectricityCost(void *ptr);
void printWindDirection(void *ptr);
void printBatteryInfo(void *ptr);



#endif /* SNIFFER_H_ */
