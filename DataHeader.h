/*
 * DataPacketHeader.h
 *
 *  Created on: Apr 28, 2016
 *      Author: NagendraBabu
 */

#ifndef _DATAHEADER_H_
#define _DATAHEADER_H_

#define DataPort 0x9C41

//*********************** Node Id *************************************//
// Your Node ID to be provided in each packet to identify you
// If your node has capability of sending different kinds of data in the N/W
// The kind of differs in messageType, but NodeId will be same if data coming from same Node.
//*********************************************************************//

#define	n_behavior 				0x01	// Behaviour Node
#define	n_batteryCharger 		0x0A	// Battery charger Node
#define	n_thermostat 			0x14	// Thermostat node
#define	n_weatherSensors 		0x1E	// Weather sensors
#define n_weatherSensorsDisplay	0x23	// Displaying weather sensors Data
#define	n_webMessage 			0x28	// Webmessage Node
#define	n_jpeg 					0x32	// Jpeg Node
#define	n_dataSniffer 			0x64	// DataSniffer Node
#define	n_stormAlert 			0x6E	// Storm alerts
#define	n_newsFeed 				0x78	// NewsFeed
#define n_newsFeedDisplay		0x79	// NewsFeed Display
#define	n_timeServices 			0x82	// Timeservices Node
#define	n_weatherServices 		0x8C	// Weather Service Node
#define	n_electricityCost 		0x96	// Electricity Cost Node


//******************* Message Type fields ***************************//
// Message type field to be filled in Data packet
// This field represents which kind of data you are transmitting
//******************************************************************//

#define	newsFeed 		  0x01	// Newsfeed service
#define	temp			  0x02	// temperature from thermostat
#define humidity		  0x03	// Humidty for thermostat node
#define	stormAlert 	 	  0x04	// Strom alert Message
#define	aPressure 	 	  0x05	// Atmosphere pressure from sensors
#define bPressure	 	  0x06	// Barometric Pressure
#define	batteryStatus 	  0x07	// Status of the battery
#define	electricityCost	  0x08	// Electricity cost based on Time
#define	timeDateService	  0x09	// Time and provided by time services team in utc format
#define	sunsetSunriseData 0x0A	// Sunsetand sunrise data from services Team
#define	holidaysInfo	  0x0B	// Holiday information from services Team
#define dewPoint	 	  0x0C	// Dew Point
#define visibility	 	  0x0D	// Visibility
#define serviceArea		  0x0E	// Area
#define zipCode			  0x0F	// Zipcode service.
#define	webMessage		  0x10	// WebMessage delivery
#define	jpegData		  0x11	// Jpeg Image Data
#define wSpeed		 	  0x12	// Wind speed
#define wDirection	 	  0x13	// Wind direction

//**************** Structures to be appended in the data Frame**************//

// Data header info to be added in at start of data field in UDP packet
// This structure gives us idea about Node from which packet is coming and
// how many messages you are sending.
struct _dataInfo
{
	uint8_t validationCode;
	uint8_t nodeId;
	uint8_t noOfMsgs;
	uint8_t data;
} *dataInfo;

// UDP Data frame for each data (Sensor Data/Message) in a Single data packet
struct _data
{
	uint8_t messagetype;
	uint8_t data;
} *udpDataFrame[5];				// If you are planning to send more kinds of data frames in single packet you can increase Size

#endif /* DATAHEADER_H_ */
