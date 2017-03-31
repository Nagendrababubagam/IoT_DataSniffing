//-------------------------------------------------------------------------------
//  Control Node
//
//  Simple Breakdown of what is needed for the discovery transaction
//
//  More information will be added to this header very soon
//
//	Author:  Jacob Longwell
//
//	Change History:  Created 4May2016 JL; Focus is on Discovery Request and Response
//
//-------------------------------------------------------------------------------
#include <stdint.h>
//-------------------------------------------------------------------------------
#define VALIDATIONCODE 0x95

//deviceType 		for now, this will be the last octet of your ip Address

//nodeId 			this will be the last octet of your ip Address

//ctrlType			this is the ctrlType packet identifier
#define DISC 0x01

//opCode
#define CTRL_REQUEST 	0x01	//  the control node will send a 0x01
#define CTRL_RESPONSE	0x02	//  the nodes and services teams will send a 0x02

//noOfCtrlMessages 	//  stating the obvious: this will be the number of ctrl messages.
					//  For example 1,2,3,...,255
	
struct _ctrlInfo
{
	uint8_t validationCode;
	uint8_t deviceType;
	uint8_t nodeId;
	uint8_t ctrlType;
	uint8_t opCode;
	uint8_t noOfCtrlMessages;
	uint8_t data;
} *ctrlInfo;
//-------------------------------------------------------------------------------
     	
//localFunctionID 		//  The control node will need an identifier that identifies
						//  what your device produces or consumes.
						//  Start with 1 and increment from there.
						//  This is how the rules will be able to dictate where
						//  your services will be delivered

//functionType
#define NAME 		0x01//  This is for the friendly name of your device.
						//  This MUST be in your DISC response message
#define DATA 		0x02//	This tells the control node what service or information
						//  you can provide to the network.
#define ACTION 		0x03//  This tells the control node what action is associated with your device.
						//  Every device may produce or consume data
						
//dataType
#define	t_string 	0x01
#define	t_int	 	0x02
#define t_float		0x03

#define t_time		0x04


//lengthOfFriendlyString 	//  length in bytes of the friendly string name, no null termination

struct _ctrlData
{
	uint8_t localFunctionID;
	uint8_t functionType;
	uint8_t dataType;
	uint8_t lengthOfFriendlyString;
	uint8_t stringData;
} *ctrlData;
//-------------------------------------------------------------------------------
