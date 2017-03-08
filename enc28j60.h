// ENC28J60 Driver
// Author: NagendraBabu

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

#ifndef _ENC28J60_H_
#define _ENC28J60_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Buffer is configured as follows
// Receive buffer starts at 0x0000 (bottom 6666 bytes of 8K space)
// Transmit buffer at 01A0A (top 1526 bytes of 8K space)

// ------------------------------------------------------------------------------
//  Defines                
// ------------------------------------------------------------------------------

// Pins
#define PIN_ETHER_CS (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))

#define ETHER_UNICAST        0x80
#define ETHER_BROADCAST      0x01
#define ETHER_MULTICAST      0x02
#define ETHER_HASHTABLE      0x04
#define ETHER_MAGICPACKET    0x08
#define ETHER_PATTERNMATCH   0x10

#define ETHER_HALFDUPLEX     0x00
#define ETHER_FULLDUPLEX     0x40

// Ether registers
#define ERDPTL		0x00
#define ERDPTH		0x01
#define EWRPTL		0x02
#define EWRPTH		0x03
#define ETXSTL		0x04
#define ETXSTH		0x05
#define ETXNDL		0x06
#define ETXNDH		0x07
#define ERXSTL		0x08
#define ERXSTH		0x09
#define ERXNDL		0x0A
#define ERXNDH		0x0B
#define ERXRDPTL	0x0C
#define ERXRDPTH	0x0D
#define ERXWRPTL	0x0E
#define ERXWRPTH	0x0F
#define EIE		    0x1B
#define EIR		    0x1C
#define RXERIF  0x01
#define TXERIF  0x02
#define TXIF    0x08
#define PKTIF   0x40
#define ESTAT       0x1D
#define CLKRDY  0x01
#define TXABORT 0x02
#define ECON2		0x1E
#define PKTDEC  0x40
#define ECON1       0x1F
#define RXEN    0x04
#define TXRTS   0x08
#define ERXFCON		0x38
#define EPKTCNT     0x39
#define MACON1		0x40
#define MARXEN  0x01
#define RXPAUS  0x04
#define TXPAUS  0x08
#define MACON2		0x41
#define MARST   0x80
#define MACON3		0x42
#define FULDPX  0x01
#define FRMLNEN 0x02
#define TXCRCEN 0x10
#define PAD60   0x20
#define MACON4		0x43
#define MABBIPG		0x44
#define MAIPGL		0x46
#define MAIPGH		0x47
#define MACLCON1	0x48
#define MACLCON2	0x49
#define MAMXFLL		0x4A
#define MAMXFLH		0x4B
#define MICMD       0x52
#define MIIRD   0x01
#define MIREGADR    0x54
#define MIWRL       0x56
#define MIWRH       0x57
#define MIRDL       0x58
#define MIRDH       0x59
#define MAADR1      0x60
#define MAADR0      0x61
#define MAADR3      0x62
#define MAADR2      0x63
#define MAADR5      0x64
#define MAADR4      0x65
#define MISTAT      0x6A
#define MIBUSY  0x01
#define ECOCON      0x75

// Ether phy registers
#define PHCON1       0x00
#define PDPXMD 0x0100
#define PHCON2       0x10
#define HDLDIS 0x0100
#define PHLCON       0x14

// ------------------------------------------------------------------------------
//  Macros                
// ------------------------------------------------------------------------------

#define LOBYTE(x) ((x) & 0xFF)
#define HIBYTE(x) (((x) >> 8) & 0xFF)

// ------------------------------------------------------------------------------
//  Functions                
// ------------------------------------------------------------------------------

void etherInit(uint8_t mode);
void etherWritePhy(uint8_t reg, uint16_t data);
uint16_t etherReadPhy(uint8_t reg);
uint8_t etherKbhit();
uint16_t etherGetPacket(uint8_t data[], uint16_t max_size);
uint8_t etherIsOverflow();
bool etherPutPacket(uint8_t data[], uint16_t size);

uint8_t etherIsIp(uint8_t data[]);
bool etherIsIpUnicast(uint8_t data[]);
bool etherIsValidIp();
void etherSetIpAddress(uint8_t a, uint8_t b,  uint8_t c, uint8_t d);

uint8_t etherIsPingReq(uint8_t data[]);
void etherSendPingResp(uint8_t data[]);

#define ARP_INVALID 0
#define ARP_REQUEST 1
#define ARP_RESPONSE 2
uint8_t etherIsArp(uint8_t data[]);
void etherSendArpResp(uint8_t data[]);
void etherSendArpReq(uint8_t data[], uint8_t ip[]);

uint8_t etherIsUdp(uint8_t data[]);
uint8_t* etherGetUdpData(uint8_t data[]);
void etherSendUdpData(uint8_t data[], uint8_t* udp_data, uint8_t udp_size);
//void etherSendNewUdpPacket(uint8_t data[], uint16_t destport,uint8_t *destIp,uint8_t *destMac,uint8_t noOfMsgs,uint16_t *temp,uint16_t *humidity);
void etherInitStructures(uint8_t data[]);
void analyzeUdpPacket(uint8_t data[]);
uint16_t htons(uint16_t value);
uint8_t etherIsDiscReq(uint8_t data[]);
void etherSendDiscResponse(uint8_t data[]);
#define ntohs htons

#endif
