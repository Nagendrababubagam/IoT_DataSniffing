// RTOS Framework
//  Author: NagendraBabu
// E-mail: nagendrababu.bagam@mavs.uta.edu

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 4 Pushbuttons and 4 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "UART.h"
#include "enc28j60.h"
#include "DataHeader.h"
#include "iotctrl.h"

// REQUIRED: correct these bitbanding references for green and yellow LEDs (temporary to guarantee compilation)
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))


//****************************************************************************
// RTOS Defines and Kernel Variables
//****************************************************************************
// function pointer
typedef void (*_fn)();
char tolower(char c);
void update_SP(void* sp);
void putsUart0(char* str);
int atoi(const char *str);
void intTostring(int value,char *str);

// Task states
#define STATE_INVALID    0 // no task
#define STATE_READY      1 // ready to run
#define STATE_BLOCKED    2 // has run, but now blocked by semaphore
#define STATE_DELAYED    3 // has run, but now awaiting timer


#define MAX_TASKS 5       	// maximum number of valid tasks
#define MAX_PACKETS 10
uint8_t taskCount = 0;      // total number of valid tasks
uint32_t stack[MAX_TASKS][256];
uint8_t taskCurrent = 0xFF;// index of last dispatched task
uint8_t nextPacketLsb = 0x00;
uint8_t nextPacketMsb = 0x00;
uint8_t sequenceId = 1;
uint32_t sum;
uint8_t macAddress[6] = {0x63,0x14,0xAB,0xCD,0x10,0xA0};
uint8_t ipv4Address[4];
uint8_t broadCastIp[4]={192,168,137,100};
uint16_t sourcePort = 0x4005;
uint16_t ctrlDest = 0x9C40;
uint8_t data[256];
uint8_t UDPPacketCount = 0;
uint8_t packets[MAX_PACKETS][256];
uint8_t BWriteIndex=0,BReadIndex=0;
//******************************************* Ethernet(enc28J60) structures ************************************//
//  Structures
// ------------------------------------------------------------------------------

// This M4F is little endian
// Network byte order is big endian
// Must uinterpret uint16_ts in reverse order

struct enc28j60Frame // 4-bytes
{
  uint16_t size;
  uint16_t status;
  uint8_t data;
} *enc28j60;

struct etherFrame // 14-bytes
{
  uint8_t destAddress[6];
  uint8_t sourceAddress[6];
  uint16_t frameType;
  uint8_t data;
} *ether;

struct _ip // minimum 20 bytes
{
  uint8_t rev_size;
  uint8_t typeOfService;
  uint16_t length;
  uint16_t id;
  uint16_t flagsAndOffset;
  uint8_t ttl;
  uint8_t protocol;
  uint16_t headerChecksum;
  uint8_t sourceIp[4];
  uint8_t destIp[4];
} *ip;

struct _icmp
{
  uint8_t type;
  uint8_t code;
  uint16_t check;
  uint16_t id;
  uint16_t seq_no;
  uint8_t data;
} *icmp;

struct _arp
{
  uint16_t hardwareType;
  uint16_t protocolType;
  uint8_t hardwareSize;
  uint8_t protocolSize;
  uint16_t op;
  uint8_t sourceAddress[6];
  uint8_t sourceIp[4];
  uint8_t destAddress[6];
  uint8_t destIp[4];
} *arp;

struct _udp // 8 bytes
{
  uint16_t sourcePort;
  uint16_t destPort;
  uint16_t length;
  uint16_t check;
  uint8_t  data;
} *udp;

struct _tcp
{
	uint16_t sourcePort;
	uint16_t destPort;
	uint32_t seqNo;
	uint32_t ackNo;
	uint8_t headerLen; // dec eq=Hex_eq_dec_value/4;
	uint8_t	flags;
	uint16_t windSize;
	uint16_t checkSum;
	uint16_t urgentPointer;
	uint8_t data;
} *tcp;

//******************************************* Ethernet(enc28J60) functions ************************************//
//*************************************************************************************************************//
void spiWrite(uint8_t data)
{
	SSI2_DR_R = data;
	while (SSI2_SR_R & SSI_SR_BSY);
}

uint8_t spiRead()
{
    return SSI2_DR_R;
}

void etherCsOn()
{
    PIN_ETHER_CS = 0;
	__asm (" NOP");                    // allow line to settle
	__asm (" NOP");
	__asm (" NOP");
	__asm (" NOP");
}

void etherCsOff()
{
    PIN_ETHER_CS = 1;
}

void etherWriteReg(uint8_t reg, uint8_t data)
{
    etherCsOn();
    spiWrite(0x40 | (reg & 0x1F));
    spiRead();
    spiWrite(data);
    spiRead();
    etherCsOff();
}

uint8_t etherReadReg(uint8_t reg)
{
    uint8_t data;
    etherCsOn();
    spiWrite(0x00 | (reg & 0x1F));
    spiRead();
    spiWrite(0);
    data = spiRead();
    etherCsOff();
    return data;
}

void etherSetReg(uint8_t reg, uint8_t mask)
{
    etherCsOn();
    spiWrite(0x80 | (reg & 0x1F));
    spiRead();
    spiWrite(mask);
    spiRead();
    etherCsOff();
}

void etherClearReg(uint8_t reg, uint8_t mask)
{
    etherCsOn();
    spiWrite(0xA0 | (reg & 0x1F));
    spiRead();
    spiWrite(mask);
    spiRead();
    etherCsOff();
}

void etherSetBank(uint8_t reg)
{
    etherClearReg(ECON1, 0x03);
    etherSetReg(ECON1, reg >> 5);
}

void etherWritePhy(uint8_t reg, uint16_t data)
{
    etherSetBank(MIREGADR);
    etherWriteReg(MIREGADR, reg);
    etherWriteReg(MIWRL, data & 0xFF);
    etherWriteReg(MIWRH, (data >> 8) & 0xFF);
}

uint16_t etherReadPhy(uint8_t reg)
{
    uint16_t data, data2;
    etherSetBank(MIREGADR);
    etherWriteReg(MIREGADR, reg);
    etherWriteReg(MICMD, etherReadReg(MICMD) | MIIRD);
    waitMicrosecond(50);
    while ((etherReadReg(MISTAT) | MIBUSY) != 0);
    etherWriteReg(MICMD, etherReadReg(MICMD) & ~MIIRD);
    data = etherReadReg(MIRDL);
    data2 = etherReadReg(MIRDH);
    data |= (data2 << 8);
    return data;
}

void etherWriteMemStart()
{
    etherCsOn();
    spiWrite(0x7A);
    spiRead();
}

void etherWriteMem(uint8_t data)
{
    spiWrite(data);
    spiRead();
}

void etherWriteMemStop()
{
    etherCsOff();
}

void etherReadMemStart()
{
    etherCsOn();
    spiWrite(0x3A);
    spiRead();
}

uint8_t etherReadMem()
{
    spiWrite(0);
    return spiRead();
}

void etherReadMemStop()
{
    etherCsOff();
}

// Initializes ethernet device
// Uses order suggested in Chapter 6 of datasheet except 6.4 OST which is first here
void etherInit(uint8_t mode)
{
    // make sure that oscillator start-up timer has expired
    while ((etherReadReg(ESTAT) & CLKRDY) == 0) {}

    // disable transmission and reception of packets
    etherClearReg(ECON1, RXEN);
    etherClearReg(ECON1, TXRTS);

    // initialize receive buffer space
    etherSetBank(ERXSTL);
    etherWriteReg(ERXSTL, LOBYTE(0x0000));
    etherWriteReg(ERXSTH, HIBYTE(0x0000));
    etherWriteReg(ERXNDL, LOBYTE(0x1A09)); //1A09
    etherWriteReg(ERXNDH, HIBYTE(0x1A09)); //1A09

    // initialize receiver write and read ptrs
    // at startup, will write from 0 to 1A08 only and will not overwrite rd ptr
    etherWriteReg(ERXWRPTL, LOBYTE(0x0000));
    etherWriteReg(ERXWRPTH, HIBYTE(0x0000));
    etherWriteReg(ERXRDPTL, LOBYTE(0x1A09));	//
    etherWriteReg(ERXRDPTH, HIBYTE(0x1A09));	//0x1A09
    etherWriteReg(ERDPTL, LOBYTE(0x0000));
    etherWriteReg(ERDPTH, HIBYTE(0x0000));

    // setup receive filter
    // always check CRC, use OR mode
    etherSetBank(ERXFCON);
    etherWriteReg(ERXFCON, (mode | 0x20) & 0xBF);
    // bring mac out of reset
    etherSetBank(MACON2);
    etherWriteReg(MACON2, 0);

    // enable mac rx, enable pause control for full duplex
    etherWriteReg(MACON1, TXPAUS | RXPAUS | MARXEN);

    // enable padding to 60 bytes (no runt packets)
    // add crc to tx packets, set full or half duplex
    if ((mode & ETHER_FULLDUPLEX) != 0)
        etherWriteReg(MACON3, FULDPX | FRMLNEN | TXCRCEN | PAD60);
    else
        etherWriteReg(MACON3, FRMLNEN | TXCRCEN | PAD60);

    // leave MACON4 as reset

    // set maximum rx packet size
    etherWriteReg(MAMXFLL, LOBYTE(1518));
    etherWriteReg(MAMXFLH, HIBYTE(1518));

    // set back-to-back uint8_ter-packet gap to 9.6us
    if ((mode & ETHER_FULLDUPLEX) != 0)
        etherWriteReg(MABBIPG, 0x15);
    else
        etherWriteReg(MABBIPG, 0x12);

    // set non-back-to-back uint8_ter-packet gap registers
    etherWriteReg(MAIPGL, 0x12);
    etherWriteReg(MAIPGH, 0x0C);

    // leave collision window MACLCON2 as reset

    // setup mac address
    etherSetBank(MAADR0);
    etherWriteReg(MAADR5, macAddress[0]);
    etherWriteReg(MAADR4, macAddress[1]);
    etherWriteReg(MAADR3, macAddress[2]);
    etherWriteReg(MAADR2, macAddress[3]);
    etherWriteReg(MAADR1, macAddress[4]);
    etherWriteReg(MAADR0, macAddress[5]);

    // initialize phy duplex
    if ((mode & ETHER_FULLDUPLEX) != 0)
        etherWritePhy(PHCON1, PDPXMD);
    else
        etherWritePhy(PHCON1, 0);

    // disable phy loopback if in half-duplex mode
    etherWritePhy(PHCON2, HDLDIS);

    // set LEDA (link status) and LEDB (tx/rx activity)
    // stretch LED on to 40ms (default)
    etherWritePhy(PHLCON, 0x0472);

    // enable reception
    etherSetReg(ECON1, RXEN);
}

// Returns TRUE if packet received
uint8_t etherKbhit()
{
    return ((etherReadReg(EIR) & PKTIF) != 0);
}

// Returns up to max_size characters in data buffer
// Returns number of bytes copied to buffer
// Contents written are 16-bit size, 16-bit status, payload excl crc
uint16_t etherGetPacket(uint8_t data[], uint16_t max_size)
{
    uint16_t i = 0, size, tmp;
    // enable read from FIFO buffers
    etherReadMemStart();
    // get next pckt information
    nextPacketLsb = etherReadMem();
    nextPacketMsb = etherReadMem();

    // calc size
    // don't return crc, instead return size + status, so size is correct
    size = etherReadMem();
    data[i++] = size;
    tmp = etherReadMem();
    data[i++] = tmp;
    size |= (tmp << 8);
    // copy status + data
    if (size > max_size)
        size = max_size;
    while (i < size)
        data[i++] = etherReadMem();
    // end read from FIFO buffers
    etherReadMemStop();
    // advance read ptr
    etherSetBank(ERXRDPTL);
    etherWriteReg(ERXRDPTL, nextPacketLsb); // hw ptr
    etherWriteReg(ERXRDPTH, nextPacketMsb);
    etherWriteReg(ERDPTL, nextPacketLsb); // dma rd ptr
    etherWriteReg(ERDPTH, nextPacketMsb);

    // decrement packet counter so that PKTIF is mauintained correctly
    etherSetReg(ECON2, PKTDEC);

    return size;
}

// Returns TRUE is rx buffer overflowed after correcting the problem
uint8_t etherIsOverflow()
{
    uint8_t err;
    err = (etherReadReg(EIR) & RXERIF) != 0;
    if (err)
        etherClearReg(EIR, RXERIF);
    return err;
}

// Writes a packet
bool etherPutPacket(uint8_t data[], uint16_t size)
{
    uint16_t i;

    // clear out any tx errors
    if ((etherReadReg(EIR) & TXERIF) != 0)
    {
        etherClearReg(EIR, TXERIF);
        etherSetReg(ECON1, TXRTS);
        etherClearReg(ECON1, TXRTS);
    }

    // set DMA start address
    etherSetBank(EWRPTL);
    etherWriteReg(EWRPTL, LOBYTE(0x1A0A));
    etherWriteReg(EWRPTH, HIBYTE(0x1A0A));

    // start FIFO buffer write
    etherWriteMemStart();

    // write control byte
    etherWriteMem(0);

    // write data
    for (i = 0; i < size; i++)
        etherWriteMem(data[i]);

    // stop write
    etherWriteMemStop();

    // request transmit
    etherWriteReg(ETXSTL, LOBYTE(0x1A0A));
    etherWriteReg(ETXSTH, HIBYTE(0x1A0A));
    etherWriteReg(ETXNDL, LOBYTE(0x1A0A+size));
    etherWriteReg(ETXNDH, HIBYTE(0x1A0A+size));
    etherClearReg(EIR, TXIF);
    etherSetReg(ECON1, TXRTS);

    // wait for completion
    while ((etherReadReg(ECON1) & TXRTS) != 0);

    // determine success
    return ((etherReadReg(ESTAT) & TXABORT) == 0);
}

// Calculate sum of words
// Must use getEtherChecksum to complete 1's compliment addition
void etherSumWords(void* data, uint16_t size_in_bytes)
{
	uint8_t* pData = (uint8_t*)data;
    uint16_t i;
    uint8_t phase = 0;
    uint16_t data_temp;
    for (i = 0; i < size_in_bytes; i++)
    {
        if (phase)
        {
            data_temp = *pData;
            sum += data_temp << 8;
        }
        else
          sum += *pData;
        phase = 1 - phase;
        pData++;
    }
}

// Completes 1's compliment addition by folding carries back uint8_to field
uint16_t getEtherChecksum()
{
    uint16_t result;
    // this is based on rfc1071
    while ((sum >> 16) > 0)
      sum = (sum & 0xFFFF) + (sum >> 16);
    result = sum & 0xFFFF;
    return ~result;
}

// Converts from host to network order and vice versa
uint16_t htons(uint16_t value)
{
    return ((value & 0xFF00) >> 8) + ((value & 0x00FF) << 8);
}

#define ntohs htons

void etherInitStructures(uint8_t data[])
{
	enc28j60 = (void*)data;
	ether = (void*)&enc28j60->data;
}
// Determines whether packet is IP datagram
uint8_t etherIsIp(uint8_t data[])
{
    uint8_t ok;
    ip = (void*)&ether->data;
    ok = (ether->frameType == 0x0008);
    if (ok)
    {
        sum = 0;
        etherSumWords(&ip->rev_size, (ip->rev_size & 0xF) * 4);
        ok = (getEtherChecksum() == 0);
        if(!ok)
        	putsUart0("IP Checksum Error! \r\n");
    }
    return ok;
}

// Determines whether packet is unicast to this ip
// Must be an IP packet
bool etherIsIpUnicast(uint8_t data[])
{
    uint8_t i = 0;
    bool ok = true;
    while (ok && (i < 4))
    {
        ok = (ip->destIp[i] == ipv4Address[i]);
        i++;
    }
    return ok;
}

// Determines whether packet is ping request
// Must be an IP packet
uint8_t etherIsPingReq(uint8_t data[])
{
    icmp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    return (ip->protocol == 0x01 && icmp->type == 8);
}

// Sends a ping response given the request data
void etherSendPingResp(uint8_t data[])
{
    uint8_t i, tmp;
    uint16_t icmp_size;
    // swap source and destination fields
    for (i = 0; i < 6; i++)
    {
        tmp = ether->destAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = tmp;
    }
    for (i = 0; i < 4; i++)
    {
        tmp = ip->destIp[i];
        ip->destIp[i] = ip ->sourceIp[i];
        ip->sourceIp[i] = tmp;
    }
    // this is a response
    icmp->type = 0;
    // calc icmp checksum
    sum = 0;
    etherSumWords(&icmp->type, 2);
    icmp_size = ntohs(ip->length);
    icmp_size -= 24; // sub ip header and icmp code, type, and check
    etherSumWords(&icmp->id, icmp_size);
    icmp->check = getEtherChecksum();
    // send packet
    etherPutPacket((uint8_t*)ether, 14 + ntohs(ip->length));
}

// Determines whether packet is ARP
uint8_t etherIsArp(uint8_t data[])
{
    uint8_t ok;
    uint8_t i = 0;
    arp = (void*)&ether->data;
    ok = (ether->frameType == 0x0608);
    while (ok && (i < 4))
    {
        ok = (arp->destIp[i] == ipv4Address[i]);
        i++;
    }
    return ok;
}

// Sends an ARP response given the request data
void etherSendArpResp(uint8_t data[])
{
    uint8_t i, tmp;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    arp = (void*)&ether->data;
    // set op to response
    arp->op = 0x0200;
    // swap source and destination fields
    for (i = 0; i < 6; i++)
    {
        arp->destAddress[i] = arp->sourceAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = arp->sourceAddress[i] = macAddress[i];
    }
    for (i = 0; i < 4; i++)
    {
        tmp = arp->destIp[i];
        arp->destIp[i] = arp->sourceIp[i];
        arp->sourceIp[i] = tmp;
    }
    // send packet
    etherPutPacket((uint8_t*)ether, 42);
}

// Sends an ARP request
void etherSendArpReq(uint8_t data[], uint8_t ip[])
{
    uint8_t i;
    ether = (void*)data;
    arp = (void*)&ether->data;
    // fill ethernet frame
    for (i = 0; i < 6; i++)
    {
        ether->destAddress[i] = 0xFF;
        ether->sourceAddress[i] = macAddress[i];
    }
    ether->frameType = 0x0608;
    // fill arp frame
    arp->hardwareType = 0x0100;
    arp->protocolType = 0x0008;
    arp->hardwareSize = 6;
    arp->protocolSize = 4;
    arp->op = 0x0100;
    for (i = 0; i < 6; i++)
    {
        arp->sourceAddress[i] = macAddress[i];
        arp->destAddress[i] = 0xFF;
    }
    for (i = 0; i < 4; i++)
    {
        arp->sourceIp[i] = ipv4Address[i];
        arp->destIp[i] = ip[i];
    }
    // send packet
    etherPutPacket(data, 42);
}

// Determines whether packet is UDP datagram
// Must be an IP packet
uint8_t etherIsUdp(uint8_t data[])
{
    uint8_t ok;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    udp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    ok = (ip->protocol == 0x11);
    if(ok)
    {
    		uint16_t dest = 0x00;
    		dest = ntohs(udp->destPort);
        	ok = (dest == 40001);
        	//if(!ok)
        	//	putsUart0("UDP Packet received with wrong destination port Error! \n");
    }
    if (ok)
    {
        // 32-bit sum over pseudo-header
        sum = 0;
        etherSumWords(ip->sourceIp, 8);
        tmp_int = ip->protocol;
        sum += (tmp_int & 0xff) << 8;
        etherSumWords(&udp->length, 2);
        // add udp header and data
        etherSumWords(udp, ntohs(udp->length));
        ok = (getEtherChecksum() == 0);
        if(!ok)
        	putsUart0("UDP Packet received with checksum Error! \r\n");
    }
    return ok;
}
uint8_t etherIsDataUdp(uint8_t data[])
{
	uint8_t ok;
	uint16_t tmp_int;
	ok = (ip->protocol == 0x11);
	if(ok && ntohs(udp->destPort) == 40001)
	{
		// 32-bit sum over pseudo-header
		sum = 0;
		etherSumWords(ip->sourceIp, 8);
		tmp_int = ip->protocol;
		sum += (tmp_int & 0xff) << 8;
		etherSumWords(&udp->length, 2);
		// add udp header and data
		etherSumWords(udp, ntohs(udp->length));
		ok = (getEtherChecksum() == 0);
		if(!ok)
			putsUart0("UDP Packet received with checksum Error! \r\n");
		else
		{
			strncpy(packets[BWriteIndex],&(udp->data),udp->length - 8);
			BWriteIndex++;
			UDPPacketCount++;
		}

	}
	return ok;
}
// Gets pouint8_ter to UDP payload of frame
uint8_t* etherGetUdpData(uint8_t data[])
{
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    udp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
    return &udp->data;
}

void etherCalcIpChecksum()
{
    // 32-bit sum over ip header
    sum = 0;
    etherSumWords(&ip->rev_size, 10);
    etherSumWords(ip->sourceIp, ((ip->rev_size & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();
}
uint8_t etherIsDiscReq(uint8_t data[])
{
	uint8_t ok = 0;
	udp = (void*)((uint8_t*)&ether->data + ((ip->rev_size & 0xF) * 4));
	ctrlInfo = (void*)&udp->data;
	if(ctrlInfo->ctrlType == 1 && ctrlInfo->opCode == 1)
		ok = 1;
	return ok;
}
void etherSendDiscResponse(uint8_t data[])
{
	uint8_t i;
	uint16_t tmp_int;
	// fill ethernet frame
	for (i = 0; i < 6; i++)
	{
		ether->destAddress[i] = 0xFF;
		ether->sourceAddress[i] = macAddress[i];
	}
	ether->frameType = 0x0008;
	// fill ip header

	for (i = 0; i < 4; i++)
	{
		ip->sourceIp[i] = ipv4Address[i];
		ip->destIp[i] = broadCastIp[i];
	}
	ip->rev_size = 0x45;
	ip->typeOfService = 0;
	ip->id = ip->id+1;
	ip->id %= 65535;
	ip->flagsAndOffset = 0x40;
	ip->ttl = 0x50;
	ip->protocol = 0x11;

	udp->sourcePort = htons(sourcePort);
	udp->destPort = htons(ctrlDest);
	ip->length = htons(((ip->rev_size & 0xF) * 4) + 8 + 6 + 16);

	sum = 0;
	etherSumWords(&ip->rev_size, 10);
	etherSumWords(ip->sourceIp, ((ip->rev_size & 0xF) * 4) - 12);
	ip->headerChecksum = getEtherChecksum();
	udp->length = htons(8 + 6 + 16);

	ctrlInfo = (void*)&udp->data;
	ctrlInfo->validationCode = 0x95;
	ctrlInfo->deviceType = 100;
	ctrlInfo->nodeId = 100;
	ctrlInfo->ctrlType = 1;
	ctrlInfo->opCode = 2;
	ctrlInfo->noOfCtrlMessages = 1;
	ctrlData = (void*)&ctrlInfo->data;

	ctrlData->localFunctionID = 00;
	ctrlData->functionType = 01;
	ctrlData->dataType = 01;
	ctrlData->lengthOfFriendlyString = 0x0C;
	strncpy(&ctrlData->stringData,"Data Sniffer",0x0C);
	//ctrlData[1] = (void*)(&ctrlData[0]->stringData + 13);

	/*ctrlData[1]->localFunctionID = 01;
	ctrlData[1]->functionType = 02;
	ctrlData[1]->dataType = 01;
	ctrlData[1]->lengthOfFriendlyString = 0x0D;
	strncpy(&ctrlData[1]->stringData,"Data Sniffing",0x0D);*/

	sum = 0;
	etherSumWords(ip->sourceIp, 8);
	tmp_int = ip->protocol;
	sum += (tmp_int & 0xff) << 8;
	etherSumWords(&udp->length, 2);

	etherSumWords(udp, 6);
	etherSumWords(&udp->data, 6 + 16);
	udp->check = getEtherChecksum();

	etherPutPacket((uint8_t*)ether, 14 + ((ip->rev_size & 0xF) * 4) + 8 + 6 + 16);
}
// Send responses to a udp datagram
// destination port, ip, and hardware address are extracted from provided data
// uses destination port of received packet as destination of this packet
void etherSendUdpData(uint8_t data[], uint8_t* udp_data, uint8_t udp_size)
{
    uint8_t *copy_data;
    uint8_t i, tmp;
    uint16_t tmp_int;
    enc28j60 = (void*)data;
    ether = (void*)&enc28j60->data;
    ip = (void*)&ether->data;
    udp = (void*)((uint8_t*)&ether->data + ((ip->rev_size & 0xF) * 4));
    // swap source and destination fields
    for (i = 0; i < 6; i++)
    {
        tmp = ether->destAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = tmp;
    }
    for (i = 0; i < 4; i++)
    {
        tmp = ip->destIp[i];
        ip->destIp[i] = ip->sourceIp[i];
        ip->sourceIp[i] = tmp;
    }
    // set source port of resp will be dest port of req
    // dest port of resp will be left at source port of req
    // unusual nomenclature, but this allows a different tx
    // and rx port on other machine
    udp->sourcePort = udp->destPort;
    // adjust lengths
    ip->length = htons(((ip->rev_size & 0xF) * 4) + 8 + udp_size);
    // 32-bit sum over ip header
    sum = 0;
    etherSumWords(&ip->rev_size, 10);
    etherSumWords(ip->sourceIp, ((ip->rev_size & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();
    udp->length = htons(8 + udp_size);
    // copy data
    copy_data = &udp->data;
    for (i = 0; i < udp_size; i++)
        copy_data[i] = udp_data[i];
        // 32-bit sum over pseudo-header
    sum = 0;
    etherSumWords(ip->sourceIp, 8);
    tmp_int = ip->protocol;
    sum += (tmp_int & 0xff) << 8;
    etherSumWords(&udp->length, 2);
    // add udp header except crc
    etherSumWords(udp, 6);
    etherSumWords(&udp->data, udp_size);
    udp->check = getEtherChecksum();

    // send packet with size = ether + udp hdr + ip header + udp_size
    etherPutPacket((uint8_t*)ether, 22 + ((ip->rev_size & 0xF) * 4) + udp_size);
}

uint16_t etherGetId()
{
    return htons(sequenceId);
}

void etherIncId()
{
    sequenceId++;
}

// Determines if the IP address is valid
bool etherIsValidIp()
{
    return ipv4Address[0] || ipv4Address[1] || ipv4Address[2] || ipv4Address[3];
}

// Sets IP address
void etherSetIpAddress(uint8_t a, uint8_t b,  uint8_t c, uint8_t d)
{
    ipv4Address[0] = a;
    ipv4Address[1] = b;
    ipv4Address[2] = c;
    ipv4Address[3] = d;
}
//******************************************* RTOS Support Functions ******************************************//
// Task control Block Structure
struct _tcb
{
	uint8_t state;                 // see STATE_ values above
	void *pid;                     // used to uniquely identify process
	void *sp;                      // location of stack pointer for process
	uint8_t priority;              // 0=highest, 7=lowest
	uint8_t currentPriority;       // used for priority check
	char name[15];				   // Name of the task
	uint32_t ticks;				   // Ticks until sleep complete
} tcb[MAX_TASKS];

// Initializing RTOS tasks
void rtosInit()
{
	uint8_t i;
	taskCount = 0;							// no tasks running
	// clear out all tcb records
	for (i = 0; i < MAX_TASKS; i++)
	{
		tcb[i].state = STATE_INVALID;
		tcb[i].pid = 0;
	}
}
// Schedular to dispatch next Task to execute
int rtosScheduler()
{
	bool ok;
	static uint8_t task = 0xFF;
	ok = false;
	while (!ok)
	{
		task++;
		if (task >= MAX_TASKS)
			task = 0;
		// Here currentPriority is used as a SKIPCOUNT parameter
		if(tcb[task].state == STATE_READY)
		{
			if(tcb[task].currentPriority == tcb[task].priority)
			{
				tcb[task].currentPriority = 0;
				ok = true;
			}
			else
			{
				if(tcb[task].currentPriority < tcb[task].priority)
					tcb[task].currentPriority++ ;
			}
		}
	}
	return task;
}
// To Run the First task
void rtosStart()
{
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
	taskCurrent = rtosScheduler();
	update_SP(tcb[taskCurrent].sp);
	__asm(" POP { R4-R11 } ");
	__asm(" pop {R15} ");
}
// To create a Process if it doesn't exist
bool createProcess(_fn fn, int priority,char *name)
{
	NVIC_ST_CTRL_R&= ~NVIC_ST_CTRL_INTEN;
	bool ok = false;
	uint8_t i = 0;
	bool found = false;
	// REQUIRED: take steps to ensure a task switch cannot occur
	if (taskCount < MAX_TASKS)
	{
		// make sure fn not already in list (prevent reentrancy)
		while (!found && (i < MAX_TASKS))
		{
			found = (tcb[i++].pid ==  fn);
		}
		if (!found)
		{
			// find first available tcb record
			i = 0;
			while (tcb[i].state != STATE_INVALID) {i++;}
			// Task Initialization
			tcb[i].pid = fn;
			tcb[i].sp = &stack[i][243];
			stack[i][251] = (int)fn;
			tcb[i].priority = priority;
			tcb[i].currentPriority = priority;
			strcpy(tcb[i].name,name);
			tcb[i].state = STATE_READY;
			taskCount++;
			ok = true;
		}
	}
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_INTEN;
	return ok;
}
// To update the stackpointer to given value
void update_SP(void* sp)
{
	__asm(" ADD R13,#8");	// To eliminate SP-8 Effect of compiler
	__asm(" MOV R13,R0");
	__asm(" SUB R13,#8");	// To eliminate SP+8 Effect of compiler
}
// To get current SP value
void* get_SP()
{
	__asm(" MOV R0,R13");
	__asm(" SUB R13,#8");
}
// Yield proceesor for other Tasks
void yield()
{
	__asm(" ADD R13,#8 ");
	__asm(" PUSH {R14} ");
	__asm(" PUSH {R4-R11} ");
	tcb[taskCurrent].sp =get_SP();
	taskCurrent = rtosScheduler();
	update_SP(tcb[taskCurrent].sp);
	__asm(" POP { R4-R11 } ");
	__asm(" POP {R15} ");
}
// To set Sleep time to a task and execute other
void sleep(uint32_t tick)
{
	tcb[taskCurrent].ticks = tick;
	__asm(" ADD R13,#8 ");
	__asm(" PUSH {R14} ");
	__asm(" PUSH {R4-R11} ");
	tcb[taskCurrent].sp =get_SP();
	tcb[taskCurrent].state = STATE_DELAYED;
	taskCurrent = rtosScheduler();
	update_SP(tcb[taskCurrent].sp);
	__asm(" POP { R4-R11 } ");
	__asm(" POP {R15} ");
}
// function to support for the system timer
void systickIsr()
{
	int i=0;
	// Decrement tick count for all delayed Tasks
	for(i=0;i<MAX_TASKS;i++)
	{
		if(tcb[i].state == STATE_DELAYED )
		{
			tcb[i].ticks -- ;
			if(tcb[i].ticks == 0)
				tcb[i].state = STATE_READY;
		}
	}
}



//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
	SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

	// Set GPIO ports to use APB (not needed since default configuration -- for clarity)
	// Note UART on port A must use APB
	SYSCTL_GPIOHBCTL_R = 0;

	// Enable GPIO port B and E peripherals
	SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOF;

	// Configure LED and pushbutton pins
	GPIO_PORTF_DIR_R = 0x0E;  // bits 1-3 are outputs, other pins are inputs
	GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTF_DEN_R = 0x1E;  // enable LEDs and pushbuttons
	GPIO_PORTF_PUR_R = 0x1E;  // enable internal pull-up for push button

	GPIO_PORTD_DIR_R = 0x0;  // bits 1-3 are outputs, other pins are inputs
	GPIO_PORTD_DEN_R = 0x0;  // enable LEDs and pushbuttons

	// Configure ~CS for ENC28J60
	GPIO_PORTB_DIR_R = 0x02;  // make bit 1 an output
	GPIO_PORTB_DR2R_R = 0x02; // set drive strength to 2mA
	GPIO_PORTB_DEN_R = 0x02;  // enable bits 1 for digital

	// Configure SSI2 pins for SPI configuration
	SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
	GPIO_PORTB_DIR_R |= 0x90;                        // make bits 4 and 7 outputs
	GPIO_PORTB_DR2R_R |= 0x90;                       // set drive strength to 2mA
	GPIO_PORTB_AFSEL_R |= 0xD0;                      // select alternative functions for MOSI, MISO, SCLK pins
	GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB6_SSI2RX | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI2
	GPIO_PORTB_DEN_R |= 0xD0;                        // enable digital operation on TX, RX, CLK pins

	// Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
	SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
	SSI2_CR1_R = 0;                                  // select master mode
	SSI2_CC_R = 0;                                   // select system clock as the clock source
	SSI2_CPSR_R = 40;                                // set bit rate to 1 MHz (if SR=0 in CR0)
	SSI2_CR0_R = SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8;   // set SR=0, mode 0 (SPH=0, SPO=0), 8-bit
	SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2
	// UART0 Configuration
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
	__asm(" NOP");                                  // wait 3 clocks
	__asm(" NOP");
	__asm(" NOP");
	GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
	GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
	UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
	UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
	UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
	UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
	UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
	                                            // Approx clocks per us
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// listen to network task must be ready at all times  to prevent scheduler fail
void listenToNetwork()
{
  while(1)
  {
	  if (etherKbhit())
	  {
		  if (etherIsOverflow())
		  {
			  RED_LED = 1;
			  putsUart0("\t \e[31moverflow....\r\n");
			  waitMicrosecond(100000);
			  RED_LED = 0;
		  }
		  etherGetPacket(data, 256);
		  etherInitStructures(data);
		  // handle arp request
		  if (etherIsArp(data))
		  {
			  etherSendArpResp(data);
			  RED_LED = 1;
			  GREEN_LED = 1;
			  waitMicrosecond(50000);
			  RED_LED = 0;
			  GREEN_LED = 0;
		  }
		  // handle ip datagram
		  else if (etherIsIp(data))
		  {
			  // handle icmp ping request
			  if (etherIsPingReq(data))
			  {
				  if (etherIsIpUnicast(data))
				  {
					  etherSendPingResp(data);
					  RED_LED = 1;
					  BLUE_LED = 1;
					  waitMicrosecond(50000);
					  RED_LED = 0;
					  BLUE_LED = 0;
				  }
			  }
			  if(etherIsDiscReq(data))
			  {
				  etherSendDiscResponse(data);
				  BLUE_LED = 1;
				  waitMicrosecond(50000);
				  BLUE_LED = 0;
			  }
			  // handle udp datagram
			  if (etherIsDataUdp(data))
			  {
				  GREEN_LED = 1;
				  strncpy(packets[BWriteIndex],&udp->data,udp->length - )
				  GREEN_LED = 0;
				  BLUE_LED = 1;
				  waitMicrosecond(100000);
				  BLUE_LED = 0;
			  }
		  }
	  }
	  else
	  {
		  yield();
	  }
  }
}
void copyDataToBuffer(data)
{


}
void processPacket()
{
	while(true)
	{
		enc28j60 = (void*)data;
		ether = (void*)&enc28j60->data;
		ip = (void*)&ether->data;
		udp = (void*)((uint8_t*)ip + ((ip->rev_size & 0xF) * 4));
		dataInfo = (void*)&udp->data;
		uint8_t m_validationCode = dataInfo->validationCode;
		if(m_validationCode != 0x95)
		{
			putsUart0("Not our IoT packet or Invalid validation code recieved In your Packet!\r\n");
			return;
		}
		uint8_t m_nodeId = dataInfo -> nodeId;
		switch(m_nodeId)
		{
			case n_thermostat:
				putsUart0("Thermostat Node: \r\n");
				processThermostatPacket(&udp->data);
				break;
			case n_weatherSensors:
				putsUart0("Weather Sensors Node: \r\n");
				processWeatherSensorsPacket(&udp->data);
				break;
			case n_weatherSensorsDisplay:
				putsUart0("Weather Sensors Display Node: \r\n");
				break;
			case n_newsFeed:
				putsUart0("NewsFeed service: \r\n");
				processNewsFeedPacket(&udp->data);
				break;
			case n_electricityCost:
				putsUart0("Electrcity Cost Service: \r\n");
				processElectrictyCostPacket(&udp->data);
				break;
			case n_weatherServices:
				putsUart0("Weather Services: \r\n");
				processWeatherServicesPacket(&udp->data);
				break;
			case n_stormAlert:
				putsUart0("Storm Alert Service: \r\n");
				processStromAlertPacket(&udp->data);
				break;
			case n_webMessage:
				putsUart0("WebMessaging Node :\r\n");
				break;
			case n_jpeg:
				putsUart0("Jpeg image Node: \r\n");
				break;
			case n_timeServices:
				putsUart0("Time services :\r\n");
				processTimeServicesPacket(&udp->data);
				break;
			case n_newsFeedDisplay:
				putsUart0("News feed display :\r\n");
				break;
			case n_batteryCharger:
				putsUart0("BatteryStatus Node :\r\n");
				ProcessBatteryChargerPacket(&udp->data);
				break;
			default:
				putsUart0("Packet from packet from Unnown Node.\r\n");
				break;
		}
	}
}


// Part of some lengthy function
void partOfLengthyFn()
{
	waitMicrosecond(1000);
	yield();
}
// Lengthy Function
void lengthyFn()
{
  uint16_t i;
  while(true)
  {
    for (i = 0; i < 4000; i++)
    {
      partOfLengthyFn();
    }
    RED_LED ^= 1;
  }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

	// Initialize hardware
	initHw();
	rtosInit();

	// Power-up flash
	RED_LED = 1;
	waitMicrosecond(250000);
	RED_LED = 0;
	waitMicrosecond(250000);
	char name[15]={'\0'};
	strcpy(name,"listenToNetwork");
	ok =  createProcess(listenToNetwork,0,name);
	strcpy(name,"processPacket");
	ok &= createProcess(processPacket, 2,name);
	putsUart0("\r\n");
	// Start up RTOS
	if (ok)
	  rtosStart();
	else
	  RED_LED = 1;

    return 0;
    // don't delete this unreachable code
    // if a function is only called once in your code, it will be
    // accessed with two goto instructions instead of call-return,
    // so any stack-based code will not function correctly
    yield(); sleep(0);
}


