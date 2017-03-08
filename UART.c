// UART functions
 Author: NagendraBabu

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "UART.h"

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Print character to UART terminal
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}
// To print string to UART terminal
void putsUart0(char* str)
{
	int i;
	int slen = strlen(str);
    for (i = 0; i < slen; i++)
	  putcUart0(str[i]);
}
// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);
	return UART0_DR_R & 0xFF;
}
// To print hexvalues in string format starting from given address
void putsnUart0(uint8_t *str,int n)
{
	int i=0;
	for(i=0;i<n;i++)
		putcUart0(str[i]&0xFF);
}
// To convert integer to string
void intTostring(int value,char *str)
{
	int num = value;
	int i=0;
	bool isNegative=false;
	char res[5];
    int len = 0;
    if(num==0)
    {
    	*str='0';
    	str++;
    	*str='\0';
    	return;
    }
    if(num<0)
    {
    	isNegative=true;
    	num=0-num;
    }
    for(; num > 0; ++len)
    {
       res[len] = num%10+'0';
       num/=10;
    }
    if(isNegative)
    {
    	res[len]='-';
    	len=len+1;
    }
    res[len] = 0; //null-terminating
    //now we need to reverse result
    for(i = 0; i < len/2; ++i)
    {
        char c = res[i]; res[i] = res[len-i-1]; res[len-i-1] = c;
    }
    for(i=0;i<len;i++)
    {
    	*str=res[i];
    	str++;
    }
    *str='\0';
}
