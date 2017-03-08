// UART
//  Author: NagendraBabu

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

#ifndef _UART_H_
#define _UART_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void putcUart0(char c);
void putsUart0(char* str);
void intTostring(int value,char *str);
char getcUart0(void);
void putsnUart0(uint8_t *str,int n);
#endif
