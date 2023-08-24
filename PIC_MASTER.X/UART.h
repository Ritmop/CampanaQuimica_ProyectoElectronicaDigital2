/* 
 * File:   UART.h
 * Author: judah
 *
 * Created on July 27, 2023, 3:13 PM
 */

#ifndef UART_H
#define	UART_H

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000
#endif

#include <xc.h>

void UART_RX_config(long baudrate);
void UART_TX_config(long baudrate);
void UART_write_char(char c);
void UART_write_string(char *s);
char UART_read_char();

#endif	/* UART_H */

