/* Courtesy of Chris Cook and Kevin Gill 
 * CSC460 2016 Roomba Tank
 *
 */ 

#ifndef UART_H_
#define UART_H_

#include <stdint.h>

void Roomba_UART_Init(void);
void Roomba_Send_Byte(uint8_t);
unsigned char Roomba_Receive_Byte(void);
void Roomba_Send_String(char*);

void Bluetooth_UART_Init(void);
void Bluetooth_Send_Byte(uint8_t);
unsigned char Bluetooth_Receive_Byte(void);
void Bluetooth_Send_String(char*);

#endif /* UART_H_ */