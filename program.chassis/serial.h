#ifndef SERIAL_h
#define SERIAL_h

#include"afx.h"

int UART0_Open(char* port); 
int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity);
int UART0_Recv(int fd, unsigned char *rcv_buf,int data_len);
int UART0_Send(int fd, char *send_buf,int data_len); 
void UART0_Close(int fd); 


#endif