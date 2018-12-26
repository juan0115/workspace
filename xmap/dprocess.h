#ifndef DPROCESS_H
#define DPROCESS_H


#include"afx.h"
#include"serial.h"
#define tfm(x) (3.14159*(x)/180.0)

void* SendComd(void *arg);
void* Process_data(void *arg);
void* set_server(void* arg);
void* set_client(void* arg);
void* send_map(void *arg);
void cv_map(char *_ip);


#endif