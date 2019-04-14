#ifndef DPROCESS_H
#define DPROCESS_H


#include"afx.h"
#include"serial.h"
#define tfm(x) (3.14159*(x)/180.0)

#define MAP_BARRIER_BLACK       4
#define MAP_LOCUS_RED           9
#define MAP_EXPLORED_WHITE      1 
#define MAP_UNEXPLORED_GRAY     0

void* SendComd(void *arg);
void* Process_data(void *arg);
void* set_server(void* arg);
void* set_client(void* arg);
void* send_map(void *arg);
void cv_map(char *_ip);


#endif