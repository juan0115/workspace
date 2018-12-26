#ifndef DPROCESS_H
#define DPROCESS_H

#include <math.h>
#include <termio.h>
#include <stdio.h>
#include"afx.h"
#include"serial.h"

void* SendComd(void *arg);
void* Process_data(void *arg);
void* set_server15000(void *arg);
void* set_server13000(void *arg);
int control_sweeper(char *serverIP, int fd);

#endif