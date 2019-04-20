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
void control_sweeper(int fd);

#define odo_axis 237.5          //主动轮轴距,单位：mm
#define odo_pulse 1620          //主动轮旋转一周脉冲总量
#define odo_perimeter 219.8     //轮周长

#endif