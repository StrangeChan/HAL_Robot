#ifndef __GETPOSITION_H
#define __GETPOSITION_H


#include "stm32f4xx_hal.h"
#include "tim.h"
#include "control.h"
#include "sys.h"



void ReadEncoder(void);
void GetYaw(void);
void GetPosition(void);		//坐标转换

void GetVisionData(void);		//视觉数据处理
void GetRadarData(void);		//激光处理数据

#endif

