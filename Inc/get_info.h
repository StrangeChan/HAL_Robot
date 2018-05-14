#ifndef __GETPOSITION_H
#define __GETPOSITION_H


#include "stm32f4xx_hal.h"
#include "tim.h"
#include "control.h"
#include "sys.h"


void ReadEncoder(void);
void GetYaw(void);
void GetPosition(void);		//����ת��

u8 GetVisionData(void);		//�Ӿ����ݴ���
u8 GetRadarData(void);		//���⴦������

#endif

