#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "common.h"

void Init_System(void);
void Init_Para(void);
void Init_Drivers(void);
void Inductor_init(void);

//Algorithms
void QuikSort(float arr[], int low, int high) reentrant;
int getStandard(int arr[], int i, int j);


#endif