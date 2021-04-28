 #include "common.h"
 #include "MPU6050.h"
 #include "Balance.h"
 #include <math.h>

// char Offset_OK = 0;

// /*
//  * 函数名：MPU6050_Offset
//  * 描述  ：传感器采集零偏
//  * 输入  ：无
//  * 输出  ：无
//  * 调用  ：内部调用
//  */
// void MPU6050_Offset(void)
// {
// 	uint8 i = 0;
// 	int32 temp[6] = {0};
	
// 	GYRO_Offset.X = 0;
// 	GYRO_Offset.Y = 0;
// 	GYRO_Offset.Z = 0;
	
// 	for (i = 0; i < 100; i++)
// 	{
// 		MPU6050_GetData(&GYRO, &_ACC);	// 读取陀螺仪数据
// 		DELAY_MS(2);
		
// 		temp[0] += _ACC.X;
// 		temp[1] += _ACC.Y;
// 		temp[2] += _ACC.Z;
		
// 		temp[3] += GYRO.X;
// 		temp[4] += GYRO.Y;
// 		temp[5] += GYRO.Z;
// 	}
// 	ACC_Offset.X = temp[0] / 100;
// 	ACC_Offset.Y = temp[1] / 100;
// 	ACC_Offset.Z = temp[2] / 100;
	
// 	GYRO_Offset.X = temp[3] / 100;
// 	GYRO_Offset.Y = temp[4] / 100;
// 	GYRO_Offset.Z = temp[5] / 100;
	
// 	Offset_OK = 1;
// }

// /*
//  * 函数名：MPU6050_GetData
//  * 描述  ：获得传感器所有数据
//  * 输入  ：*GYRO 陀螺仪		*_ACC 加速度计
//  * 输出  ：无
//  * 调用  ：外部调用
//  */
// void MPU6050_GetData(S_INT16_XYZ *GYRO, S_INT16_XYZ *_ACC)
// {
// 	if (Offset_OK)
// 	{
// 		mpu_acc_x = GetData(MPU_ACCEL_XOUTH_REG);	// 获取加速度计原始数据
// 		mpu_acc_y = GetData(MPU_ACCEL_YOUTH_REG);
// 		mpu_acc_z = GetData(MPU_ACCEL_ZOUTH_REG);
		
// 		mpu_gyro_x = GetData(MPU_GYRO_XOUTH_REG) - GYRO_Offset.X;	// 获取陀螺仪原始数据
// 		mpu_gyro_y = GetData(MPU_GYRO_YOUTH_REG) - GYRO_Offset.Y;
// 		mpu_gyro_z = GetData(MPU_GYRO_ZOUTH_REG) - GYRO_Offset.Z;
// 	}
// 	else
// 	{
// 		mpu_acc_x = GetData(MPU_ACCEL_XOUTH_REG);	// 获取加速度计原始数据并归一化
// 		mpu_acc_y = GetData(MPU_ACCEL_YOUTH_REG);
// 		mpu_acc_z = GetData(MPU_ACCEL_ZOUTH_REG);
		
// 		mpu_gyro_x = GetData(MPU_GYRO_XOUTH_REG);	// 获取陀螺仪原始数据并归一化
// 		mpu_gyro_y = GetData(MPU_GYRO_YOUTH_REG);
// 		mpu_gyro_z = GetData(MPU_GYRO_ZOUTH_REG);
// 	}
// }

// /*
//  * 函数名：GetData
//  * 描述  ：获得16位数据
//  * 输入  ：REG_Address 寄存器地址
//  * 输出  ：返回寄存器数据
//  * 调用  ：外部调用
//  */
// int16 GetData(uint8 REG_Address)
// {
// 	uint8 H, L;

// 	H = MPU6050_RD(REG_Address);
// 	L = MPU6050_RD(REG_Address+1);
	
// 	return ((H<<8)|L);   //合成数据
// }