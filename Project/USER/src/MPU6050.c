 #include "common.h"
 #include "MPU6050.h"
 #include "Balance.h"
 #include <math.h>

// char Offset_OK = 0;

// /*
//  * ��������MPU6050_Offset
//  * ����  ���������ɼ���ƫ
//  * ����  ����
//  * ���  ����
//  * ����  ���ڲ�����
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
// 		MPU6050_GetData(&GYRO, &_ACC);	// ��ȡ����������
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
//  * ��������MPU6050_GetData
//  * ����  ����ô�������������
//  * ����  ��*GYRO ������		*_ACC ���ٶȼ�
//  * ���  ����
//  * ����  ���ⲿ����
//  */
// void MPU6050_GetData(S_INT16_XYZ *GYRO, S_INT16_XYZ *_ACC)
// {
// 	if (Offset_OK)
// 	{
// 		mpu_acc_x = GetData(MPU_ACCEL_XOUTH_REG);	// ��ȡ���ٶȼ�ԭʼ����
// 		mpu_acc_y = GetData(MPU_ACCEL_YOUTH_REG);
// 		mpu_acc_z = GetData(MPU_ACCEL_ZOUTH_REG);
		
// 		mpu_gyro_x = GetData(MPU_GYRO_XOUTH_REG) - GYRO_Offset.X;	// ��ȡ������ԭʼ����
// 		mpu_gyro_y = GetData(MPU_GYRO_YOUTH_REG) - GYRO_Offset.Y;
// 		mpu_gyro_z = GetData(MPU_GYRO_ZOUTH_REG) - GYRO_Offset.Z;
// 	}
// 	else
// 	{
// 		mpu_acc_x = GetData(MPU_ACCEL_XOUTH_REG);	// ��ȡ���ٶȼ�ԭʼ���ݲ���һ��
// 		mpu_acc_y = GetData(MPU_ACCEL_YOUTH_REG);
// 		mpu_acc_z = GetData(MPU_ACCEL_ZOUTH_REG);
		
// 		mpu_gyro_x = GetData(MPU_GYRO_XOUTH_REG);	// ��ȡ������ԭʼ���ݲ���һ��
// 		mpu_gyro_y = GetData(MPU_GYRO_YOUTH_REG);
// 		mpu_gyro_z = GetData(MPU_GYRO_ZOUTH_REG);
// 	}
// }

// /*
//  * ��������GetData
//  * ����  �����16λ����
//  * ����  ��REG_Address �Ĵ�����ַ
//  * ���  �����ؼĴ�������
//  * ����  ���ⲿ����
//  */
// int16 GetData(uint8 REG_Address)
// {
// 	uint8 H, L;

// 	H = MPU6050_RD(REG_Address);
// 	L = MPU6050_RD(REG_Address+1);
	
// 	return ((H<<8)|L);   //�ϳ�����
// }