/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		MPU6050
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 * @note		
					���߶��壺
					------------------------------------ 
						���IIC
                        SCL                 �鿴SEEKFREE_IIC�ļ��ڵ�SEEKFREE_SCL�궨��
						SDA                 �鿴SEEKFREE_IIC�ļ��ڵ�SEEKFREE_SDA�궨��
					------------------------------------ 
 ********************************************************************************************************************/


#include "SEEKFREE_IIC.h"
#include "SEEKFREE_MPU6050.h"
#include "zf_delay.h"
#include "SEEKFREE_OLED.h"

#define M_PI 3.14159265358979323846


int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;





//-------------------------------------------------------------------------------------------------------------------
//  @brief      MPU6050�Լ캯��
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_self1_check(void)
{
    simiic_write_reg(MPU6050_DEV_ADDR, PWR_MGMT_1, 0x00);	//�������״̬
    simiic_write_reg(MPU6050_DEV_ADDR, SMPLRT_DIV, 0x07);   //125HZ������
    while(0x07 != simiic_read_reg(MPU6050_DEV_ADDR, SMPLRT_DIV,SIMIIC))
    {
		delay_ms(1);
        //��������ԭ�������¼���
        //1 MPU6050���ˣ�������µ������ĸ��ʼ���
        //2 ���ߴ������û�нӺ�
        //3 ��������Ҫ����������裬������3.3V
		//4 ����û�е���ģ��IIC�ĳ�ʼ������
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʼ��MPU6050
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_init(void)
{
    delay_ms(100);                                   //�ϵ���ʱ

    mpu6050_self1_check();
    simiic_write_reg(MPU6050_DEV_ADDR, PWR_MGMT_1, 0x00);	//�������״̬
    simiic_write_reg(MPU6050_DEV_ADDR, SMPLRT_DIV, 0x07);   //125HZ������
    simiic_write_reg(MPU6050_DEV_ADDR, MPU6050_CONFIG, 0x04);       //
    simiic_write_reg(MPU6050_DEV_ADDR, GYRO_CONFIG, 0x18);  //2000
    simiic_write_reg(MPU6050_DEV_ADDR, ACCEL_CONFIG, 0x10); //8g
	simiic_write_reg(MPU6050_DEV_ADDR, User_Control, 0x00);
    simiic_write_reg(MPU6050_DEV_ADDR, INT_PIN_CFG, 0x02);
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡMPU6050���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_get_accdata(void)
{
    uint8 dat[6];

    simiic_read_regs(MPU6050_DEV_ADDR, ACCEL_XOUT_H, dat, 6, SIMIIC);  
    mpu_acc_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    mpu_acc_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    mpu_acc_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡMPU6050����������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_get_gyro(void)
{
    uint8 dat[6];

    simiic_read_regs(MPU6050_DEV_ADDR, GYRO_XOUT_H, dat, 6, SIMIIC);  
    mpu_gyro_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    mpu_gyro_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    mpu_gyro_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡMPU6050������&&���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void MPU6050_get_GyandAcc(void)
{
    mpu6050_get_accdata();
    mpu6050_get_gyro();
}

//get accel and gyro from iam20609 
// ��accelһ�׵�ͨ�˲�(�ο�����)����gyroת�ɻ���ÿ��(2000dps)
#define new_weight           0.35f
#define old_weight           0.65f
void IMU_getValues(float * values) 
{  
    static float lastaccel[3]= {0,0,0};
    int i;
    values[0] = ((float)mpu_acc_x) * new_weight + lastaccel[0] * old_weight;
    values[1] = ((float)mpu_acc_y) * new_weight + lastaccel[1] * old_weight;
    values[2] = ((float)mpu_acc_z) * new_weight + lastaccel[2] * old_weight;
    for(i=0; i<3; i++)
    {
        lastaccel[i] = values[i];
    }
 
    values[3] = ((float)mpu_gyro_x) * M_PI / 180 / 16.4f;
    values[4] = ((float)mpu_gyro_y) * M_PI / 180 / 16.4f;
    values[5] = ((float)mpu_gyro_z) * M_PI / 180 / 16.4f;
 
    //
}
float Angle=0.0;
float Gyro_x=0.0;
float Q_angle=0.001; //����������Э����
float Q_gyro=0.003;  //������Ư������Э����
float R_angle=0.03;   //���ٶȼƲ�������Э����
float dt=0.001;
float PCt_0=0, PCt_1=0, E=0;	//�м����
char C_0=1;			 //H�����ϵ��
float Q_bias=0,Angle_err=0;
float K_0=0,K_1=0,t_0=0,t_1=0; //����������
float Pdot[4] ={0,0,0,0};	   //P�����м����
float PP[2][2] = { { 1, 0 },{ 0, 1 } };	  //P����  X��Э����

 
float KalmanFilter(float Accel,float Gyro)		
{
    static float Angle = 0, Gyro_y = 0;
	Angle+=(Gyro - Q_bias) * dt; //�������
 
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��
 
	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3]= Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // ����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	
	PCt_0 = C_0 * PP[0][0];		//a
	PCt_1 = C_0 * PP[1][0];		//c
	E = R_angle + C_0 * PCt_0;	//��ĸ
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	Angle_err = Accel - Angle;	//zk-�������	
	Angle	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	Gyro_y   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];
 
	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	return Angle;
}
// #define delta_T      0.005f  //5ms����һ��
 
// float I_ex, I_ey, I_ez;  // ������
// quaterInfo_t Q_info;  // ȫ����Ԫ��
// eulerianAngles_t eulerAngle; //ŷ����
// float param_Kp = 50.0;   // ���ٶȼ�(������)���������ʱ�������50 
// float param_Ki = 0.20;   //�������������ʵĻ������� 0.2
typedef struct
{
	float w, x, y, z;
}Quat_t;//��Ԫ�ؽṹ��
// w = cos(theta/2)    
// x  = ax * sin(theta/2)    
// y  = ay * sin(theta/2)    
// z  = az * sin(theta/2)  


typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
}Euler_t;//ŷ���ǽṹ��


//ŷ����ת��Ԫ��
//euler_angle������ŷ����
//q1�������Ԫ��
// int Conversion_Euler_to_Quaternion(Quat_t q1, Euler_t euler_angle)
// {
// 	euler_angle.Yaw = euler_angle.Yaw *  M_PI / 180;
// 	euler_angle.Pitch = euler_angle.Pitch * M_PI / 180;
// 	euler_angle.Roll = euler_angle.Roll * M_PI / 180;
// 	double c1 = acos(euler_angle.Yaw / 2);
// 	double s1 = asin(euler_angle.Yaw / 2);
// 	double c2 = acos(euler_angle.Pitch / 2);
// 	double s2 = asin(euler_angle.Pitch / 2);
// 	double c3 = acos(euler_angle.Roll / 2);
// 	double s3 = asin(euler_angle.Roll / 2);
// 	double c1c2 = c1 * c2;
// 	double s1s2 = s1 * s2;
// 	q1.w = (c1c2 * c3 + s1s2 * s3);
// 	q1.x = (c1c2 * s3 + s1s2 * c3);
// 	q1.y = (s1 * c2 * c3 + c1 * s2 * s3);
// 	q1.z = (c1 * s2 * c3 - s1 * c2 * s3);
// 	return 0;
// }

// //��Ԫ��תŷ����
// //quat:������Ԫ��
// //euler:���ŷ����
// int Conversion_Quaternion_to_Euler(Quat_t *quat, Euler_t *euler)
// {
// 	double q0, q1, q2, q3;
// 	q0 = quat->w;
// 	q1 = quat->x;
// 	q2 = quat->y;
// 	q3 = quat->z;
// 	euler->Pitch = (float)(asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3); // pitch
// 	euler->Roll = (float)(atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3); // roll
// 	euler->Yaw = (float)(atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3);
// 	return 0;
// }

// //��Ԫ����ת
// //quat1:����ԭʼ��Ԫ��
// //quat2:������ת������Ԫ��
// //���أ������ת�����Ԫ��
// int quat_pro(Quat_t *quat1, Quat_t *quat2, Quat_t *quat3)
// {
// 	float w1, x1, y1, z1;
// 	float w2, x2, y2, z2;
// 	w2 = quat1->w;
// 	x2 = quat1->x;
// 	y2 = quat1->y;
// 	z2 = quat1->z;

// 	w1 = quat2->w;
// 	x1 = quat2->x;
// 	y1 = quat2->y;
// 	z1 = quat2->z;

// 	quat3->w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
// 	quat3->x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
// 	quat3->y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
// 	quat3->z = w1*z2 + x1*y2 - y1*x2 + z1*w2;
// 	return 0;
// }



