/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		MPU6050
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		MDK FOR C251 V5.60
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-04-30
 * @note		
					接线定义：
					------------------------------------ 
						软件IIC
                        SCL                 查看SEEKFREE_IIC文件内的SEEKFREE_SCL宏定义
						SDA                 查看SEEKFREE_IIC文件内的SEEKFREE_SDA宏定义
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
//  @brief      MPU6050自检函数
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_self1_check(void)
{
    simiic_write_reg(MPU6050_DEV_ADDR, PWR_MGMT_1, 0x00);	//解除休眠状态
    simiic_write_reg(MPU6050_DEV_ADDR, SMPLRT_DIV, 0x07);   //125HZ采样率
    while(0x07 != simiic_read_reg(MPU6050_DEV_ADDR, SMPLRT_DIV,SIMIIC))
    {
		delay_ms(1);
        //卡在这里原因有以下几点
        //1 MPU6050坏了，如果是新的这样的概率极低
        //2 接线错误或者没有接好
        //3 可能你需要外接上拉电阻，上拉到3.3V
		//4 可能没有调用模拟IIC的初始化函数
    }
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化MPU6050
//  @param      NULL
//  @return     void					
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_init(void)
{
    delay_ms(100);                                   //上电延时

    mpu6050_self1_check();
    simiic_write_reg(MPU6050_DEV_ADDR, PWR_MGMT_1, 0x00);	//解除休眠状态
    simiic_write_reg(MPU6050_DEV_ADDR, SMPLRT_DIV, 0x07);   //125HZ采样率
    simiic_write_reg(MPU6050_DEV_ADDR, MPU6050_CONFIG, 0x04);       //
    simiic_write_reg(MPU6050_DEV_ADDR, GYRO_CONFIG, 0x18);  //2000
    simiic_write_reg(MPU6050_DEV_ADDR, ACCEL_CONFIG, 0x10); //8g
	simiic_write_reg(MPU6050_DEV_ADDR, User_Control, 0x00);
    simiic_write_reg(MPU6050_DEV_ADDR, INT_PIN_CFG, 0x02);
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取MPU6050加速度计数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
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
//  @brief      获取MPU6050陀螺仪数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
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
//  @brief      获取MPU6050陀螺仪&&加速度计数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:				执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void MPU6050_get_GyandAcc(void)
{
    mpu6050_get_accdata();
    mpu6050_get_gyro();
}

//get accel and gyro from iam20609 
// 对accel一阶低通滤波(参考匿名)，对gyro转成弧度每秒(2000dps)
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
float Q_angle=0.001; //陀螺仪噪声协方差
float Q_gyro=0.003;  //陀螺仪漂移噪声协方差
float R_angle=0.03;   //加速度计测量噪声协方差
float dt=0.001;
float PCt_0=0, PCt_1=0, E=0;	//中间变量
char C_0=1;			 //H矩阵的系数
float Q_bias=0,Angle_err=0;
float K_0=0,K_1=0,t_0=0,t_1=0; //卡尔曼增益
float Pdot[4] ={0,0,0,0};	   //P矩阵中间变量
float PP[2][2] = { { 1, 0 },{ 0, 1 } };	  //P矩阵  X的协方差

 
float KalmanFilter(float Accel,float Gyro)		
{
    static float Angle = 0, Gyro_y = 0;
	Angle+=(Gyro - Q_bias) * dt; //先验估计
 
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
 
	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3]= Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // 先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	
	PCt_0 = C_0 * PP[0][0];		//a
	PCt_1 = C_0 * PP[1][0];		//c
	E = R_angle + C_0 * PCt_0;	//分母
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	Angle_err = Accel - Angle;	//zk-先验估计	
	Angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	Gyro_y   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];
 
	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	return Angle;
}
// #define delta_T      0.005f  //5ms计算一次
 
// float I_ex, I_ey, I_ez;  // 误差积分
// quaterInfo_t Q_info;  // 全局四元数
// eulerianAngles_t eulerAngle; //欧拉角
// float param_Kp = 50.0;   // 加速度计(磁力计)的收敛速率比例增益50 
// float param_Ki = 0.20;   //陀螺仪收敛速率的积分增益 0.2
typedef struct
{
	float w, x, y, z;
}Quat_t;//四元素结构体
// w = cos(theta/2)    
// x  = ax * sin(theta/2)    
// y  = ay * sin(theta/2)    
// z  = az * sin(theta/2)  


typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
}Euler_t;//欧拉角结构体


//欧拉角转四元素
//euler_angle：输入欧拉角
//q1：输出四元素
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

// //四元素转欧拉角
// //quat:输入四元素
// //euler:输出欧拉角
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

// //四元素旋转
// //quat1:输入原始四元素
// //quat2:输入旋转向量四元素
// //返回：输出旋转后的四元素
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



