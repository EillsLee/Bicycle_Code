#include "headfile.h"
#include "outputdata.h"

int main(void)    
{    
    DisableGlobalIRQ();    
    board_init();//务必保留，本函数用于初始化MPU 时钟 调试串口    

	delay_ms(300);
	Init_System(); 
    pit_timer_ms(TIM_2, 1);

    delay_ms(50);       //初始化完成后延时一定的时间  
    //总中断最后开启    
    EnableGlobalIRQ();

    while (1)    

    {        		
        OutPut_Data(mpu_acc_z, mpu_gyro_y, (int)angle_filted, 0); //发送数据到上位机
        delay_ms(5);//延时5ms
    }    

}