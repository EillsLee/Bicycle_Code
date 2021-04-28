#include "headfile.h"

/*
// @brief  System initialize.
// @para   NULL
// @return void
// @sample 
*/
void Init_System(void)
{
    Init_Para();    //Parameters done!
    Init_Drivers(); //Drivers done!
}



/*
// @brief  Parameters initialize.
// @para   NULL
// @return void
// @sample 
*/
void Init_Para()
{
    
}



/*
// @brief  Drivers initialize.
// @para   NULL
// @return void
// @sample 
*/
void Init_Drivers()
{
    //OLED initialize.
    oled_init();
    
    //Inductor initialize.
    Inductor_init();

    //mpu6050 initialize.
    //icm20602_init_simiic();
    mpu6050_init();
    // MPU6050_Offset();

    //Stero initialize.
    pwm_init(PWMB_CH1_P74, 80, Stereo_MID);     //初始化PWMB  使用引脚P7.4  输出PWM频率80HZ 中值7500.
    pwm_init(PWMA_CH1P_P60, 10000, 10000);
	pwm_init(PWMA_CH2P_P62, 10000, 0);
}

void QuikSort(float arr[], int low, int high) reentrant 
{
    if(low < high) {
        int standard = getStandard(arr, low, high);
        //递归调用
        QuikSort(arr, low, standard - 1);
        QuikSort(arr, standard + 1, high);
    }
}

int getStandard(int arr[], int i, int j) 
{
    int key = arr[i];
    while (i < j)
    {
        // 因为默认基准是从左边开始，所以从右边开始比较
        // 当队尾的元素大于等于基准数据 时,就一直向前挪动 j 指针
        while (i < j && arr[j] >= key) {
            j--;
        }
        // 当找到比 arr[i] 小的时，就把后面的值 arr[j] 赋给它
        if (i < j) {
            arr[i] = arr[j];
        }
        // 当队首元素小于等于基准数据 时,就一直向后挪动 i 指针
        while (i < j && arr[i] <= key) {
            i++;
        }
        // 当找到比 arr[j] 大的时，就把前面的值 arr[i] 赋给它
        if (i < j) {
            arr[j] = arr[i];
        }
    }
    arr[i] = key;
    return i;
    
}