#include "headfile.h"

void Inductor_init()
{
	adc_init(ADC_P00,ADC_SYSclk_DIV_2);
	adc_init(ADC_P01,ADC_SYSclk_DIV_2);
	adc_init(ADC_P05,ADC_SYSclk_DIV_2);
	adc_init(ADC_P06,ADC_SYSclk_DIV_2);		// ADC channel initialize.
}
// void Get_Inductor(uint16 Indc_val[])
// {

// }