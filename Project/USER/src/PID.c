#include "headfile.h"
#include "common.h"
#include "PID.h"
struct PID_strc
{
    float P;
    float I;
    float D;
    float P_out;
    float I_out;
    float D_out;
    float ek, ek1, ek2;
    float OUT_LAST;
    float integral;

}PID;

void Steereo_Ctrl(float deviation)
{
	static float pre_dev;
    PID.P = 75.0f;
    PID.D = 0.0f;
    PID.I = 0.002f;

    PID.integral += deviation;
    

    PID.P_out = deviation * PID.P;
    PID.D_out = PID.D * (pre_dev - deviation);
    PID.I_out = PID.integral *  PID.I;

    PID.OUT_LAST = Stereo_MID + PID.P_out - PID.D_out;//(float)(PID.D_out - PID.P_out)// + PID.I_out);

            /*******限幅保护*****/
            if(PID.OUT_LAST > 9000)
                PID.OUT_LAST = 9000;
            else if(PID.OUT_LAST < 6000)
                PID.OUT_LAST = 6000;
    /*****更新pre_dev*****/
	pre_dev = deviation;

    pwm_duty(PWMB_CH1_P74, PID.OUT_LAST);
}