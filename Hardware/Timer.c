#include "Timer.h"

volatile int oledupdate_state;
// volatile int TurningUpdate_state;
// volatile int TurningDowndate_state;
volatile int date_state;
uint32_t Shoot_CurrentTime;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM3) // 检查定时器更新中断标志
    {
        
        static int32_t last_count = 0;
        last_count++;
        if (last_count % 10 == 0)
        {
            oledupdate_state = 1;
        }
        if (last_count % 25 == 0)
        {
            date_state = 1;
        }
        // if(last_count % 1500 == 0)
        // {
        //     Shoot_CurrentTime++;
        // }
        Shoot_CurrentTime = last_count;
    }

}