#include "Timer.h"

volatile int oledupdate_state;
volatile int TurningUpdate_state;
volatile int TurningDowndate_state;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM3) // 检查定时器更新中断标志
    {
        
        static int32_t last_count = 0;
        last_count++;
        if (last_count % 10 == 0)
        {
            oledupdate_state = 1;

            // Emm_V5_GetCurrentLocation(&Usart2, 0x02);
            // Emm_V5_GetCurrentLocation(&Usart3, 0x01);
        }
        if (last_count % 25 == 0)
        {
            TurningUpdate_state = 1;
            TurningDowndate_state = 1;
        }
    }

}