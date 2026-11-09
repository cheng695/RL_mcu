#include "ControlTask.hpp"
#include "ChassisController.hpp"
#include "RobotContext.hpp"
#include "buzzer_manager.hpp"

/*
 * 兼容旧接口：初始化底盘控制模块。
 */

void fsm_init()
{
    InitChassisControl();
}

extern "C" {
void Control(void const *argument)
{
    (void)argument;

    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    InitChassisControl();

    for (;;)
    {
        ChassisControlLoop(operator_input);
        osDelay(1);
    }
}

/* Compatibility entry point used by the CubeMX-generated FreeRTOS setup. */
void AlgTask(void const *argument)
{
    Control(argument);
}

}
