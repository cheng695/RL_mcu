#include "ChassisPlanner.hpp"

/*
 * 旧的键鼠/四轮全向底盘规划器不参与 RL 2real 执行链路。
 * 保留空实现以兼容仍引用这些符号的旧模块；实际目标来自上位机协议帧，
 * 由 ChassisController 直接跟踪。
 */
void InitChassisPlanner()
{
}

ChassisTarget PlanChassisTarget(Enum_Chassis_States state, const OperatorInput& input)
{
    (void)state;
    (void)input;
    return {};
}
