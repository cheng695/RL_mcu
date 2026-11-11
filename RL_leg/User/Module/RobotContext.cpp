#include "RobotContext.hpp"

namespace
{
constexpr float PI_VALUE = 3.1415926535897932384626433832795f;
} /* namespace */

// 遥控器
BSP::REMOTE_CONTROL::RemoteController DT7;

// IMU
BSP::IMU::HI12_float HI12;

// 电机
BSP::Motor::Dji::GM3508<2> Motor3508(0x200, {1, 2}, 0x200);

// 达妙关节
BSP::Motor::DM::J8009p<2> RightDmMotors(0x000, {1, 3}, {0x002, 0x004}, HAL::CAN::CanDeviceId::HAL_Can1);
BSP::Motor::DM::J8009p<2> LeftDmMotors(0x000, {5, 7}, {0x006, 0x008}, HAL::CAN::CanDeviceId::HAL_Can2);
volatile bool dm_motor_enable_request = false;
volatile bool dm_motor_strategy_request = false;

// 板间通讯
BoardCommunication Cboard;

// 输入
OperatorInput operator_input;

// 底盘状态机
Chassis_FSM chassis_fsm;

// 底盘目标和输出
ChassisTarget chassis_target;
ChassisOutput chassis_output;

