#include "ChassisEventDetector.hpp"

#include "DT7.hpp"

namespace
{
const uint8_t DOWN = BSP::REMOTE_CONTROL::RemoteController::DOWN;
const uint8_t MIDDLE = BSP::REMOTE_CONTROL::RemoteController::MIDDLE;
} // namespace

ChassisEvent ResolveChassisEvent(const OperatorInput& input, bool equipment_online)
{
    if (!equipment_online)
    {
        return ChassisEvent::StopRequested;
    }

    if (input.left_switch == DOWN && input.right_switch == DOWN)
    {
        return ChassisEvent::EmergencyStopRequested;
    }

    if (input.left_switch == DOWN && input.right_switch == MIDDLE)
    {
        return ChassisEvent::StopRequested;
    }

    if (input.left_switch == MIDDLE && input.right_switch == DOWN)
    {
        return ChassisEvent::MotorEnableRequested;
    }

    if (input.left_switch == MIDDLE && input.right_switch == MIDDLE)
    {
        return ChassisEvent::ControlRequested;
    }

    /* 未指定的拨杆组合一律停止，避免误进入控制状态。 */
    return ChassisEvent::StopRequested;
}

ChassisEvent DetectChassisEvent(const OperatorInput& input, bool equipment_online)
{
    static uint8_t last_left = 0xFF;
    static uint8_t last_right = 0xFF;
    static bool last_online = false;

    if (equipment_online == last_online &&
        input.left_switch == last_left &&
        input.right_switch == last_right)
    {
        return ChassisEvent::None;
    }

    last_online = equipment_online;
    last_left = input.left_switch;
    last_right = input.right_switch;
    return ResolveChassisEvent(input, equipment_online);
}
