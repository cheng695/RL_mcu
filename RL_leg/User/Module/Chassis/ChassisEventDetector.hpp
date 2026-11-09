#pragma once

#include "OperatorInput.hpp"
#include "ChassisStateMachine.hpp"

/* 左下右下急停；左下右中进入控制；其他组合或遥控器离线时停止。 */
ChassisEvent ResolveChassisEvent(const OperatorInput& input, bool equipment_online);
ChassisEvent DetectChassisEvent(const OperatorInput& input, bool equipment_online);
