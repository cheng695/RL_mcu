#pragma once

#include "OperatorInput.hpp"
#include "ChassisStateMachine.hpp"

/* 左下右下急停；左中右下仅使能；左中右中进入策略控制；其他组合或遥控器离线时停止。 */
ChassisEvent ResolveChassisEvent(const OperatorInput& input, bool equipment_online);
ChassisEvent DetectChassisEvent(const OperatorInput& input, bool equipment_online);
