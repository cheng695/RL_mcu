#pragma once

#include "ChassisStateMachine.hpp"
#include "ChassisTypes.hpp"
#include "CommunicationTask.hpp"
#include "DT7.hpp"
#include "DjiMotor.hpp"
#include "DmMotor.hpp"
#include "OmniCalculation.hpp"
#include "OperatorInput.hpp"
#include "PowerControl.hpp"

/*
 * 跨任务共享对象集中声明。
 *
 * Task 层只负责调度，不再通过互相 include 对方的 Task 头文件来借用全局对象。
 */
extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BSP::IMU::HI12_float HI12;
extern BSP::Motor::Dji::GM3508<2> Motor3508;
extern BSP::Motor::DM::J8009p<2> RightDmMotors;
extern BSP::Motor::DM::J8009p<2> LeftDmMotors;
extern volatile bool dm_motor_enable_request;

extern BoardCommunication Cboard;

extern OperatorInput operator_input;

extern Chassis_FSM chassis_fsm;
extern ChassisTarget chassis_target;
extern ChassisOutput chassis_output;

