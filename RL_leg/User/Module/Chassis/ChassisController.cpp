#include "ChassisController.hpp"

#include "ChassisEventDetector.hpp"
#include "ChassisStateMachine.hpp"
#include "ChassisTypes.hpp"
#include "JointCalibration.hpp"
#include "RobotContext.hpp"
#include "DT7.hpp"
#include "pid.hpp"

#include <algorithm>
#include <cmath>

namespace
{
constexpr float REMOTE_MAX_VX = 3.0f;          // m/s，对应摇杆满量程
constexpr float REMOTE_MAX_YAW_RATE = 10.0f;   // rad/s，对应摇杆满量程
constexpr float HEIGHT_CHANGE_RATE = 0.10f;    // m/s，滚轮满量程时的高度变化速度
constexpr float HEIGHT_MIN = 0.20f;
constexpr float HEIGHT_MAX = 0.37f;
constexpr float HEIGHT_INITIAL = (HEIGHT_MIN + HEIGHT_MAX) * 0.5f;
constexpr float JOINT_MIT_KP = 60.0f;
constexpr float JOINT_MIT_KD = 2.0f;

/* 沿用工程中原有的3508速度环参数，输出单位为电调电流指令。 */
ALG::PID::PID wheel_pid[2] = {
    ALG::PID::PID(700.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(700.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f)};

bool IsValidCommand(const BoardCommunication::BoardRX& command)
{
    const float raw_joint_targets[4] = {
        command.leg_position_target[0] + LeftDmMotors.getAngleOffset(2),
        command.leg_position_target[1] + RightDmMotors.getAngleOffset(1),
        command.leg_position_target[2] + LeftDmMotors.getAngleOffset(1),
        command.leg_position_target[3] + RightDmMotors.getAngleOffset(2)};

    for (float target : raw_joint_targets)
    {
        if (!std::isfinite(target) || target < -12.5f || target > 12.5f)
        {
            return false;
        }
    }

    for (float target : command.wheel_velocity_target)
    {
        if (!std::isfinite(target))
            return false;
    }

    return true;
}

void StopControllerOutput()
{
    dm_motor_enable_request = false;
    for (float& output : chassis_output.out_wheel)
    {
        output = 0.0f;
    }
    wheel_pid[0].reset();
    wheel_pid[1].reset();
}
} // namespace

ChassisController chassis_controller;

void ChassisController::Init()
{
    chassis_fsm.Init();
    height_target_m_ = HEIGHT_INITIAL;
    last_update_tick_ = HAL_GetTick();

    /* 协议关节顺序：左前、右前、左后、右后。 */
    LeftDmMotors.setAngleOffset(2, JointCalibration::LEFT_FRONT_OFFSET_RAD);
    RightDmMotors.setAngleOffset(1, JointCalibration::RIGHT_FRONT_OFFSET_RAD);
    LeftDmMotors.setAngleOffset(1, JointCalibration::LEFT_REAR_OFFSET_RAD);
    RightDmMotors.setAngleOffset(2, JointCalibration::RIGHT_REAR_OFFSET_RAD);

    for (uint8_t i = 0; i < 4; ++i)
    {
        SetJointGains(i, JOINT_MIT_KP, JOINT_MIT_KD);
    }
    StopControllerOutput();
    Cboard.board_tx_.remote_command.status = 0;
    Cboard.board_tx_.remote_command.vx = 0.0f;
    Cboard.board_tx_.remote_command.yaw_rate = 0.0f;
    Cboard.board_tx_.remote_command.height = height_target_m_;
}

bool ChassisController::SetJointGains(uint8_t index, float kp, float kd)
{
    if (index >= 4 || !std::isfinite(kp) || !std::isfinite(kd) ||
        kp <= 0.0f || kp > 500.0f || kd < 0.0f || kd > 5.0f)
    {
        return false;
    }

    joint_kp_[index] = kp;
    joint_kd_[index] = kd;
    joint_gains_configured_ = true;
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (joint_kp_[i] <= 0.0f)
        {
            joint_gains_configured_ = false;
        }
    }
    return true;
}

bool ChassisController::AreJointGainsConfigured() const
{
    return joint_gains_configured_;
}

float ChassisController::GetJointKp(uint8_t index) const
{
    return index < 4 ? joint_kp_[index] : 0.0f;
}

float ChassisController::GetJointKd(uint8_t index) const
{
    return index < 4 ? joint_kd_[index] : 0.0f;
}

void ChassisController::Update(const OperatorInput& input)
{
    const uint32_t now = HAL_GetTick();
    float dt = static_cast<float>(now - last_update_tick_) * 0.001f;
    last_update_tick_ = now;
    dt = std::clamp(dt, 0.0f, 0.02f);

    const bool remote_online = DT7.isConnected();
    const ChassisEvent event = DetectChassisEvent(input, remote_online);
    chassis_fsm.Dispatch(event);
    chassis_fsm.TIM_Update();

    const bool control_state = chassis_fsm.Get_Now_State() == CONTROL;
    if (control_state)
    {
        height_target_m_ += input.height_increment * HEIGHT_CHANGE_RATE * dt;
        height_target_m_ = std::clamp(height_target_m_, HEIGHT_MIN, HEIGHT_MAX);

        Cboard.board_tx_.remote_command.status = 1;
        Cboard.board_tx_.remote_command.vx = input.move_y * REMOTE_MAX_VX;
        /* 右摇杆向左为正yaw，按常用Z轴逆时针为正的约定取反。 */
        Cboard.board_tx_.remote_command.yaw_rate = -input.rotate * REMOTE_MAX_YAW_RATE;
    }
    else
    {
        Cboard.board_tx_.remote_command.status = 0;
        Cboard.board_tx_.remote_command.vx = 0.0f;
        Cboard.board_tx_.remote_command.yaw_rate = 0.0f;
    }
    Cboard.board_tx_.remote_command.height = height_target_m_;
    Cboard.board_tx_.status_flags = control_state ? 0x01 : 0x00;

    const BoardCommunication::BoardRX& command = Cboard.board_rx_;
    if (!control_state || !Cboard.isConnected() || command.control_enable != 1 ||
        !joint_gains_configured_ || !IsValidCommand(command))
    {
        StopControllerOutput();
        return;
    }

    dm_motor_enable_request = true;
    for (uint8_t i = 0; i < 2; ++i)
    {
        const float output = wheel_pid[i].UpDate(command.wheel_velocity_target[i],
                                                  Motor3508.getVelocityRads(i + 1));
        chassis_output.out_wheel[i] = std::clamp(output, -16384.0f, 16384.0f);
    }
    chassis_output.out_wheel[2] = 0.0f;
    chassis_output.out_wheel[3] = 0.0f;
}

void InitChassisControl()
{
    chassis_controller.Init();
}

void ChassisControlLoop(const OperatorInput& input)
{
    chassis_controller.Update(input);
}
