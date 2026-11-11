#include "MotorTask.hpp"
#include "ChassisController.hpp"
#include "RobotContext.hpp"
#include "can_device.hpp"

/*
 * 注册 CAN 接收回调。
 */
void MotorInit(void)
{
    static auto& can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    static auto& can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    can1.register_rx_callback([](const HAL::CAN::Frame& frame)
    {
        Motor3508.Parse(frame);
        RightDmMotors.Parse(frame);
    });

    can2.register_rx_callback([](const HAL::CAN::Frame& frame)
    {
        LeftDmMotors.Parse(frame);
    });
}

/*
 * 下发左右轮电机 CAN 指令。
 */
static void motor_control_logic(uint32_t tick)
{
    (void)tick;

    for (int i = 0; i < 2; i++)
    {
        Motor3508.setCAN(static_cast<int16_t>(chassis_output.out_wheel[i]), i + 1);
    }

    Motor3508.sendCAN();
}

/* 每轮在两个关节间轮换；两条 CAN 总线各发送一台关节。 */
static void dm_motor_control_logic()
{
    auto control_motor = [](auto& motors, uint8_t id, uint8_t command_index)
    {
        if (!dm_motor_enable_request)
        {
            motors.Off(id, BSP::Motor::DM::Model::MIT);
            return;
        }

        const uint8_t error_code = motors.getErrorCode(id);
        if (error_code == 0x0D)
        {
            motors.ClearErr(id, BSP::Motor::DM::Model::MIT);
        }
        else if (dm_motor_strategy_request)
        {
            const auto& command = Cboard.board_rx_;
            /* 上行时右腿关节用了负号映射到RL坐标；下发期望角时做逆变换。 */
            const float position_target = (command_index == 1 || command_index == 3)
                ? -command.leg_position_target[command_index]
                : command.leg_position_target[command_index];
            motors.ctrl_MitZeroed(id,
                                  position_target,
                                  0.0f,
                                  chassis_controller.GetJointKp(command_index),
                                  chassis_controller.GetJointKd(command_index),
                                  0.0f);
        }
        else
        {
            motors.On(id, BSP::Motor::DM::Model::MIT);
        }
    };

    /* 按 JointCalibration.hpp 中确认的实际位置映射：
       RightDm(1)=左前、RightDm(2)=左后、LeftDm(2)=右前、LeftDm(1)=右后。
       每轮都更新四台电机，避免轮换周期导致单台命令刷新率减半。 */
    control_motor(RightDmMotors, 1, 0);
    control_motor(RightDmMotors, 2, 2);
    control_motor(LeftDmMotors, 2, 1);
    control_motor(LeftDmMotors, 1, 3);
}

extern "C" {
void CanTask(void const* argument)
{
    (void)argument;

    MotorInit();

    static uint32_t loop_count = 0;
    for (;;)
    {
        loop_count++;
        motor_control_logic(loop_count);
        dm_motor_control_logic();

        osDelay(2);
    }
}
}
