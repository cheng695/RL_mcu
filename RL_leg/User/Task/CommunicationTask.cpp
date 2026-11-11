#include "CommunicationTask.hpp"
#include "RobotContext.hpp"
#include "VmcVelocityEstimator.hpp"
#include "uart_device.hpp"
#include <cstdint>
#include <cmath>

/* 通讯数据缓存 ---------------------------------------------------------------------------------------------*/
uint8_t BoardRx_buffer[BoardCommunication::RX_FRAME_SIZE];
uint8_t BoardTx_buffer[BoardCommunication::TX_FRAME_SIZE];
uint8_t send_str2[sizeof(float) * 10];

uint8_t DT7Rx_buffer[18];                  
uint8_t HI12Rx_buffer[82];

/* 板间通讯 ------------------------------------------------------------------------------------------------*/
/**
 * @brief 板件通讯串口接收回调与数据解�?
 *
 */
void BoardCommunicationInit()
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);

    HAL::UART::Data uart1_rx_buffer{DT7Rx_buffer, sizeof(DT7Rx_buffer)};
    HAL::UART::Data uart6_rx_buffer{HI12Rx_buffer, sizeof(HI12Rx_buffer)};
    HAL::UART::Data uart7_rx_buffer{BoardRx_buffer, sizeof(BoardRx_buffer)};

    uart1.receive_dma_idle(uart1_rx_buffer);
    uart6.receive_dma_idle(uart6_rx_buffer);
    uart7.receive_dma_idle(uart7_rx_buffer);

    uart1.register_rx_callback([](const HAL::UART::Data &data)
    {
        if (data.buffer != nullptr && data.size == sizeof(DT7Rx_buffer))
        {
            DT7.parseData(data.buffer); 
        }
    });

    uart6.register_rx_callback([](const HAL::UART::Data &data)
    {
        if (data.buffer != nullptr && data.size == sizeof(HI12Rx_buffer))
        {
            HI12.DataUpdate(data.buffer);
        }
    });

    uart7.register_rx_callback([](const HAL::UART::Data &data)
    {
        if (data.buffer != nullptr && data.size > 0)
        {
            Cboard.parseRX(data.buffer, data.size);
        }
    });
}

/**
 * @brief vofa
 *
 */
void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6, float x7, float x8, float x9)
{
    const float channels[9] = {x1, x2, x3, x4, x5, x6, x7, x8, x9};
    const uint8_t sendSize = sizeof(float); // 单浮点数�?字节

    // �?个浮点数据写入缓冲区（小端模式）
    memcpy(send_str2, channels, sendSize * 9);

    // 写入帧尾（协议要�?0x00 0x00 0x80 0x7F�?
    *((uint32_t*)&send_str2[sizeof(float) * 9]) = 0x7F800000; // 小端存储�?00 00 80 7F
}

void UART_TX()
{
    // 板间通讯
    auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);
    const float deg_to_rad = 0.017453292519943295f;

    /* 关节顺序统一为左前、右前、左后、右后。 */
    Cboard.board_tx_.leg_position[0] = RightDmMotors.getAngleRadZeroed(1); //左
    Cboard.board_tx_.leg_position[1] = -LeftDmMotors.getAngleRadZeroed(2);
    Cboard.board_tx_.leg_position[2] = RightDmMotors.getAngleRadZeroed(2);  //左
    Cboard.board_tx_.leg_position[3] = -LeftDmMotors.getAngleRadZeroed(1);

    Cboard.board_tx_.leg_velocity[0] = RightDmMotors.getVelocityRads(2);  //左
    Cboard.board_tx_.leg_velocity[1] = -LeftDmMotors.getVelocityRads(2);
    Cboard.board_tx_.leg_velocity[2] = RightDmMotors.getVelocityRads(1);  //左
    Cboard.board_tx_.leg_velocity[3] = -LeftDmMotors.getVelocityRads(1);

    Cboard.board_tx_.wheel_velocity[0] = -Motor3508.getVelocityRads(1);
    Cboard.board_tx_.wheel_velocity[1] = Motor3508.getVelocityRads(2);

    Cboard.board_tx_.imu_gyro[0] = HI12.GetGyro(1) * deg_to_rad;
    Cboard.board_tx_.imu_gyro[1] = HI12.GetGyro(0) * deg_to_rad;
    Cboard.board_tx_.imu_gyro[2] = HI12.GetGyro(2) * deg_to_rad;

    /* HI12提供wxyz；按当前安装与上位机坐标约定交换x/y，z轴和w保持不变。 */
    Cboard.board_tx_.imu_quaternion[0] = HI12.GetQuaternion(2);
    Cboard.board_tx_.imu_quaternion[1] = HI12.GetQuaternion(1);
    Cboard.board_tx_.imu_quaternion[2] = HI12.GetQuaternion(3);
    Cboard.board_tx_.imu_quaternion[3] = HI12.GetQuaternion(0);

    /* vx估计使用参考VMC项目的原始编码器零点、方向和机构正运动学。
       ID映射：左后/左前=CAN2 ID5/7，右前/右后=CAN1 ID1/3；
       左右轮=CAN1 ID2/1。这里刻意使用getAngleRad()原始角度，不使用
       当前项目JointCalibration的零偏，也不使用上行遥测的方向变换。 */
    Cboard.board_tx_.base_forward_velocity = VmcVelocityEstimator::Estimate(
        LeftDmMotors.getAngleRad(1), LeftDmMotors.getAngleRad(2),
        RightDmMotors.getAngleRad(1), RightDmMotors.getAngleRad(2),
        LeftDmMotors.getVelocityRads(1), LeftDmMotors.getVelocityRads(2),
        RightDmMotors.getVelocityRads(1), RightDmMotors.getVelocityRads(2),
        Motor3508.getVelocityRads(2), Motor3508.getVelocityRads(1),
        HI12.GetAngle(1) * deg_to_rad, HI12.GetGyro(0) * deg_to_rad);
    // status_flags bit 0 means base_forward_velocity is valid; it must not
    // depend on the chassis DISABLED / MOTOR_ENABLE / CONTROL state.
    Cboard.board_tx_.status_flags =
        std::isfinite(Cboard.board_tx_.base_forward_velocity) ? 0x01U : 0x00U;

    Cboard.prepareTXFrame();
    memcpy(BoardTx_buffer, &Cboard.board_tx_, sizeof(Cboard.board_tx_));
    HAL::UART::Data uart7_tx_buffer{BoardTx_buffer, sizeof(BoardTx_buffer)};
    uart7.transmit_dma(uart7_tx_buffer);
}

/* 只从 DT7 更新摇杆、拨轮和左右拨杆，不读取遥控器键盘按键。 */
void UpdateMomentaryInput(OperatorInput &input)
{
    DT7.SetDeadzone(20.0f);

    input.left_switch = DT7.get_s1();
    input.right_switch = DT7.get_s2();

    /* 左摇杆上下控制前后，右摇杆左右控制yaw，滚轮用于高度增量。 */
    input.move_x = DT7.get_left_x();
    input.move_y = DT7.get_left_y();
    input.rotate = DT7.get_right_x();
    input.height_increment = DT7.get_scroll_();
}


extern "C" {
void UartTask(void const * argument)
{
    BoardCommunicationInit();
    for(;;)
    {
        UpdateMomentaryInput(operator_input);
        UART_TX();
        osDelay(10);
    }
}

}
