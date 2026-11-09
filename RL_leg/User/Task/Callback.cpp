#include "MotorTask.hpp"
#include "CommunicationTask.hpp"
#include "RobotContext.hpp"

/* CAN回调 ---------------------------------------------------------------------------------------------*/
// fifo0
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;
    
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    if (hcan == can1.get_handle())
    {
        can1.receive(rx_frame);  // receive()内部会自动触发所有注册的回调
    }
}

// fifo1
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;
    
    auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    if (hcan == can2.get_handle())
    {
        can2.receive(rx_frame);  // receive()内部会自动触发所有注册的回调
    }
}

/* 串口回调 ---------------------------------------------------------------------------------------------*/
extern "C" 
{
    // 处理DMA接收中的空闲线路检测
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
    {
        // 遥控器
        if(huart->Instance == USART1)
        {
            HAL::UART::Data uart1_rx_buffer{DT7Rx_buffer, sizeof(DT7Rx_buffer)};
            auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
            
            if(huart == uart1.get_handle())
            {
                uart1.receive_dma_idle(uart1_rx_buffer);
                uart1.trigger_rx_callbacks(uart1_rx_buffer);
            }
        }
        // IMU
        else if (huart->Instance == USART6) 
        {
            HAL::UART::Data uart6_rx_buffer{HI12Rx_buffer, Size};
            auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
            
            if(huart == uart6.get_handle())
            {
                uart6.receive_dma_idle(uart6_rx_buffer);
                uart6.trigger_rx_callbacks(uart6_rx_buffer);
            }
        }
        // 上下板
        else if(huart->Instance == UART7)
        {
            HAL::UART::Data uart7_rx_buffer{BoardRx_buffer, Size};
            auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);
            
            if(huart == uart7.get_handle())
            {
                uart7.trigger_rx_callbacks(uart7_rx_buffer);

                HAL::UART::Data uart7_dma_buffer{BoardRx_buffer, sizeof(BoardRx_buffer)};
                uart7.receive_dma_idle(uart7_dma_buffer);
            }
        }
    }
}
