#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "buzzer_manager.hpp"
#include "can_hal.hpp"
#include "DT7.hpp"
#include "HI12_imu.hpp"
#include "state_watch.hpp"
#include "uart_hal.hpp"
#include <cstddef>
#include <cstdint>
#include <cstring>

// ──── 板间通讯协议 ──────────────────────────────────────────────────
class BoardCommunication
{
    public:
        static constexpr uint8_t FRAME_HEADER[2] = {0xA8, 0xE6};
        static constexpr uint8_t FRAME_TAIL[2] = {0xC3, 0xF7};
        static constexpr size_t TX_FRAME_SIZE = 94;
        static constexpr size_t RX_FRAME_SIZE = 33;

        #pragma pack(push, 1)
        // 发送：底盘→上位机
        struct BoardTX
        {
            uint8_t header[2];
            uint16_t frame_seq;
            float leg_position[4];       // 左前、右前、左后、右后，rad
            float leg_velocity[4];       // rad/s
            float wheel_velocity[2];     // 左轮、右轮，rad/s
            float imu_gyro[3];            // base坐标系，rad/s
            float imu_quaternion[4];      // x、y、z、w
            float base_forward_velocity;  // base坐标系，m/s
            uint8_t status_flags;
            struct
            {
                uint8_t status;
                float vx;
                float yaw_rate;
                float height;
            } remote_command;
            uint16_t crc16;
            uint8_t tail[2];
        };
        
        // 接收：上位机→底盘
        struct BoardRX
        {
            uint8_t header[2];
            uint16_t frame_seq;
            uint8_t control_enable;       // 0:安全状态，1:允许执行
            float leg_position_target[4]; // 左前、右前、左后、右后，rad
            float wheel_velocity_target[2]; // 左轮、右轮，rad/s
            uint16_t crc16;
            uint8_t tail[2];
        };
        #pragma pack(pop)

        static_assert(sizeof(BoardTX) == TX_FRAME_SIZE, "BoardTX协议帧长度必须为94字节");
        static_assert(sizeof(BoardRX) == RX_FRAME_SIZE, "BoardRX协议帧长度必须为33字节");
        static_assert(offsetof(BoardTX, crc16) == 90, "BoardTX CRC偏移错误");
        static_assert(offsetof(BoardRX, crc16) == 29, "BoardRX CRC偏移错误");

        BoardTX board_tx_;
        BoardRX board_rx_;

    private:
        uint8_t rx_frame_buffer_[RX_FRAME_SIZE] = {};
        size_t rx_frame_length_ = 0;
        uint16_t tx_sequence_ = 0;

        static uint16_t calculateCRC16(const uint8_t *data, size_t length)
        {
            uint16_t crc = 0xFFFF;
            for (size_t i = 0; i < length; ++i)
            {
                crc ^= data[i];
                for (uint8_t bit = 0; bit < 8; ++bit)
                {
                    crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408U)
                                     : static_cast<uint16_t>(crc >> 1);
                }
            }
            return crc;
        }

        bool validateRXFrame(const uint8_t *frame)
        {
            if (frame[0] != FRAME_HEADER[0] || frame[1] != FRAME_HEADER[1] ||
                frame[RX_FRAME_SIZE - 2] != FRAME_TAIL[0] || frame[RX_FRAME_SIZE - 1] != FRAME_TAIL[1])
            {
                return false;
            }

            const uint16_t received_crc = static_cast<uint16_t>(frame[29]) |
                                          (static_cast<uint16_t>(frame[30]) << 8);
            return received_crc == calculateCRC16(frame, 29);
        }

    public:
        /**
         * @brief 构造函�?
         * @param timeThreshold 连接超时阈�?毫秒)，默�?00ms
         */
        BoardCommunication(int timeThreshold = 100) : statewatch_(timeThreshold) 
        {
            memset(&board_tx_, 0, sizeof(board_tx_));
            memset(&board_rx_, 0, sizeof(board_rx_));
            memcpy(board_tx_.header, FRAME_HEADER, sizeof(FRAME_HEADER));
            memcpy(board_tx_.tail, FRAME_TAIL, sizeof(FRAME_TAIL));
        }

        /**
         * @brief 虚析构函�?
         */
        virtual ~BoardCommunication() = default;

        /**
         * @brief 更新时间�?
         * @details 记录最后一次通讯时间，用于判断连接状�?
         */
        void updateTimestamp()
        {
            statewatch_.UpdateLastTime();
        }

        /**
         * @brief 按A8 E6帧头同步并解析上位机命令帧，只有帧头、帧尾和CRC均正确才更新数据。
         */
        void parseRX(const uint8_t *data, size_t length)
        {
            if (data == nullptr)
            {
                return;
            }

            for (size_t i = 0; i < length; ++i)
            {
                const uint8_t byte = data[i];
                if (rx_frame_length_ == 0)
                {
                    if (byte == FRAME_HEADER[0])
                    {
                        rx_frame_buffer_[rx_frame_length_++] = byte;
                    }
                    continue;
                }

                if (rx_frame_length_ == 1 && byte != FRAME_HEADER[1])
                {
                    rx_frame_length_ = (byte == FRAME_HEADER[0]) ? 1 : 0;
                    if (rx_frame_length_ == 1)
                    {
                        rx_frame_buffer_[0] = byte;
                    }
                    continue;
                }

                rx_frame_buffer_[rx_frame_length_++] = byte;
                if (rx_frame_length_ == RX_FRAME_SIZE)
                {
                    if (validateRXFrame(rx_frame_buffer_))
                    {
                        memcpy(&board_rx_, rx_frame_buffer_, sizeof(board_rx_));
                        updateTimestamp();
                    }
                    rx_frame_length_ = 0;
                }
            }
        }

        /**
         * @brief 更新帧头、序号、CRC和帧尾，调用后可直接发送board_tx_。
         */
        void prepareTXFrame()
        {
            memcpy(board_tx_.header, FRAME_HEADER, sizeof(FRAME_HEADER));
            board_tx_.frame_seq = tx_sequence_++;
            memcpy(board_tx_.tail, FRAME_TAIL, sizeof(FRAME_TAIL));
            board_tx_.crc16 = calculateCRC16(reinterpret_cast<const uint8_t *>(&board_tx_), offsetof(BoardTX, crc16));
        }

        /**
         * @brief 检查连接状�?
         * @return true-在线 false-离线
         * @details 检测通讯是否正常，若离线则触发蜂鸣器报警
         */
        bool isConnected()
        {
            statewatch_.UpdateTime();
            statewatch_.CheckStatus();
            if(statewatch_.GetStatus() == BSP::WATCH_STATE::Status::OFFLINE)
            {
                BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestCommunicationRing();
            }
            return statewatch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE;
        }
        
    private:
        BSP::WATCH_STATE::StateWatch statewatch_;  // 连接状态监视器
};

extern uint8_t BoardRx_buffer[BoardCommunication::RX_FRAME_SIZE];
extern uint8_t DT7Rx_buffer[18];                  
extern uint8_t HI12Rx_buffer[82];

#endif
