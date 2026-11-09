#pragma once

#include <cstdint>
#include "OperatorInput.hpp"

/*
 * RL部署执行控制器。
 * 上位机提供关节位置目标和左右轮速度目标；本地执行轮速闭环，
 * 并向 MotorTask 提供四个达妙关节的 MIT 增益。
 */
class ChassisController
{
public:
    void Init();
    void Update(const OperatorInput& input);

    /* index顺序：左前、右前、左后、右后。 */
    bool SetJointGains(uint8_t index, float kp, float kd);
    bool AreJointGainsConfigured() const;
    float GetJointKp(uint8_t index) const;
    float GetJointKd(uint8_t index) const;

private:
    float joint_kp_[4] = {};
    float joint_kd_[4] = {};
    bool joint_gains_configured_ = false;
    float height_target_m_ = 0.30f;
    uint32_t last_update_tick_ = 0;
};

extern ChassisController chassis_controller;

/* 保留任务层现有入口名称，具体逻辑由 chassis_controller 执行。 */
void InitChassisControl();
void ChassisControlLoop(const OperatorInput& input);
