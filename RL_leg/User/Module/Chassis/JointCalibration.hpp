#pragma once

/* 按URDF零位姿态下的getAngleRad()实测值标定，单位为rad。 */
namespace JointCalibration
{
// 当前绑定LeftDmMotors(2)，实际为右前；URDF零位时的原始编码器角度，单位rad。
inline constexpr float LEFT_FRONT_OFFSET_RAD = -2.4538421f;
// 当前绑定RightDmMotors(1)，实际为左前。
inline constexpr float RIGHT_FRONT_OFFSET_RAD = 2.92191219f;
// 当前绑定LeftDmMotors(1)，实际为右后；URDF零位时的原始编码器角度，单位rad。
inline constexpr float LEFT_REAR_OFFSET_RAD = 1.68135357f;
// 当前绑定RightDmMotors(2)，实际为左后。
inline constexpr float RIGHT_REAR_OFFSET_RAD = 1.31704426f;
} // namespace JointCalibration
