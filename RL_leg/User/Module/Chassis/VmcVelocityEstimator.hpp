#ifndef VMC_VELOCITY_ESTIMATOR_HPP
#define VMC_VELOCITY_ESTIMATOR_HPP

namespace VmcVelocityEstimator
{
// Input order matches the reference VMC project: left-back, left-ahead,
// right-ahead, right-back. Angles and angular rates are raw motor feedback.
float Estimate(float left_back_angle, float left_ahead_angle,
               float right_ahead_angle, float right_back_angle,
               float left_back_rate, float left_ahead_rate,
               float right_ahead_rate, float right_back_rate,
               float left_wheel_rate, float right_wheel_rate,
               float body_pitch, float body_pitch_rate);
}

#endif
