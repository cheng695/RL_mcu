#include "VmcVelocityEstimator.hpp"

#include <cmath>

namespace
{
constexpr double kWheelRadius = 0.060;
constexpr double kPhi0Vertical = 1.5707963267948966;

// Copied from the reference project's VMC calibration and linkage model.
constexpr double kRefRawLeftBack = 2.51313968;
constexpr double kRefRawLeftAhead = -3.20170302;
constexpr double kRefRawRightAhead = -2.630482;
constexpr double kRefRawRightBack = 0.495861;
constexpr double kRefModelPhi1 = 2.20948918;
constexpr double kRefModelPhi4 = 0.93210348;
constexpr double kDirLeftBack = -1.0;
constexpr double kDirLeftAhead = -1.0;
constexpr double kDirRightAhead = 1.0;
constexpr double kDirRightBack = 1.0;

float WrapRad(float angle)
{
    constexpr float pi = 3.14159265358979323846f;
    constexpr float twoPi = 6.28318530717958647692f;
    while (angle > pi) angle -= twoPi;
    while (angle < -pi) angle += twoPi;
    return angle;
}

double ModelAngle(double raw, double raw_reference,
                  double model_reference, double direction)
{
    // Match the reference's float arithmetic and float wrap at this stage.
    const float modelAngle = static_cast<float>(model_reference) +
        static_cast<float>(direction) * (raw - static_cast<float>(raw_reference));
    return WrapRad(modelAngle);
}

void LegPosition(double phi1, double phi4, double &length, double &phi0)
{
    const double c1 = std::cos(phi1);
    const double c4 = std::cos(phi4);
    const double s1 = std::sin(phi1);
    const double s4 = std::sin(phi4);
    const double t6 = c1 * 0.21;
    const double t8 = c1 * 0.105;
    const double t9 = c4 * 0.105;
    const double t10 = s1 * 0.21;
    const double t12 = s1 * 0.105;
    const double t13 = s4 * 0.105;
    const double a = t8 - t9;
    const double b = t12 - t13;
    const double dx = t6 - c4 * 0.21;
    const double dy = t10 - s4 * 0.21;
    const double d2 = dx * dx + dy * dy;
    const double angle = 2.0 * std::atan(
        (1.0 / ((t9 - t8) + d2)) *
        ((t13 - t12) + std::sqrt((a * a + b * b) - d2 * d2)));
    const double x = t10 + std::sin(angle) / 4.0;
    const double z = t6 + std::cos(angle) / 4.0;
    length = std::sqrt(x * x + z * z);
    phi0 = std::atan2(x, z);
}

void LegSpeed(double dPhi1, double dPhi4, double phi1, double phi4,
              double &dLength, double &dPhi0)
{
    // Analytic VMC Jacobian from the reference project's vmc_leg_speed().
    const double t2a = std::cos(phi1);
    const double t3 = std::cos(phi4);
    const double t4a = std::sin(phi1);
    const double t5 = std::sin(phi4);
    const double t10 = t2a * 0.21;
    double t12 = t2a * 0.105;
    const double t13 = t3 * 0.105;
    const double t14 = t4a * 0.21;
    double t16 = t4a * 0.105;
    const double t17 = t5 * 0.105;
    double t29 = t10 - t3 * 0.21;
    const double t30 = t12 - t13;
    double t31 = t14 - t5 * 0.21;
    const double t32 = t16 - t17;
    const double t37 = t4a * t29 * 0.42;
    const double t38 = t5 * t29 * 0.42;
    const double t39initial = t2a * t31 * 0.42;
    const double t40 = t3 * t31 * 0.42;
    const double t51initial = t29 * t29 + t31 * t31;
    const double t57initial = 1.0 / ((t13 - t12) + t51initial);
    const double t58initial = t57initial * t57initial;
    const double root = std::sqrt((t30 * t30 + t32 * t32) - t51initial * t51initial);
    const double t63initial = 1.0 / root;
    const double t66initial = (t17 - t16) + root;
    double angle = 2.0 * std::atan(t57initial * t66initial);
    const double cosAngle = std::cos(angle);
    const double sinAngle = std::sin(angle);
    const double t81 = 1.0 / (t58initial * t66initial * t66initial + 1.0);
    const double t16derivative = t58initial * ((t16 + t39initial) - t37) * t66initial +
        t57initial * (t12 - t63initial * ((t10 * t32 + t4a * t30 * -0.21) +
        t51initial * (t37 - t39initial) * 2.0) / 2.0);
    const double t2derivative = t58initial * ((t17 + t40) - t38) * t66initial +
        t57initial * (t13 - t63initial * ((t5 * t30 * -0.21 +
        t51initial * (t38 - t40) * 2.0) + t3 * t32 * 0.21) / 2.0);
    const double x = t10 + cosAngle / 4.0;
    const double y = t14 + sinAngle / 4.0;
    const double t29sq = x * x + y * y;
    const double invNorm = 1.0 / std::sqrt(t29sq);
    const double invNormSq = 1.0 / t29sq;
    const double t51 = cosAngle * t81;
    const double t12rate = t51 * t2derivative;
    const double t31rate = sinAngle * t81;
    const double t57rate = t31rate * t2derivative;
    const double t51value = t10 - t51 * t16derivative / 2.0;
    const double t29value = t14 - t31rate * t16derivative / 2.0;
    dLength = dPhi4 * invNorm * (y * t12rate - x * t57rate) / 2.0 +
        dPhi1 * invNorm * (y * t51value * 2.0 - x * t29value * 2.0) / 2.0;
    const double t2 = y * (1.0 / (x * x));
    dPhi0 = dPhi1 * (x * x) * invNormSq * ((1.0 / x) * t51value + t2 * t29value) -
        dPhi4 * (x * x) * invNormSq * ((1.0 / x) * (0.0 - t12rate / 2.0) -
        t2 * (t57rate / 2.0));
    (void)t39initial;
    (void)t31;
    (void)t16;
    (void)angle;
}
}

float VmcVelocityEstimator::Estimate(float leftBackAngle, float leftAheadAngle,
                                     float rightAheadAngle, float rightBackAngle,
                                     float leftBackRate, float leftAheadRate,
                                     float rightAheadRate, float rightBackRate,
                                     float leftWheelRate, float rightWheelRate,
                                     float bodyPitch, float bodyPitchRate)
{
    double leftPhi1 = ModelAngle(leftBackAngle, kRefRawLeftBack,
                                 kRefModelPhi1, kDirLeftBack);
    double leftPhi4 = ModelAngle(leftAheadAngle, kRefRawLeftAhead,
                                 kRefModelPhi4, kDirLeftAhead);
    double rightPhi4 = ModelAngle(rightAheadAngle, kRefRawRightAhead,
                                  kRefModelPhi4, kDirRightAhead);
    double rightPhi1 = ModelAngle(rightBackAngle, kRefRawRightBack,
                                  kRefModelPhi1, kDirRightBack);

    double leftLength, leftPhi0, rightLength, rightPhi0;
    double leftDLength, leftDPhi0, rightDLength, rightDPhi0;
    LegPosition(leftPhi1, leftPhi4, leftLength, leftPhi0);
    LegPosition(rightPhi1, rightPhi4, rightLength, rightPhi0);
    LegSpeed(kDirLeftBack * leftBackRate, kDirLeftAhead * leftAheadRate,
             leftPhi1, leftPhi4, leftDLength, leftDPhi0);
    LegSpeed(kDirRightBack * rightBackRate, kDirRightAhead * rightAheadRate,
             rightPhi1, rightPhi4, rightDLength, rightDPhi0);

    const double pitchRate = bodyPitchRate;
    const double leftTheta = WrapRad(static_cast<float>(leftPhi0 - kPhi0Vertical - bodyPitch));
    const double rightTheta = WrapRad(static_cast<float>(rightPhi0 - kPhi0Vertical - bodyPitch));
    const double dTheta = (leftDPhi0 + rightDPhi0) * 0.5 - pitchRate;
    const double leftWheelW = leftWheelRate - pitchRate - leftDPhi0;
    const double rightWheelW = -rightWheelRate - pitchRate - rightDPhi0;
    const double leftVx = -leftWheelW * kWheelRadius +
        leftLength * dTheta * std::cos(leftTheta) + leftDLength * std::sin(leftTheta);
    const double rightVx = -rightWheelW * kWheelRadius +
        rightLength * dTheta * std::cos(rightTheta) + rightDLength * std::sin(rightTheta);
    // This firmware's BoardTX/base-forward convention is opposite to the
    // reference project's d_x_raw sign convention.
    return static_cast<float>(-((leftVx + rightVx) * 0.5));
}
