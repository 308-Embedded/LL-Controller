// The MIT License (MIT)
// Copyright 2026 Liu Chuangye @ chuangyeliu0206@gmail.com
#ifndef ATTITUDE_FILTER_HPP_
#define ATTITUDE_FILTER_HPP_

#include <EmbeddedLie.hpp>
#include <EmbeddedMath.hpp>
namespace Filter
{

using namespace EmbeddedMath;
using namespace EmbeddedLie;
template<typename Scalar>
class AttiFilter{
public:

    using QuaternionType = Quaternion<Scalar>;
    using VectorType = Matrix<Scalar, 3, 1>;
    AttiFilter()
    {
        attitude = QuaternionType::Identity();
        bias = VectorType::Zero();
        uncertainty = 1e-2;
        sample_rate = 1000;
        dt = 1.0 / static_cast<Scalar>(sample_rate);
        gyro_noise = 1e-4;
        meas_noise = 1e-1;
        bias_gain = 0;
    }

    ~AttiFilter() = default;

    QuaternionType predict(const VectorType& w)
    {
        attitude = attitude * quat_Exp((w - bias) * dt);
        uncertainty += gyro_noise * dt;
        return attitude;
    }

    QuaternionType update(const VectorType& y)
    {
        Scalar gain = uncertainty / meas_noise;
        VectorType tmp = 2 * attitude.vec().cross(y);
        VectorType z = y + attitude.w() * tmp + attitude.vec().cross(tmp);
        VectorType dx = VectorType(gain * z.y(), -gain * z.x(), 0);

        attitude = quat_Exp(dx) * attitude;
        uncertainty = uncertainty * meas_noise / (uncertainty + meas_noise);
        tmp = -2 * attitude.vec().cross(dx);
        z = dx + attitude.w() * tmp - attitude.vec().cross(tmp);
    
        bias = bias - bias_gain * z * dt;
        return attitude;
    }

private:
    Scalar gyro_noise;
    Scalar meas_noise;
    Scalar bias_gain;
    uint32_t sample_rate;
    Scalar uncertainty;
    Quaternion<Scalar> attitude;
    VectorType bias;
    Scalar dt;

};
}
#endif