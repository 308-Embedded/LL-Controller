// Copyright 2025 Liu Chuangye @ chuangyeliu0206@gmail.com
// Apache Licence 2.0
#pragma once

#include <cmath>
#include <stdio.h>

template <typename Scalar, int SampleRate>
class DynamicNotchFilter {
public:
    DynamicNotchFilter() = default;
    ~DynamicNotchFilter() = default;

    Scalar filter(const Scalar x)
    {
        Scalar out = nb0 * x + states[0];
        states[0] = nb1 * x - na1 * out + states[1];
        states[1] = nb2 * x - na2 * out;
        return out;
    }

    void set_params(const Scalar center_freq, const Scalar Q = 20)
    {
        this->center_freq = center_freq;
        this->Q = Q;

        update_internal();
        return;
    }

private:
    void update_internal()
    {
        Scalar omega = 2 * M_PI * center_freq / SampleRate;
        Scalar alpha = std::sin(omega) / (2 * Q);
        Scalar cos_w = std::cos(omega);

        a0 =  1 + alpha;
        nb0 = 1 / a0;
        nb1 = (-2 * cos_w) / a0;
        nb2 = 1 / a0;
        na1 = (-2 * cos_w) / a0;
        na2 = (1 - alpha) / a0;

        return;
    }

    Scalar states[2] = {0};
    Scalar center_freq = 100.0;
    Scalar Q = 20.0;
    Scalar a0, nb0, nb1, nb2, na1, na2;

};


template <typename Scalar, int Length, int SampleRate>
class DynamicNotchVecFilter {
public:
    DynamicNotchVecFilter() = default;
    ~DynamicNotchVecFilter() = default;

    void filter(const Scalar* in, Scalar* out)
    {
        for(int i = 0; i < Length; i++)
        {
            out[i] = channels[i].filter(in[i]);
        }
        return;
    }

    void set_params(const Scalar center_freq, const Scalar Q = 20)
    {
        for(int i = 0; i < Length; i++)
        {
            channels[i].set_params(center_freq, Q);
        }
        return;
    }

private:
    DynamicNotchFilter<Scalar, SampleRate> channels[Length];
};