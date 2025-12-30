// remote control types
#pragma once

struct AcroCmd
{
    uint32_t id;
    uint32_t sec;
    uint32_t nsec;
    uint32_t status;
    float    throttle;
    float    yaw_vel;
    float    pitch_vel;
    float    roll_vel;
}__attribute__((aligned(32)));