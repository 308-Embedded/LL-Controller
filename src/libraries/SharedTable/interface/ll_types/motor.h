// remote control types
#pragma once

struct QuadrotorCmd
{
    uint32_t id;
    uint32_t sec;
    uint32_t nsec;
    float    motor1;
    float    motro2;
    float    motor3;
    float    motor4;
}__attribute__((aligned(32)));