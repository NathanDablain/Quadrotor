#pragma once

#include <cstdint>

#define MICROSECONDS_CAP 1000000

struct Sim_Time{
    int32_t Seconds;
    int32_t MicroSeconds;

    double Time_fp();

    void operator+=(Sim_Time const& in);

    void operator-=(Sim_Time const& in);

    Sim_Time operator+(Sim_Time const& in);

    Sim_Time operator-(Sim_Time const& in);

    bool operator<(Sim_Time const& in);

    bool operator>(Sim_Time const& in);

    bool operator<=(Sim_Time const& in);

    bool operator>=(Sim_Time const& in);

    bool operator==(Sim_Time const& in);
};

