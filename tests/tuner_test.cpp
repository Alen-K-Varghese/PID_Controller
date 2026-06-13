#include<iostream>
#include<array>
#include<functional>

#include"../ClassicalPID.hpp"
#include"../PIDTuner.hpp"

constexpr double dt = 1e-2;

time_span tspan{0.0, 10.0, 1e-3};

std::array<double, 4> ub = {1.0, 2.0, 3.0, 1.0};
std::array<double, 4> lb = {0.0, 0.0, 0.0, 0.0};

double Sys(double t, double x, double u)
{
    return (1 - dt)*x + dt*u;
}

int main(void)
{
    // Declaring Controller
    PIDConfig cfg;
    // Configuring Controller
    cfg.kp = 1.0;   cfg.ki = 1.0;   cfg.kd = 1e-3;

    cfg.time_step = dt;
    
    cfg.u_min = -1.1;   cfg.u_max = 1.1;
    
    cfg.sp_weight = 1.0;
    cfg.allow_windup_protection = true;
    cfg.allow_filter = true;
    cfg.filter_const = 1.0;
    cfg.f_enable_monitoring = true;

    PIDTuner tuner(
        Sys,
        0.0, 0.0, 1.0, 
        tspan, cfg, 5, 2, ub, lb);

    return 0;
}