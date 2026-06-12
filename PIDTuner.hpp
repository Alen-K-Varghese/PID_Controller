#ifndef PIDTUNER_H

#define PIDTUNER_H

#include<iostream>
#include<cmath>
#include<algorithm>
#include<vector>
#include<array>
#include<random>
#include<cassert>
#include<functional>

#include"ClassicalPID.hpp"


std::mt19937 rng(std::random_device{}());
std::uniform_real_distribution<double> dist(0.0, 1.0);

struct time_span
{
    double initial_time = 0.0;
    double final_time = 1.0;
    double time_step = 1e-3;
};

class PIDTuner
{
private:
    // The system to be controler as a system in the form x[n+1] = f(x[n], u[n], t[n])
    std::function<double(double, double, double)> system;

    // initial State and Control
    double initial_state, initial_control, setpoint;

    // Simulation time span for fitness function
    const time_span tspan; 

    // Initial Config for PID Controller
    PIDConfig cfg;

    // Number of wolves in GWO or number of points to be processed in parallel in each gneration
    const unsigned short int m_agents; 

    const unsigned short int m_iter_limit; 

    // Bounds on Parameters 
    std::array<double, 4> m_upper_bound, m_lower_bound;

    std::vector<std::array<double,5>> candidate_values;

public:
    PIDTuner(
        const std::function<double(double, double, double)>& controlled_system,
        const double& initial_state,
        const double& initial_control,
        const double& setpoint,
        const time_span tspan,
        const PIDConfig& controller_config, 
        const unsigned short int& number_of_agents,
        const unsigned short int& iterations,
        const std::array<double, 4>& upper_bound, 
        const std::array<double, 4>& lower_bound)
    :
    system(controlled_system),
    initial_state(initial_state),
    initial_control(initial_control),
    setpoint(setpoint),
    tspan(tspan),
    cfg(controller_config),
    m_agents(number_of_agents),
    m_iter_limit(iterations),
    m_upper_bound(upper_bound), 
    m_lower_bound(lower_bound),
    candidate_values(number_of_agents)
    {
        assert(upper_bound[0] > lower_bound[0]);
        assert(upper_bound[1] > lower_bound[1]);
        assert(upper_bound[2] > lower_bound[2]);
        assert(upper_bound[3] > lower_bound[3]);

        for (int i = 0; i < m_agents; i++)
        {
            candidate_values[i][0] = m_lower_bound[0] + dist(rng)*(m_upper_bound[0] - m_lower_bound[0]);
            candidate_values[i][1] = m_lower_bound[1] + dist(rng)*(m_upper_bound[1] - m_lower_bound[1]);
            candidate_values[i][2] = m_lower_bound[2] + dist(rng)*(m_upper_bound[2] - m_lower_bound[2]);
            candidate_values[i][3] = m_lower_bound[3] + dist(rng)*(m_upper_bound[3] - m_lower_bound[3]);
        }
    }

    double fitness_measure(const std::array<double, 4>& candidate_value)
    {
        PIDConfig config = cfg;
        config.kp = candidate_value[0];
        config.ki = candidate_value[1];
        config.kd = candidate_value[2];
        config.sp_weight = candidate_value[3];
        config.f_enable_monitoring = false;

        PidController ctrl(config);

        double ITAE = 0.0;

        double i = tspan.initial_time;
        double tf = tspan.final_time;
        double dt = tspan.time_step; 

        double error = 0.0;
        double state = initial_state;
        double control_signal = initial_control;

        while(i <= tf)
        {
            error = setpoint - state;
            state = system(i, state, control_signal);
            control_signal =  ctrl.computeControlSignal(setpoint, state);

            // ITAE
            ITAE += dt*std::fabs(error)*i;

            i+=dt;
        }
        return ITAE;
    }
};



#endif