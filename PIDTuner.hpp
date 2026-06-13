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
    double initial_time;
    double final_time;
    double time_step;
};

class PIDTuner
{
private:
    // Best Solution Candidates
    std::array<double, 4> alpha_candidate, bate_candidate, delta_candidate;

    // Random Parameter vactors
    std::array<double, 4> rand_vec_r1, rand_vec_r2;

    // Vector of Parameters
    std::array<double, 4> vec_C, vec_A; 

    // The system to be controler as a system in the form x[n+1] = f(t[n], x[n], u[n])
    std::function<double(double, double, double)> system;

    // initial State, Control and Setpoint
    double initial_state, initial_control, setpoint;

    // Simulation time span for fitness function
    const time_span tspan; 

    // Initial Config for PID Controller
    PIDConfig cfg;

    // Number of wolves in GWO or number of points to be processed in parallel in each gneration
    const unsigned short int initial_candidates; 

    const unsigned short int max_iterations; 

    // Bounds on Parameters 
    std::array<double, 4> upper_bound, lower_bound;

    std::vector<std::array<double,5>> initial_candidate_arr;

    std::array<double, 4> random_vector()
    {
        std::array<double, 4> rand_vector;
        rand_vector[0] = dist(rng);
        rand_vector[1] = dist(rng);
        rand_vector[2] = dist(rng);
        rand_vector[3] = dist(rng);

        return rand_vector;
    }

    double fitness_measure(
        const double kp,
        const double kd,
        const double ki,
        const double sp_weight)
    {
        PIDConfig config = cfg;
        config.kp = kp;
        config.ki = ki;
        config.kd = kd;
        config.sp_weight = sp_weight;
        config.f_enable_monitoring = false;
        config.f_enable_logging = false;

        PIDController ctrl(config);

        double ITAE = 0.0;
      
        double i = tspan.initial_time;
        double tf = tspan.final_time;
        double dt = tspan.time_step; 

        
        double error = 0.0;
        double state = initial_state;
        double control_signal = initial_control;

        // ctrl.begin(0.0, 0.0);

        while(i <= tf)
        {
            error = setpoint - state;
            state = system(i, state, control_signal);

            control_signal =  ctrl.computeControlSignal(setpoint, state);

            // ITAE
            ITAE += dt*std::fabs(error)*i + (state > setpoint)*100;

            i+=dt;
        }
        // ctrl.end();
        return ITAE;
    }

public:
    PIDTuner(
        const std::function<double(double, double, double)>& controlled_system,
        const double& initial_state,
        const double& initial_control,
        const double& setpoint,
        const time_span tspan,
        const PIDConfig& controller_config, 
        const unsigned short int& number_of_agents,
        const unsigned short int& max_iterations,
        const std::array<double, 4>& upper_bound, 
        const std::array<double, 4>& lower_bound)
    :
    system(controlled_system),
    initial_state(initial_state),
    initial_control(initial_control),
    setpoint(setpoint),
    tspan(tspan),
    cfg(controller_config),
    initial_candidates(number_of_agents),
    max_iterations(max_iterations),
    upper_bound(upper_bound), 
    lower_bound(lower_bound),
    initial_candidate_arr(number_of_agents)
    {
        assert((upper_bound[0] >= lower_bound[0]) and (lower_bound[0] >= 0));
        assert((upper_bound[1] >= lower_bound[1]) and (lower_bound[1] >= 0));
        assert((upper_bound[2] >= lower_bound[2]) and (lower_bound[2] >= 0));
        assert((upper_bound[3] >= lower_bound[3]) and (lower_bound[3] >= 0)  and (upper_bound[3] <= 1));

        
        for (int i = 0; i < initial_candidates; i++)
        {
            // Initialization of array for random initial guesses
            initial_candidate_arr[i][0] = lower_bound[0] + dist(rng)*(upper_bound[0] - lower_bound[0]);
            initial_candidate_arr[i][1] = lower_bound[1] + dist(rng)*(upper_bound[1] - lower_bound[1]);
            initial_candidate_arr[i][2] = lower_bound[2] + dist(rng)*(upper_bound[2] - lower_bound[2]);
            initial_candidate_arr[i][3] = lower_bound[3] + dist(rng)*(upper_bound[3] - lower_bound[3]);

            // Finding the cost of these candidate values
            initial_candidate_arr[i][4] = fitness_measure(
                                            initial_candidate_arr[i][0], initial_candidate_arr[i][1], 
                                            initial_candidate_arr[i][2], initial_candidate_arr[i][3]);
        }

        // Sorting the array as the cost as pivot. The 1st three vales are respectively alpha, beta and delta candidates
        for(int i = 1; i < initial_candidates; i++)
        {
            std::array<double, 5> key = initial_candidate_arr[i];
            int j = i - 1;

            while((j >= 0) and (initial_candidate_arr[j][4] > key[4]))
            {
                initial_candidate_arr[j + 1] = initial_candidate_arr[j];
                --j;
            }
            initial_candidate_arr[j + 1] = key;
        }
        
    }


};



#endif