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

struct candidate_solution
{
    std::array<double, 4> candiate_vector;
    double associated_cost;
};


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
    candidate_solution alpha_candidate, beta_candidate, delta_candidate;

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

    std::vector<candidate_solution> initial_candidate_arr;

    std::array<double, 4> random_vector()
    {
        std::array<double, 4> rand_vector;
        rand_vector[0] = dist(rng);
        rand_vector[1] = dist(rng);
        rand_vector[2] = dist(rng);
        rand_vector[3] = dist(rng);

        return rand_vector;
    }

    double fitness_measure(std::array<double, 4> candidate)
    {
        PIDConfig config = cfg;
        config.kp = candidate[0];
        config.ki = candidate[1];
        config.kd = candidate[2];
        config.sp_weight = candidate[3];
        config.enable_monitoring = false;
        config.enable_logging = false;

        config._tuner_call = true;

        PIDController ctrl(config);

        
      
        double i = tspan.initial_time;
        double tf = tspan.final_time;
        double dt = tspan.time_step; 

        
        double error = 0.0;
        double state = initial_state;
        double control_signal = initial_control;

        ctrl.begin(state, control_signal);

        // Penalty or cost of the system
        double ITAE = 0.0;
        double overshoot_penalty = 0.0;
        double total_penalty = 0.0;

        while(i <= tf)
        {
            error = setpoint - state;
            state = system(i, state, control_signal);

            control_signal =  ctrl.computeControlSignal(setpoint, state);

            // ITAE
            ITAE += dt*std::fabs(error)*i;

            // Penalty imposed for overshoot
            overshoot_penalty += (state > setpoint)*25;

            i+=dt;
        }
        total_penalty = ITAE + overshoot_penalty + ctrl._returnPerformanceMatrices();
        
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
        const unsigned short int& number_of_initial_candidates,
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
    initial_candidates(number_of_initial_candidates),
    max_iterations(max_iterations),
    upper_bound(upper_bound), 
    lower_bound(lower_bound),
    initial_candidate_arr(number_of_initial_candidates)
    {
        // At leat 3 initial candidates for alpha, beta and delta. the more the better
        assert(number_of_initial_candidates >= 3);

        // there should be at least 1 iteration and max allowed iterations = 1e6
        assert((max_iterations > 1) and (max_iterations < 1e6));

        assert((upper_bound[0] >= lower_bound[0]) and (lower_bound[0] >= 0));
        assert((upper_bound[1] >= lower_bound[1]) and (lower_bound[1] >= 0));
        assert((upper_bound[2] >= lower_bound[2]) and (lower_bound[2] >= 0));
        assert((upper_bound[3] >= lower_bound[3]) and (lower_bound[3] >= 0)  and (upper_bound[3] <= 1));
    }

    void initialize(void)
    {
        for (int i = 0; i < initial_candidates; i++)
        {
            // Initialization of array for random initial guesses
            initial_candidate_arr[i].candiate_vector[0] = lower_bound[0] + dist(rng)*(upper_bound[0] - lower_bound[0]);
            initial_candidate_arr[i].candiate_vector[1] = lower_bound[1] + dist(rng)*(upper_bound[1] - lower_bound[1]);
            initial_candidate_arr[i].candiate_vector[2] = lower_bound[2] + dist(rng)*(upper_bound[2] - lower_bound[2]);
            initial_candidate_arr[i].candiate_vector[3] = lower_bound[3] + dist(rng)*(upper_bound[3] - lower_bound[3]);

            // Finding the cost of these candidate values
            initial_candidate_arr[i].associated_cost = fitness_measure(initial_candidate_arr[i].candiate_vector);
        }

        // Sorting the array with cost as pivot. The 1st three vales are respectively alpha, beta and delta candidates
        for(int i = 1; i < initial_candidates; i++)
        {
            candidate_solution key = initial_candidate_arr[i];
            int j = i - 1;

            while((j >= 0) and (initial_candidate_arr[j].associated_cost > key.associated_cost))
            {
                initial_candidate_arr[j + 1] = initial_candidate_arr[j];
                --j;
            }
            initial_candidate_arr[j + 1] = key;
        }

        alpha_candidate = initial_candidate_arr[0];
        beta_candidate = initial_candidate_arr[1];
        delta_candidate = initial_candidate_arr[2];
    }


};



#endif