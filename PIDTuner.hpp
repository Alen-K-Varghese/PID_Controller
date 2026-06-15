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

class _vec4
{
public:
    std::array<double, 4> vector;

    const double& operator[](const int& a) const
    {
        return vector[a];
    }

    _vec4 operator+(const _vec4& other) const
    {
        _vec4 result;

        result[0] = vector[0] + other.vector[0];
        result[1] = vector[1] + other.vector[1];
        result[2] = vector[2] + other.vector[2];
        result[3] = vector[3] + other.vector[3];

        return result;
    }

    _vec4 operator+(const double other) const
    {
        _vec4 result;

        result[0] = vector[0] + other;
        result[1] = vector[1] + other;
        result[2] = vector[2] + other;
        result[3] = vector[3] + other;

        return result;
    }

    _vec4 operator-(const _vec4& other) const
    {
        _vec4 result;

        result[0] = vector[0] - other.vector[0];
        result[1] = vector[1] - other.vector[1];
        result[2] = vector[2] - other.vector[2];
        result[3] = vector[3] - other.vector[3];

        return result;
    }

    _vec4 operator-(const double other) const
    {
        _vec4 result;

        result[0] = vector[0] - other;
        result[1] = vector[1] - other;
        result[2] = vector[2] - other;
        result[3] = vector[3] - other;

        return result;
    }

    _vec4 operator*(const _vec4& other) const
    {
        _vec4 result;

        result[0] = vector[0] * other.vector[0];
        result[1] = vector[1] * other.vector[1];
        result[2] = vector[2] * other.vector[2];
        result[3] = vector[3] * other.vector[3];

        return result;
    }

    _vec4 operator*(const double other) const
    {
        _vec4 result;

        result[0] = vector[0] * other;
        result[1] = vector[1] * other;
        result[2] = vector[2] * other;
        result[3] = vector[3] * other;

        return result;
    }

    double& operator[](const int& a)
    {
        return vector[a];
    }

    _vec4 abs()
    {
        _vec4 result;

        result[0] = std::fabs(vector[0]);
        result[1] = std::fabs(vector[1]);
        result[2] = std::fabs(vector[2]);
        result[3] = std::fabs(vector[3]);

        return result;
    }

};

struct _candidateSolution
{
    _vec4 candiate_vector;
    double associated_cost = 1.0e8;
};

std::ostream& operator<<(std::ostream& out, const _candidateSolution& sol)
{
    return out << "\n kp =   "<< sol.candiate_vector[0]
               << "\n ki =   "<< sol.candiate_vector[1]
               << "\n kd =   "<< sol.candiate_vector[2]
               << "\n sp_weight =   "<< sol.candiate_vector[3]
               << "\n assosiated cost =   "<< sol.associated_cost
               <<std::endl;
}


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
    _candidateSolution best_candidate[3];

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

    std::vector<_candidateSolution> initial_candidate_arr;

    _vec4 random_vector()
    {
        _vec4 rand_vector;
        rand_vector[0] = dist(rng);
        rand_vector[1] = dist(rng);
        rand_vector[2] = dist(rng);
        rand_vector[3] = dist(rng);

        return rand_vector;
    }

    void shuffleCandidates(const _candidateSolution candidate)
    {
        if(candidate.associated_cost < best_candidate[0].associated_cost)
        {
            best_candidate[2] = best_candidate[1];
            best_candidate[1] = best_candidate[0];
            best_candidate[0] = candidate;
        }

        else if(candidate.associated_cost < best_candidate[1].associated_cost)
        {
            best_candidate[2] = best_candidate[1];
            best_candidate[1] = candidate;
        }

        else if (candidate.associated_cost < best_candidate[2].associated_cost)
        {
            best_candidate[2] = candidate;
        }
        else {}
    }

    double fitness_measure(const _vec4& candidate)
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
        
        return total_penalty;
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

            std::cout<<initial_candidate_arr[i].associated_cost<<std::endl;
        }

        // best_candidate[0] = initial_candidate_arr[0];
        for(int i = 0; i < initial_candidates; i++)
        {
            shuffleCandidates(initial_candidate_arr[i]);
            std::cout<<"\n"<<best_candidate[0].associated_cost;
        }

    }

    void optimize()
    {
        double a = 2;
        double delta_a = 2.0/max_iterations;

        for(int i = 0; i < max_iterations; i++)
        {
            for(int j = 0; j < initial_candidates; j++)
            {
                _candidateSolution X, Xi[3];

                for(int k = 0; k < 3; k++)
                {
                    _vec4 vec_A, vec_C, vec_D;

                    _vec4 vec_r1 = random_vector();
                    _vec4 vec_r2 = random_vector();

                    vec_A = (vec_r1 * 2*a) - a;
                    vec_C = vec_r2*a;

                    vec_D = ((vec_C * best_candidate[k].candiate_vector) - initial_candidate_arr[j].candiate_vector).abs();
                    
                    Xi[k].candiate_vector = best_candidate[k].candiate_vector - (vec_A*vec_D);
                }
                X.candiate_vector = (Xi[0].candiate_vector + Xi[1].candiate_vector + Xi[2].candiate_vector)*(1/3);

                X.associated_cost = fitness_measure(X.candiate_vector);

                shuffleCandidates(X);
            }
            a -= delta_a;
        }

        std::cout<<best_candidate[0];
    }


};



#endif