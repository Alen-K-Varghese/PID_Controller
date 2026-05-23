#include<iostream>
#include<cmath>
#include<algorithm>

#include"../include/PidController.hpp"


PidController::PidController(const PIDConfig& cfg)
    :m_cfg(cfg)
    {
        __ResetParams();
    }

void PidController::__ResetParams(void)
{
    __pTerm = 0; __iTerm=0.0; __dTerm = 0.0;
    __prev_controlSignal = 0.0; __prev_error = 0.0; __prev_state = 0.0;
}

double PidController::ComputeControlSignal(
    const double& __setpoint, 
    const double& __state,
    const double& __time,
    const double& __k, 
    const double& __ti,
    const double& __td,
    const double& __tt)
        {   
            double __error = __setpoint - __state;
            double error_pTerm = (m_cfg.spWeight_ProportionalTerm * __setpoint - __state);

            __pTerm = __k * error_pTerm;
            
            if(m_cfg.allowDerivative)
            {
                __dTerm = __Differentiator(__state, __k, __td);
            }
            else __dTerm = 0.0;
            

            if(m_cfg.allowIntegral)
            {
                __iTerm = __Integrator(__error, __k, __ti, __tt);
            }
            else __iTerm = 0.0;

            // u_min <= control signal <= u_max 
            controlSignal = std::clamp(__pTerm + __iTerm + __dTerm, m_cfg.u_min, m_cfg.u_max);

            return controlSignal;
        }

double PidController::__Differentiator(
    const double& state, 
    const double& k, 
    const double& td)    
    {
        const double& h = m_cfg.timeStep;    // Time Step

        // Weight Computation for Derivative Term (Tustins Method)
        double a1 = -1.0, a2 = k*td/h; // Normal weights, no filter
        
        if (m_cfg.allowDerivativeFilter)
        {   
            // First-Order Filter
            const double& N = m_cfg.filterConst; // filterConst
            a1 = (2*td - N*h)/(2*td*N + N*h);
            a2 = -(2*k*td)/(2*td + N*h);
        }

        return a1 * __dTerm + a2*(state - __prev_state);
    }

double PidController::__Integrator(
    const double error, 
    const double k, 
    const double ti, 
    const double tt)
    {
        const double& h = m_cfg.timeStep;    // Time Step
        if (m_cfg.allowWindupProtection)
        {
            switch (m_cfg.windupMethod)
            {
            case 1:
            {
                // Back Calculation
                double u_predicted = __pTerm + __dTerm + __iTerm;
                double e_s = std::clamp(u_predicted, m_cfg.u_min, m_cfg.u_max) - u_predicted;
                return __iTerm + (k*h)/(2*ti)*(error + __prev_error) + (1/tt)*(e_s);
            }



            default:
                // Clamping
                return std::max(__iTerm + (k*h)/(2*ti)*(error + __prev_error), m_cfg.maxIntegralValue);
                
            }
        }
        return __iTerm + (k*h)/(2*ti)*(error + __prev_error);
    }

int main()
{
    PIDConfig cfg;
    cfg.spWeight_ProportionalTerm = 0.5;
    cfg.allowDerivative = true;
    cfg.allowIntegral = true;
    cfg.timeStep = 0.1;
    cfg.u_min = -10.0; cfg.u_max = 10.0;

    PidController pid(cfg);

    double setpoint = 1.0; double state = 0.0; double time = 0.0;
    double k = 2.0; double ti = 1.0; double td = 0.5; double tt = 10.0;


    return 0;
}
