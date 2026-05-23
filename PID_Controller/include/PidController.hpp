#ifndef PID_CONTROLLER_HPP

#define PID_CONTROLLER_HPP

#include<iostream>

#include"ControllerConfig.hpp"
#include"Polynomial.hpp"

struct PIDConfig: public Config
{
    double timeStep;
    double u_max;
    double u_min;
    double spWeight_ProportionalTerm;

    bool allowIntegral;
    bool allowWindupProtection;
    int windupMethod;
    double maxIntegralValue;

    bool allowDerivative;
    bool allowDerivativeFilter;
    double filterConst;

    bool verbose;
};

class PidController
{
public:
    PIDConfig m_cfg;        // Config Sturct
    double controlSignal;   // Output of controller

    // Contructor & Deconstructor
    PidController(const PIDConfig& config);
    

    double ComputeControlSignal(
        const double& __setpoint, 
        const double& __state,
        const double& __time,
        const double& __k, 
        const double& __ti,
        const double& __td,
        const double& __tt);
private:
    
    double __pTerm, __iTerm, __dTerm;
    double __prev_state, __prev_controlSignal, __prev_error;

    void __ResetParams(void);
    void __UpdateParams(void);
    void __displayInternalParams(void);
    void __chechSanity(void);
    
    double __Differentiator(
        const double& state, 
        const double& k, 
        const double& td);
    

    double __Integrator(
        const double error, 
        const double k, 
        const double ti, 
        const double tt);
    
};

#endif