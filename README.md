
This is a small library for classical PID controller, with controller, tuner and closed loop performance analyzer. 
Adaptive and Fuzzy PID are built upon this frameowrk.

# Sections:
(1) PID Controller and Config
(2) PID Performance Analyser
(3) PID Tuner
(4) Fuzzy PID
(5) Adaptive PID
(6) References


# 1. PID Controller:
PID Controller is among the most used controllers with over 80% of loops still using them. They are easy, robust and clean, albeit with some caveats.
The basic control law is $u[n]] = K(e[n] + t_d  \frac{de[n]}{dt} + (1/t_s) \sum_0^n(e[n]n) )$. Though some modifications (like derivative filter, integral windup protection) are added for extra stability and robustness.

## 1.1 PID Config
    The struct of initializing the PID Controller. Includes the basic things needed for a PID Controller.

### (i) kp (can be changed, can be tuned)
        kp is the gain of the system. Value of kp decides the main response to present error signal. ki > 0.0
        small kp -> sluggish controller.
        large kp -> noise magnification
        Usually a small kp is chosen and increase it as needed, keeping ki and kd constant. Can be cahnged usomg PIDController::setGains, is tuneble using PIDTuner. Internally represented as k.
        See also: PIDController::setGains, PerformanceMonitor::Monitor, PIDTuner::tune

### (ii) ki (can be changed, can be tuned)
        ki is the gain for the integral term. Though internally, the ti term is preferred, where ti = kp/ki. ti is the integral time constant. it decides the impact of integral term i.e. error history on the control signal and is also invloved with tt = sqrt(ti*td), related to windup protection. Though ki (and subsequently ti) > 0.0, the control can infact go negative even for a positive error history, due to back calculation, which may make the controller work unexpectedly. If windup protection is enabled Integral term is a nonlinear element that has mot impact on the controllor working.
        See also: PIDController::setGains, PerformanceMonitor::Monitor, PIDTuner::tune.

### (iii) kd (can be changed, can be tuned)
        kd is gain for the derivative term. Though internally, the td term is preferred, where td = kd/kp. td is the derivative time constant. it decides the impact of derivative term i.e. rate of change of error on the control signal and is also invloved with ti = sqrt(ti*td), related to windup protection. kd (and subsequently td) > 0.0. Usually 0 < ki << 1. A small ki (i.e. a large derivative time constant) is preferred due to the sensitivity to noise of the derivative term, which can cause instabilities in the controller. A small ki also keeps tt reasonable enough to maintain controller sanity and predictability. 

### (iv) time_step:
        Means the size of time step to be taken for the simulation. Usually 0 < time_step << 1. 
### (v)

# 6. References
