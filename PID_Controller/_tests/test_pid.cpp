#include<iostream>
#include"../include/PidController.hpp"

int main()
{
    PIDConfig cfg;

    PidController pid(cfg);
    double setpoint = 10.0;
    double measurement = 0.0;
    double output = 0.0;

    for (int i = 0; i < 100; ++i) {
        output = pid.ComputeControlSignal(setpoint, measurement, 1.0, 1.0, 0.1, 0.01, 1.0);
        std::cout << "Output: " << output << std::endl;
        // Simulate the system response (for testing purposes)
        measurement += output * 0.1; // Simulate a simple system response
    }

    return 0;
}