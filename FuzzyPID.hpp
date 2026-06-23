#ifndef FUZZY_PID_H

#define FUZZY_PID_H

#include<iostream>
#include<cmath>
#include<algorithm>
#include<cassert>
#include<array>
#include<vector>

#include"ClassicalPID.hpp"

// Tolerance for floating point rounding errors
constexpr double g_eTol = 1e-6;

// The different types on signals for input and output layer
enum SignalClass {
    NL = 0, // Negative Large
    NM = 1, // Negative Medium
    NS = 2, // Negative Small
    ZR = 3, // Zero
    PS = 4, // Positive Small
    PM = 5, // Positive Medium
    PL = 6  // Positive Large
};

// Matrix That stores the input to output map
using _ruleMatrix = std::array<std::array<std::array<int, 2>, 7>, 7>;

class _TrigMembFun
{
private:
    double left, center, right;

    void validateConfig()
    {
        assert(right > left);
        assert((center > left) && (right > center));
    }

public:
    _TrigMembFun(double left,
                       double center,
                       double right)
        : left(left), center(center),right(right)

    {validateConfig();}

    double membership(double x)
    {
        return (x == center)*1.0
                + (x < center and x > left)*(x - left) / (center - left)
                + (x > center and x < right)*(right - x) / (right - center);
    }
};

class FuzzyLayer{
private:
    std::array<double, 7> peaks;
    
public:
    FuzzyLayer(
        const double NL_set[3],
        const double NM_set[3],
        const double NS_set[3],
        const double ZR_set[3],
        const double PS_set[3],
        const double PM_set[3],
        const double PL_set[3])
    {
    }


    FuzzyLayer(
        const std::string& Membership_function_type,
        const double NL_set[2],
        const double NM_set[2],
        const double NS_set[2],
        const double ZR_set[2],
        const double PS_set[2],
        const double PM_set[2],
        const double PL_set[2])
    {

    }
};

class FuzzyPID
{
private:
    double Kp_limits[2];

public:
    FuzzyPID(
        const double Kp_limits[2],
        const double Ki_limits[2],
        const double Kd_limits[2],
        const FuzzyLayer& error_layer,
        const FuzzyLayer& delta_error_layer,
        const FuzzyLayer& gain_layer)
    {}
};


#endif