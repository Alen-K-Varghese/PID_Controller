#ifndef CTRL_ABC_HPP

#define CTRL_ABC_HPP

#include<iostream>

#include "ControllerConfig.hpp"


template<typename ConfigType>
class Controller
{
public:
    explicit Controller(const ConfigType& cfg)
    :_cfg(cfg) {}

    virtual ~Controller() = default;


    virtual float compute_controlSignal(double setpoint, double reference);
    
protected:
    ConfigType _cfg;
};




#endif