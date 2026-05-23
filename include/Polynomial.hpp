#ifndef POLYNOMIAL_HPP

#define POLYNOMIAL_HPP

#include<iostream>
#include<vector>
#include<memory>


class Polynomial
{
public:
    Polynomial(const std::vector<float>& __polyCoeff);
    double operator()(double i) const;


    int order(void);

private:
    std::vector<float> _coeff;
    unsigned short int _order;

    void ValidatePolynomial(void);
    void ComputeDerivative(void);
};

class PiecewisePolynomial
{
public:
    PiecewisePolynomial(
        const double& __t0, 
        const double& __t1, 
        const Polynomial& __poly, 
        std::shared_ptr<PiecewisePolynomial> __next);
    double operator()(double i) const;

private:
        double _t0; double _t1;

        Polynomial _poly;
        std::shared_ptr<PiecewisePolynomial> _next;
};

#endif