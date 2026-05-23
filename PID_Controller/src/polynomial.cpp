#include<iostream>
#include<vector>
#include<cmath>
#include<cassert>

#include"Polynomial.hpp"

#define eTol 1e-08  // Error tolerance for floating point numbers

Polynomial::Polynomial(const std::vector<float>& polyCoeff)
{
    _coeff = polyCoeff;
    _order = polyCoeff.size()-1;
    validatePolynomial();
}

void Polynomial::validatePolynomial()
{
    assert(std::abs(_coeff[0]) > eTol);
}

double Polynomial::operator()(double i) const
{
    // calculation using Horners method
    double val = _coeff[0];
    for(int j=1; j<=_order; ++j) {val= val*i + _coeff[j];}

    return val;
}

PiecewisePolynomial::PiecewisePolynomial(
    const double __t0, 
    const double __t1, 
    const Polynomial& __poly, 
    std::shared_ptr<PiecewisePolynomial> __next)
    : _t0(__t0), _t1(__t1), _poly(__poly), _next(__next) {}