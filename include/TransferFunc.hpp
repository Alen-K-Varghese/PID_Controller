#ifndef TF_HPP

#define TF_HPP

#include <iostream>
#include<vector>

#include"Polynomial.hpp"

class TF
{
public: 
    TF(Polynomial& __numeratorPolynomial, Polynomial& __denominatorPolynomial);
    TF(std::vector<double>& __numeratorCoefficient, std::vector<double>& __denominatorCoefficient);
    double operator()(double i) const;

private:
    void validateTF(void);

    Polynomial _numPoly;
    Polynomial _denPoly;

};

#endif