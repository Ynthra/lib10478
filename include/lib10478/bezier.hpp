#pragma once

#include "Eigen/Dense"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

namespace lib10478{

class virtualPath
{
public:
    virtual units::V2Position getPoint(double t) const = 0;
    virtual units::V2Position getDerivative(double t) const = 0;
    virtual units::V2Position getSecondDerivative(double t) const = 0;
    virtual Curvature getCurvature(double t) const = 0;
};

class CubicBezier : public virtualPath
{
public:
    CubicBezier(const units::V2Position& p0, const units::V2Position& p1, const units::V2Position& p2, const units::V2Position& p3);

    units::V2Position getPoint(double t) const;
    units::V2Position getDerivative(double t) const;
    units::V2Position getSecondDerivative(double t) const;
    Curvature getCurvature(double t) const;

private:
    // 2x4 matrix of control points in metres
    Eigen::Matrix<double, 4, 2> P;

    Eigen::Matrix<double, 4, 4> coefficients;
    Eigen::Matrix<double, 3, 4> derivativeCoef;
    Eigen::Matrix<double, 2, 4> secondDerivativeCoef;
};

class Spline : public virtualPath 
{
public:
    Spline(std::vector<virtualPath>* paths);

    units::V2Position getPoint(double t) const;
    units::V2Position getDerivative(double t) const;
    units::V2Position getSecondDerivative(double t) const;
    Curvature getCurvature(double t) const;
private:
    std::vector<virtualPath>* paths;
    std::pair<int,double> convertT(double t) const;
};

} //namespace lib10478