#include "bezier.hpp"
#include "Eigen/src/Core/Redux.h"
#include "lib10478/lib10478.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"



using namespace lib10478;
using namespace units;

CubicBezier::CubicBezier(const V2Position &p0, const V2Position &p1, const V2Position &p2, const V2Position &p3)
{
    this->P << 
        p0.x.convert(m), p0.y.convert(m),
        p1.x.convert(m), p1.y.convert(m),
        p2.x.convert(m), p2.y.convert(m),
        p3.x.convert(m), p3.y.convert(m);
    this->coefficients << 
        -1, 3, -3, 1,
        3, -6, 3, 0,
        -3, 3, 0, 0,
        1, 0, 0, 0;
    this->derivativeCoef << 
        -3, 9, -9, 3,
        6, -12, 6, 0,
        -3, 3, 0, 0;
    this->secondDerivativeCoef << 
        -6, 18, -18, 6,
        6, -12, 6, 0;
}
V2Position CubicBezier::getPoint(double t) const
{
    Eigen::RowVector4d T;
    T << t * t * t, t * t, t, 1;
    auto result = (T * this->coefficients) * this->P;
    return V2Position(from_m(result(0)), from_m(result(1)));
}

V2Position CubicBezier::getDerivative(double t) const
{
    Eigen::RowVector3d T;
    T << t * t, t, 1;
    auto result = (T * this->derivativeCoef) * this->P;
    return V2Position(from_m(result(0)), from_m(result(1)));
}

V2Position CubicBezier::getSecondDerivative(double t) const
{
    Eigen::RowVector2d T;
    T << t, 1;
    auto result = (T * this->secondDerivativeCoef) * this->P;
    return V2Position(from_m(result(0)), from_m(result(1)));
}

Curvature CubicBezier::getCurvature(double t) const
{
    V2Position d = this->getDerivative(t);
    V2Position dd = this->getSecondDerivative(t);
    Curvature k = (d.x * dd.y - d.y * dd.x) / (d.magnitude()*d.magnitude()*d.magnitude());
    return k;
}



Spline::Spline(std::vector<virtualPath*> paths)
: paths(paths)
{}

units::V2Position Spline::getPoint(double t) const
{
    auto [index, tIndex] = convertT(t);
    return paths[index]->getPoint(tIndex);
}
units::V2Position Spline::getDerivative(double t) const
{
    auto [index, tIndex] = convertT(t);
    return paths[index]->getDerivative(tIndex);
}
units::V2Position Spline::getSecondDerivative(double t) const
{
    auto [index, tIndex] = convertT(t);
    return paths[index]->getSecondDerivative(tIndex);
}
Curvature Spline::getCurvature(double t) const
{
    auto [index, tIndex] = convertT(t);
    return paths[index]->getCurvature(tIndex);
}

std::pair<int,double> Spline::convertT(double t) const
{
    t *= paths.size();
    if(t < 0) return {0,0};
    if (t >= paths.size()) return {paths.size()-1,1};
    return {int(t),t-int(t)};
}
