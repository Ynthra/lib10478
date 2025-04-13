#pragma once
#include <cmath>
#include <cstdlib>
#include <queue>
#include "units/Angle.hpp"

inline double sinc(double x){
    if (std::abs(x) < 1e-9) {
      return 1.0 - 1.0 / 6.0 * x * x;
    } else {
      return std::sin(x) / x;
    }
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class SimpleMovingAverage 
{
public:
    SimpleMovingAverage(int size) : size(size), sum(0.0) {}
double next(double val) {
        if (window.size() == size) {
            sum -= window.front();
            window.pop();
        }
        window.push(val);
        sum += val;
        return sum / window.size();
    }
private:
    int size;
    std::queue<double> window;
    double sum;
};

enum turnDirection{
    CW = -1,
    CCW = 1,
    AUTO = 0
};

inline Angle getAngularError(Angle target, Angle position, turnDirection direction) {
    Angle error = units::remainder(target - position, 1_stRot);
    if(direction == CCW && error < 0_stRot) error += 1_stRot;
    else if(direction == CW && error > 0_stRot) error -= 1_stRot;
    return error;
}

// https://www.desmos.com/calculator/7setziwlwf
inline double driveCurve(double input, double scale = 1.5){
    return powf(2.71828,(std::abs(input) - 1) * scale);
}

//silly function for my silly intake code at nats
inline int roundUpToNearestMultiple(int num, int multiple, int thresh) {
    // Find the nearest multiple greater than or equal to num
    int remainder = num % multiple;
    int roundedUp = num + (remainder == 0 ? 0 : multiple - remainder);
    
    // Check if the difference is less than 500
    if (roundedUp - num < thresh) {
        roundedUp += multiple;  // Round up to the next multiple
    }
    
    return roundedUp;
}