#pragma once
#include <cmath>
#include <queue>

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