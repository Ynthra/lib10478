#pragma once

namespace lib10478 {
    
class PID {
    private:
        float kP, kI, kD, windupRange;
        bool signFlipReset;
        float integral = 0;
        float prevError = 0;

    public:
        PID(float kP, float kI, float kD, float windupRange, bool signFlipReset);

        float update(const float error);
        void reset();
};

} // namespace lib10478