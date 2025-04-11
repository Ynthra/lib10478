#include "lib10478/ProfileGenerator.hpp"
#include "units/Vector2D.hpp"


using namespace lib10478;

Profile* ProfileGenerator::generateProfile(const virtualPath& path, std::optional<Constraints> constraints) {
    std::vector<ProfilePoint> profile;
    std::vector<Curvature> midCurvatures; // Stores midpoint curvatures for each segment

    Constraints prevConstraints = this->constraints;
    if (constraints.has_value()) {
        this->constraints = constraints.value();
    }

    Length dist = 0_m;
    LinearVelocity vel = 0_mps;
    double t = 0;

    // Compute initial state once at t = 0
    units::V2Position point = path.getPoint(t);
    units::V2Position deriv = path.getDerivative(t);
    Curvature curvature = path.getCurvature(t);

    // Add the initial profile point using curvature at t = 0.
    profile.push_back(ProfilePoint(
        {point, units::atan2(deriv.y, deriv.x)},
        vel,
        curvature,
        dist
    ));

    while (t < 1) {
        double dt = this->constraints.dd.internal() / deriv.magnitude().internal();

        if (t + dt > 1) { // clamp t if over 1
            dt = 1 - t;
        }
        
        const double t_mid = t + dt / 2.0;
        const Curvature kMid = path.getCurvature(t_mid);
        midCurvatures.push_back(kMid);

        t += dt;
        dist += this->constraints.dd;

        point = path.getPoint(t);
        deriv = path.getDerivative(t);
        curvature = path.getCurvature(t);

        // uses midpoint
        const LinearAcceleration maxAccel = (2 * this->constraints.maxAccel) /
            (units::abs(kMid) * trackWidth + 2);

        
        const LinearVelocity maxSlipVel = units::sqrt(
            9.81_mps2 * this->constraints.frictionCoefficent / curvature
        );
        const LinearVelocity maxTurnVel = (2 * this->constraints.velLimits.at(dist)) /
            (units::abs(curvature) * trackWidth + 2);
        const LinearVelocity maxAccelVel = units::sqrt(vel * vel + 2 * maxAccel * this->constraints.dd);
        
        // The new velocity is the minimum of the three computed limits.
        vel = units::min(units::min(maxSlipVel, maxTurnVel), maxAccelVel);

        profile.push_back(ProfilePoint(
            {point, units::atan2(deriv.y, deriv.x)},
            vel,
            curvature,
            dist
        ));

    }

    vel = 0_mps;
    for (int i = profile.size() - 1; i >= 0; i--) {
        if (i > 0) {
            // For segment [i-1 → i], use the corresponding midpoint curvature.
            const Curvature kMid = midCurvatures[i - 1];
            const LinearAcceleration maxDecel = (2 * this->constraints.maxDecel) /
                (units::abs(kMid) * trackWidth + 2);

            vel = units::min(
                profile[i].velocity,
                units::sqrt(vel * vel + 2 * maxDecel * this->constraints.dd)
            );
        }
        profile[i].velocity = vel;
    }

    this->constraints = prevConstraints;
    return new Profile(profile, this->constraints.dd);
}