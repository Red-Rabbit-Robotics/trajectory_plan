#ifndef JERK_MINIMIZATION_OBJECTIVE_H
#define JERK_MINIMIZATION_OBJECTIVE_H

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <cmath>

namespace ob = ompl::base;

class JerkMinimizationObjective : public ob::OptimizationObjective {
public:
    JerkMinimizationObjective(const ob::SpaceInformationPtr &si) : ob::OptimizationObjective(si) {
        description_ = "Jerk Minimization Objective";
    }

    ob::Cost stateCost(const ob::State *s) const override {
        return ob::Cost(0.0); // Placeholder, not used in this example
    }

    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override {
        return ob::Cost(calculateJerk(s1, s2));// + motionCostMovement(s1, s2));
    }

    double calculateJerk(const ob::State *s1, const ob::State *s2) const {
        const auto *state1 = s1->as<ob::RealVectorStateSpace::StateType>();
        const auto *state2 = s2->as<ob::RealVectorStateSpace::StateType>();

        double jerk = 0.0;
        double dt = si_->getStateSpace()->getLongestValidSegmentLength(); // Assume a constant time step

        for (unsigned int i = 0; i < si_->getStateDimension(); ++i) {
            double p1 = state1->values[i];
            double p2 = state2->values[i];

            // Velocity is the change in position over time
            //double v1 = (p2 - p1) / dt;
            //double v2 = (p2 - p1) / dt; // Assumes equal time step, need actual dt for different steps

            // Acceleration is the change in velocity over time
            //double a1 = (v2 - v1) / dt;

            // Jerk is the change in acceleration over time
            //jerk += a1 / dt* a1 / dt;
            jerk += (p1 - p2) * (p1 - p2);
        }

        return jerk;
    }
    double motionCostMovement(const ob::State *s1, const ob::State *s2) const {
        const auto *state1 = s1->as<ob::RealVectorStateSpace::StateType>();
        const auto *state2 = s2->as<ob::RealVectorStateSpace::StateType>();
        double cost = 0.0;

        // Calculate the movement between state1 and state2
        for (size_t i = 0; i < si_->getStateDimension(); ++i) {
            double movement = std::abs(state2->values[i] - state1->values[i]);
            cost += movement;
        }

        return cost;
    }
};

#endif // JERK_MINIMIZATION_OBJECTIVE_H

