//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#ifndef SRC_XBOT_POSITIONING_CORE_H
#define SRC_XBOT_POSITIONING_CORE_H

#include "SystemModel.hpp"
#include "PositionMeasurementModel.hpp"
#include "OrientationMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>


namespace xbot {
    namespace positioning {
        typedef double T;

        typedef xbot::positioning::State<T> StateT;
        typedef xbot::positioning::Control<T> ControlT;
        typedef xbot::positioning::SystemModel<T> SystemModelT;

        typedef xbot::positioning::PositionMeasurement<T> PositionMeasurementT;
        typedef xbot::positioning::OrientationMeasurement<T> OrientationMeasurementT;
        typedef xbot::positioning::PositionMeasurementModel<T> PositionModelT;
        typedef xbot::positioning::OrientationMeasurementModel<T> OrientationModelT;

        class xbot_positioning_core {



        public:
            const StateT &predict(double vx, double vr, double dt);
            const StateT &updatePosition(double x, double y);
            const StateT &updateOrientation(double theta, double covariance);

        private:
            Kalman::ExtendedKalmanFilter<StateT> ekf;
            SystemModelT sys;
            PositionModelT pm;
            OrientationModelT om;

            ControlT u;
            PositionMeasurementT pos_m;
            OrientationMeasurementT orient_m;
        };
    }
}

#endif //SRC_XBOT_POSITIONING_CORE_H
