//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#ifndef SRC_XBOT_POSITIONING_CORE_H
#define SRC_XBOT_POSITIONING_CORE_H

#include "SystemModel.hpp"
#include "PositionMeasurementModel.hpp"
#include "OrientationMeasurementModel.hpp"
#include "OrientationMeasurementModel2.hpp"
#include "SpeedMeasurementModel.hpp"

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
        typedef xbot::positioning::OrientationMeasurement2<T> OrientationMeasurementT2;
        typedef xbot::positioning::SpeedMeasurement<T> SpeedMeasurementT;
        typedef xbot::positioning::PositionMeasurementModel<T> PositionModelT;
        typedef xbot::positioning::OrientationMeasurementModel<T> OrientationModelT;
        typedef xbot::positioning::OrientationMeasurementModel2<T> OrientationModelT2;
        typedef xbot::positioning::SpeedMeasurementModel<T> SpeedModelT;

        class xbot_positioning_core {



        public:
            xbot_positioning_core();

            const StateT &predict(double vx, double vr, double dt);
            const StateT &updatePosition(double x, double y, double covariance = 500.0);
            const StateT &updateOrientation(double theta, double covariance);
            const StateT &updateOrientation2(double vx, double vy, double covariance);
            const StateT &updateSpeed(double vx, double vr);
            const StateT &getState();
            void setState(double px, double py, double theta, double vx, double vr);
            const Kalman::Covariance<StateT> &getCovariance();
            void setAntennaOffset(double offset_x, double offset_y);

            /**
             * Process noise, as variance per second. predict() scales these by dt,
             * so the filter's behaviour no longer depends on the IMU sample rate.
             */
            void setProcessNoise(double q_xy, double q_theta, double q_vx, double q_vr);

            /**
             * Measurement noise for the wheel-odometry speed / gyro rate update.
             */
            void setSpeedNoise(double r_vx, double r_vr);

            /**
             * Innovation and its normalised form (NIS) from the most recent
             * position update, for tuning. NIS should average ~2.0 (2 DOF) when
             * the noise model is honest.
             */
            double last_position_innovation_x = 0.0;
            double last_position_innovation_y = 0.0;
            double last_position_nis = 0.0;

        public:
            Kalman::ExtendedKalmanFilter<StateT> ekf{};
            SystemModelT sys{};
            PositionModelT pm{};
            OrientationModelT om{};
            OrientationModelT2 om2{};
            SpeedModelT sm{};

            ControlT u{};
            PositionMeasurementT pos_m{};
            OrientationMeasurementT orient_m{};
            OrientationMeasurementT2 orient_m2{};
            SpeedMeasurementT speed_m{};

        private:
            // Process noise as variance per second. Defaults are the values
            // characterised on the GOAT O600; see setProcessNoise().
            double q_xy_ = 1e-4;
            double q_theta_ = 1e-4;
            double q_vx_ = 1e-2;
            double q_vr_ = 1e-2;
        };
    }
}

#endif //SRC_XBOT_POSITIONING_CORE_H
