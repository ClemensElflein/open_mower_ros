//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "xbot_positioning_core.h"


const xbot::positioning::StateT &xbot::positioning::xbot_positioning_core::predict(double vx, double vr, double dt) {
    sys.setDt(dt);
    u.v() = vx;
    u.dtheta() = vr;
    return ekf.predict(sys, u);
}

const xbot::positioning::StateT & xbot::positioning::xbot_positioning_core::updatePosition(double x, double y) {
    pos_m.x_pos() = x;
    pos_m.y_pos() = y;

    Kalman::Covariance<PositionMeasurementT> c;
    c.setIdentity();
    c*= 100.0;

    pm.setCovariance(c);

    return ekf.update(pm, pos_m);
}

const xbot::positioning::StateT & xbot::positioning::xbot_positioning_core::updateOrientation(double theta, double covariance) {
    orient_m.theta() = theta;
    Kalman::Covariance<OrientationMeasurementT> c;
    c.setIdentity();
    c*= covariance;

    om.setCovariance(c);

    return ekf.update(om, orient_m);
}
