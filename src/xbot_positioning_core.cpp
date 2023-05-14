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

const xbot::positioning::StateT &xbot::positioning::xbot_positioning_core::updatePosition(double x, double y, double covariance) {
    pos_m.x_pos() = x;
    pos_m.y_pos() = y;

    Kalman::Covariance<PositionMeasurementT> c;
    c.setIdentity();
    c *= covariance;

    pm.setCovariance(c);

    return ekf.update(pm, pos_m);
}

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updateOrientation(double theta, double covariance) {
    orient_m.theta() = theta;
    Kalman::Covariance<OrientationMeasurementT> c;
    c.setIdentity();
    c *= covariance;

    om.setCovariance(c);

    return ekf.update(om, orient_m);
}

const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updateOrientation2(double vx, double vy, double covariance) {
    orient_m2.vx() = vx;
    orient_m2.vy() = vy;

    Kalman::Covariance<OrientationMeasurementT2> c;
    c.setIdentity();
    c *= covariance;

    om2.setCovariance(c);

    return ekf.update(om2, orient_m2);
}
const xbot::positioning::StateT &
xbot::positioning::xbot_positioning_core::updateSpeed(double vx, double vr, double covariance) {
    speed_m.vx() = vx;
    speed_m.vr() = vr;
//
//    Kalman::Covariance<SpeedMeasurementT> c;
//    c.setIdentity();
//    c *= covariance;
//
//    sm.setCovariance(c);

    return ekf.update(sm, speed_m);
}

const xbot::positioning::StateT &xbot::positioning::xbot_positioning_core::getState() {
    return ekf.getState();
}

const Kalman::Covariance<xbot::positioning::StateT> &xbot::positioning::xbot_positioning_core::getCovariance() {
    return ekf.getCovariance();
}

xbot::positioning::xbot_positioning_core::xbot_positioning_core() {
//    Kalman::Covariance<StateT> c;
//    c.setIdentity();
//    c *= 0.001;
//    sys.setCovariance(c);
    setState(0,0,0,0,0);
}

void xbot::positioning::xbot_positioning_core::setState(double px, double py, double theta, double vx, double vr) {
    StateT x;
    x.setZero();
    x.x() = px;
    x.y() = py;
    x.theta() = theta;
    x.vx() = vx;
    x.vr() = vr;
    this->ekf.init(x);
    Kalman::Covariance<StateT> c;
    c.setIdentity();
    this->ekf.setCovariance(c);
}

void xbot::positioning::xbot_positioning_core::setAntennaOffset(double offset_x, double offset_y) {
    pm.antenna_offset_x = om2.antenna_offset_x = offset_x;
    pm.antenna_offset_y = om2.antenna_offset_y = offset_y;
}

