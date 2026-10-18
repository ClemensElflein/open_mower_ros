//
// Created by Clemens Elflein on 27.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "xbot_positioning_core.h"


void xbot::positioning::xbot_positioning_core::setProcessNoise(double q_xy, double q_theta, double q_vx,
                                                               double q_vr) {
    q_xy_ = q_xy;
    q_theta_ = q_theta;
    q_vx_ = q_vx;
    q_vr_ = q_vr;
}

void xbot::positioning::xbot_positioning_core::setSpeedNoise(double r_vx, double r_vr) {
    Kalman::Covariance<SpeedMeasurementT> c;
    c.setZero();
    c(SpeedMeasurementT::VX, SpeedMeasurementT::VX) = r_vx;
    c(SpeedMeasurementT::VR, SpeedMeasurementT::VR) = r_vr;
    sm.setCovariance(c);
}

const xbot::positioning::StateT &xbot::positioning::xbot_positioning_core::predict(double vx, double vr, double dt) {
    sys.setDt(dt);
    u.v() = vx;
    u.dtheta() = vr;

    // Scale the process noise by dt. Without this the amount of uncertainty
    // injected per second depends on how fast the IMU happens to publish, which
    // silently re-tunes the filter whenever the sample rate changes.
    Kalman::Covariance<StateT> q;
    q.setZero();
    q(StateT::X, StateT::X) = q_xy_ * dt;
    q(StateT::Y, StateT::Y) = q_xy_ * dt;
    q(StateT::THETA, StateT::THETA) = q_theta_ * dt;
    q(StateT::VX, StateT::VX) = q_vx_ * dt;
    q(StateT::VR, StateT::VR) = q_vr_ * dt;
    sys.setCovariance(q);

    return ekf.predict(sys, u);
}

const xbot::positioning::StateT &xbot::positioning::xbot_positioning_core::updatePosition(double x, double y, double covariance) {
    pos_m.x_pos() = x;
    pos_m.y_pos() = y;

    Kalman::Covariance<PositionMeasurementT> c;
    c.setIdentity();
    c *= covariance;

    pm.setCovariance(c);

    // Record the innovation and its normalised form before applying the update.
    // NIS is the one honest check on whether Q and R are set sensibly: with a
    // 2-DOF measurement it should average ~2.0. Consistently above means the
    // filter is over-confident (R too tight or Q too small), below means the
    // opposite.
    const auto expected = pm.h(ekf.getState());
    const double innov_x = x - expected.x_pos();
    const double innov_y = y - expected.y_pos();
    last_position_innovation_x = innov_x;
    last_position_innovation_y = innov_y;
    // H is identity for this model, so S = P_xy + R.
    const auto &P = ekf.getCovariance();
    const double s_xx = P(StateT::X, StateT::X) + covariance;
    const double s_yy = P(StateT::Y, StateT::Y) + covariance;
    const double s_xy = P(StateT::X, StateT::Y);
    const double det = s_xx * s_yy - s_xy * s_xy;
    if (det > 1e-12) {
        last_position_nis = (innov_x * (s_yy * innov_x - s_xy * innov_y) +
                             innov_y * (s_xx * innov_y - s_xy * innov_x)) / det;
    }

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
xbot::positioning::xbot_positioning_core::updateSpeed(double vx, double vr) {
    speed_m.vx() = vx;
    speed_m.vr() = vr;

    // R is set once via setSpeedNoise(); it does not change per measurement.
    return ekf.update(sm, speed_m);
}

const xbot::positioning::StateT &xbot::positioning::xbot_positioning_core::getState() {
    return ekf.getState();
}

const Kalman::Covariance<xbot::positioning::StateT> &xbot::positioning::xbot_positioning_core::getCovariance() {
    return ekf.getCovariance();
}

xbot::positioning::xbot_positioning_core::xbot_positioning_core() {
    // Q is rebuilt on every predict() from the per-second process noise, so
    // there is nothing to set here. R for the speed update is static, though.
    setSpeedNoise(1e-3, 4e-4);
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

