#ifndef KALMAN_EXAMPLES_ROBOT1_ORIENTATIONMEASUREMENTMODEL2_HPP_
#define KALMAN_EXAMPLES_ROBOT1_ORIENTATIONMEASUREMENTMODEL2_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include "SystemModel.hpp"

namespace xbot
{
    namespace positioning
    {

/**
 * @brief Measurement vector measuring an orientation (i.e. by using a compass)
 *
 * @param T Numeric scalar type
 */
template<typename T>
class OrientationMeasurement2 : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(OrientationMeasurement2, T, 2)
    
    //! Orientation
    static constexpr size_t VX = 0;
    static constexpr size_t VY = 1;

    T vx()  const { return (*this)[ VX ]; }
    T& vx() { return (*this)[ VX ]; }
    T vy()  const { return (*this)[ VY ]; }
    T& vy() { return (*this)[ VY ]; }

};

/**
 * @brief Measurement model for measuring orientation of a 3DOF robot
 *
 * This is the measurement model for measuring the orientation of our
 * planar robot. This could be realized by a compass / magnetometer-sensor.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class OrientationMeasurementModel2 : public Kalman::LinearizedMeasurementModel<State<T>, OrientationMeasurement2<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef xbot::positioning::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  xbot::positioning::OrientationMeasurement2<T> M;
    
    OrientationMeasurementModel2()
    {
        // Setup jacobians. As these are static, we can define them once
        // and do not need to update them dynamically
        this->H.setIdentity();
        this->V.setIdentity();
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;
        
        // Measurement is given by the actual robot orientation
        measurement.vx() = x.vx() * std::cos(x.theta()) - (std::sin(x.theta()) * antenna_offset_x * x.vr()) - (std::cos(x.theta()) * antenna_offset_y * x.vr());
        measurement.vy() = x.vx() * std::sin(x.theta()) + (std::cos(x.theta()) * antenna_offset_x * x.vr()) - (std::sin(x.theta()) * antenna_offset_y * x.vr());

        return measurement;
    }

        void updateJacobians( const S& x )
        {
            this->H.setZero();
            // partial derivative of meas.vx() w.r.t. x.theta()

            this->H( M::VX, S::THETA ) = -x.vx() * std::sin(x.theta()) - antenna_offset_x * x.vr() * std::cos(x.theta()) + std::sin(x.theta()) *antenna_offset_y *x.vr();
            this->H( M::VY, S::THETA ) = x.vx() * std::cos(x.theta())  - antenna_offset_x * x.vr() * std::sin(x.theta()) - std::cos(x.theta()) * antenna_offset_y * x.vr();
        }


    double antenna_offset_x = 0;
    double antenna_offset_y = 0;
};

} // namespace Robot
} // namespace KalmanExamples

#endif