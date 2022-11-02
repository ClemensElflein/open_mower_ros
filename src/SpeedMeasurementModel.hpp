#ifndef KALMAN_EXAMPLES_ROBOT1_SPEEDEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_ROBOT1_SPEEDEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>
#include "SystemModel.hpp"

namespace xbot
{
namespace positioning
{

/**
 * @brief Measurement vector measuring the robot position
 *
 * @param T Numeric scalar type
 */
template<typename T>
class SpeedMeasurement : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(SpeedMeasurement, T, 2)

    static constexpr size_t VX = 0;
    static constexpr size_t VR = 1;
    
    T vx()       const { return (*this)[ VX ]; }
    T vr()       const { return (*this)[ VR ]; }
    
    T& vx()      { return (*this)[ VX ]; }
    T& vr()      { return (*this)[ VR ]; }
};

/**
 * @brief Measurement model for measuring the position of the robot
 *        using two beacon-landmarks
 *
 * This is the measurement model for measuring the position of the robot.
 * The measurement is given by two landmarks in the space, whose positions are known.
 * The robot can measure the direct distance to both the landmarks, for instance
 * through visual localization techniques.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SpeedMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, SpeedMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef  xbot::positioning::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  xbot::positioning::SpeedMeasurement<T> M;
    
    /**
     * @brief Constructor
     *
     * @param landmark1x The x-position of landmark 1
     * @param landmark1y The y-position of landmark 1
     * @param landmark2x The x-position of landmark 2
     * @param landmark2y The y-position of landmark 2
     */
    SpeedMeasurementModel()
    {
        // Setup noise jacobian. As this one is static, we can define it once
        // and do not need to update it dynamically
        this->V.setIdentity();


        this->H.setZero();
        this->H(M::VX, S::VX) = 1;
        this->H(M::VR, S::VR) = 1;
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

        measurement.vx() = x.vx();
        measurement.vr() = x.vr();

        return measurement;
    }


protected:

    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear measurement function \f$h(x)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    void updateJacobians( const S& x )
    {
    }
};

} // namespace Robot
} // namespace KalmanExamples

#endif