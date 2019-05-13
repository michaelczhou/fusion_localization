#ifndef LOAM_MOD_LASER_STATE_PARAMETERIZATION_HPP
#define LOAM_MOD_LASER_STATE_PARAMETERIZATION_HPP

#include <array>
#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include "./math.hpp"

namespace loam {
/// \brief The simplified version of IMU state / preintegrated IMU
///        measurement (without bias) / integrated wheel measurement.
struct State {
    State() = default;

    State(double t) : t(t)
    { }

    State(double t,
            const Eigen::Quaterniond& q,
            const Eigen::Vector3d& p,
            const Eigen::Vector3d& v)
        : t(t) {
            this->q = q;
            this->p = p;
            this->v = v;
        }

    State(const State& other)
        : t(other.t), arr(other.arr)
    { }

    State& operator=(const State& other) {
        if (this != &other) {
            t = other.t;
            arr = other.arr;
            new (&q)  Eigen::Map<Eigen::Quaterniond>(arr.data());
            new (&p)  Eigen::Map<Eigen::Vector3d>(arr.data() + 4);
            new (&v)  Eigen::Map<Eigen::Vector3d>(arr.data() + 7);
        }
        return (*this);
    }

    double t = 0.0;

    std::array<double, 10> arr = {{
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    }};

    /// \brief q: The orientation of the IMU in the global cooridnates,
    ///           i.e. the rotation matrix mapping a point from the sensor
    ///           frame to the global frame, expressed in the global
    ///           coordinates.
    Eigen::Map<Eigen::Quaterniond> q = Eigen::Map<Eigen::Quaterniond>(arr.data() + 0);

    /// \brief p: The position of the IMU in the global cooridnates, i.e.
    ///           the translation vector mapping a point from the sensor
    ///           frame to the global frame, expressed in the global
    ///           coordinates.
    Eigen::Map<Eigen::Vector3d> p = Eigen::Map<Eigen::Vector3d>(arr.data() + 4);

    /// \brief v: The translational velocity of the IMU in the global
    ///           frame.
    Eigen::Map<Eigen::Vector3d> v = Eigen::Map<Eigen::Vector3d>(arr.data() + 7);
};

class StateParameterization : public ceres::LocalParameterization {
public:
    virtual ~StateParameterization() override = default;

    virtual bool Plus(
            const double* state,
            const double* error,
            double* newstate) const override {
        using Eigen::Map;
        using Eigen::Quaterniond;
        using Eigen::Vector3d;

        Map<Quaterniond const> q_old(state);
        Map<Vector3d    const> p_old(state+4);
        Map<Vector3d    const> v_old(state+7);

        Map<Vector3d const> dq(error);
        Map<Vector3d const> dp(error+3);
        Map<Vector3d const> dv(error+6);

        Map<Quaterniond> q_new(newstate);
        Map<Vector3d>    p_new(newstate+4);
        Map<Vector3d>    v_new(newstate+7);

        v_new = v_old + dv;
        p_new = p_old + q_old*dp;
        q_new = q_old * Expmap(dq);

        q_new.normalize();

        return true;
    }

    virtual bool ComputeJacobian(
            const double* /*x*/, double* jacobian) const override {
        Eigen::Map<Eigen::Matrix<double, 10, 9, Eigen::RowMajor>>(jacobian).setIdentity();
        return true;
    }

    virtual int GlobalSize() const override
    { return 10; }

    virtual int LocalSize() const override
    { return 9; }
};

}  /* namespace loam */

#endif  /* LOAM_MOD_LASER_STATE_PARAMETERIZATION_HPP */
