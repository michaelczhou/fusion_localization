//
// Created by zhouchang on 19-4-15.
//
/// \file wheel_integrator.cpp
/// \brief Wheel preintegration.
/// \author RU WANG, rockidog@live.com
/// \date 2019-04-10

#include "./wheel_integrator.hpp"

#include <cassert>
#include "./math.hpp"

namespace loam {

    WheelIntegral WheelPreintegrator::Integrate(
            const std::vector<RawWheel>& wheels, const Eigen::Matrix<double, 6, 1>& gawn_cov) {
        using Eigen::Quaterniond;
        using Eigen::Vector3d;
        using Eigen::Matrix3d;
        using Matrix9d   = Eigen::Matrix<double, 9, 9>;
        using Matrix6d   = Eigen::Matrix<double, 6, 6>;

        assert(wheels.size() > 1 && "Should provide two wheel readings at least!");

        State delta_W;
        Matrix9d delta_cov = Matrix9d::Zero();

        int i = 0;
        int j = wheels.size() - 1;

        for (int k = i; k <= j - 1; ++k) {
            const RawWheel& W_k = wheels[k];
            const RawWheel& W_k1 = wheels[k];

            /* Wheel increment from k to k+1, i.e. ΔW(k,k+1)
             * zeroth order rotation integration
             * position integration */
            double d_t = W_k1.t - W_k.t;

            const Vector3d& w = W_k.w;
            const Quaterniond& d_q_old = delta_W.q;
            Quaterniond d_q = Expmap(w * d_t);

            const Vector3d& v = W_k.v;
            Vector3d d_p = d_q_old * v * d_t;

            /* compute the propagation of the wheel noise */
            Matrix3d d_R_old = d_q_old.matrix();
            Matrix6d A = Matrix6d::Zero();
            A.block<3, 3>(0, 0) = d_q.conjugate().matrix();
            A.block<3, 3>(3, 0) = -d_R_old * SkewSymmMatrix(v) * d_t;
            A.block<3, 3>(3, 3) = Matrix3d::Identity();

            Matrix6d B = Matrix6d::Zero();
            B.block<3, 3>(0, 0) = RightJacobian(w * d_t, 1.0e-10) * d_t;
            B.block<3, 3>(3, 3) = d_R_old * d_t;

            Eigen::Block<Matrix9d, 6, 6> S = delta_cov.block<6, 6>(0, 0);
            S = A*S*A.transpose() + B*gawn_cov.asDiagonal()*B.transpose();

            /* Wheel increment from i to k+1, i.e. ΔI(i,k+1) */
            delta_W.t += d_t;
            delta_W.q *= d_q;
            delta_W.p += d_p;
        }

        delta_W.v = wheels[j].v;
        delta_cov.block<3, 3>(6, 6) = gawn_cov.tail<3>().asDiagonal();

        return WheelIntegral(wheels, delta_W, delta_cov);
    }

    State WheelPreintegrator::PropagateState(const State& old, const WheelIntegral& integral) {
        const State& W_ij = integral.mean;
        return State(
                old.t + W_ij.t,
                old.q * W_ij.q,
                old.p + old.q*W_ij.p,
                old.q * W_ij.q*W_ij.v);
    }

}  /* namespace loam */

