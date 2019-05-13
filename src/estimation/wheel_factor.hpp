//
// Created by zhouchang on 19-4-15.
//
/// \file wheel_factor.hpp
/// \brief Ceres factor for preintegrated wheel measurement.
/// \author RU WANG, rockidog@live.com
/// \date 2019-04-10
#ifndef FUSION_LOCALIZATION_WHEEL_FACTOR_HPP
#define FUSION_LOCALIZATION_WHEEL_FACTOR_HPP

#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include "./wheel_integrator.hpp"
#include "./math.hpp"

namespace loam {

    class WheelFactor : public ceres::SizedCostFunction<9, 10, 10> {
    public:
        WheelFactor(const WheelIntegral& integral)
                : integral_(integral)
        { }

        virtual ~WheelFactor() override = default;

        virtual bool Evaluate(
                double const* const* two_states,
                double* residual,
                double** jacobians) const override {
            using Eigen::Map;
            using Eigen::Quaterniond;
            using Eigen::Vector3d;
            using Eigen::Matrix3d;
            using Matrix9d = Eigen::Matrix<double, 9, 9>;
            using Vector9d = Eigen::Matrix<double, 9, 1>;
            using RowMatrix9x10d = Eigen::Matrix<double, 9, 10, Eigen::RowMajor>;

            Map<Quaterniond const> q_i(two_states[0]);
            Map<Vector3d    const> p_i(two_states[0]+4);
            Map<Vector3d    const> v_i(two_states[0]+7);

            Map<Quaterniond const> q_j(two_states[1]);
            Map<Vector3d    const> p_j(two_states[1]+4);
            Map<Vector3d    const> v_j(two_states[1]+7);

            CHECK_S(not q_i.coeffs().hasNaN()) << q_i.coeffs().transpose();
            CHECK_S(not p_i.hasNaN())          << p_i.transpose();
            CHECK_S(not v_i.hasNaN())          << v_i.transpose();
            CHECK_S(not q_j.coeffs().hasNaN()) << q_j.coeffs().transpose();
            CHECK_S(not p_j.hasNaN())          << p_j.transpose();
            CHECK_S(not v_j.hasNaN())          << v_j.transpose();

            Map<Vector9d> r_ij(residual);
            //r_ij.setZero();

            double d_t = integral_.mean.t;
            const Matrix9d& d_I_cov = integral_.covariance;

            /* the estimates */
            Quaterniond q_ij_hat = q_i.conjugate() * q_j;
            Vector3d    p_ij_hat = q_i.conjugate() * (p_j - p_i);
            Vector3d    v_j_hat  = q_j.conjugate() * v_j;

            /* the measurements */
            const Quaterniond& q_ij_tilde = integral_.mean.q;
            const Vector3d&    p_ij_tilde = integral_.mean.p;
            const Vector3d&    v_j_tilde  = integral_.mean.v;

            CHECK_S(not q_ij_hat.coeffs().hasNaN())   << q_ij_hat.coeffs().transpose();
            CHECK_S(not p_ij_hat.hasNaN())            << p_ij_hat.transpose();
            CHECK_S(not v_j_hat.hasNaN())             << v_j_hat.transpose();
            CHECK_S(not q_ij_tilde.coeffs().hasNaN()) << q_ij_tilde.coeffs().transpose();
            CHECK_S(not p_ij_tilde.hasNaN())          << p_ij_tilde.transpose();
            CHECK_S(not v_j_tilde.hasNaN())           << v_j_tilde.transpose();

            /* the residuals, i.e. the discrepency between the estimates and measurements */
            r_ij.segment<3>(0)  = Logmap(q_ij_tilde.conjugate() * q_ij_hat);
            r_ij.segment<3>(3)  = p_ij_hat - p_ij_tilde;
            r_ij.segment<3>(6)  = v_j_hat - v_j_tilde;

            CHECK_S(not r_ij.hasNaN()) << r_ij.transpose();

            Matrix9d sqrt_info = d_I_cov.inverse().llt().matrixU();

            if (jacobians) {
                Matrix3d Jr_inv = RightJacobianInverse(r_ij.head<3>(), 1.0e-10);

                if (jacobians[0]) {
                    Matrix9d J_i = Matrix9d::Zero();

                    J_i.block<3, 3>(0, 0) = -Jr_inv*q_ij_hat.conjugate().matrix();  /* jacobians of r(ΔR) wrt. φi */
                    J_i.block<3, 3>(3, 0) = SkewSymmMatrix(p_ij_hat);               /* jacobians of r(Δp) wrt. φi */
                    J_i.block<3, 3>(3, 3) = -Matrix3d::Identity();  //-q_i.conjugate().matrix(); //  /* jacobians of r(Δp) wrt. pi */

                    sqrt_info.applyThisOnTheLeft(J_i);
                    Map<RowMatrix9x10d> mapped_jacobian(jacobians[0]);
                    mapped_jacobian.leftCols(9) = J_i;
                    mapped_jacobian.rightCols(1).setZero();
                }

                if (jacobians[1]) {
                    Matrix9d J_j = Matrix9d::Zero();

                    J_j.block<3, 3>(0, 0) = Jr_inv;                    /* jacobians of r(ΔR) wrt. φj */
                    J_j.block<3, 3>(3, 3) = q_ij_hat.matrix(); //q_i.conjugate().matrix();// /* jacobians of r(Δp) wrt. pj */
                    J_j.block<3, 3>(6, 0) = SkewSymmMatrix(v_j_hat);   /* jacobians of r(Δv) wrt. φj */
                    J_j.block<3, 3>(6, 6) = q_j.conjugate().matrix();  /* jacobians of r(Δv) wrt. vj */

                    sqrt_info.applyThisOnTheLeft(J_j);
                    Map<RowMatrix9x10d> mapped_jacobian(jacobians[1]);
                    mapped_jacobian.leftCols(9) = J_j;
                    mapped_jacobian.rightCols(1).setZero();
                }
            }

            sqrt_info.applyThisOnTheLeft(r_ij);

            return true;
        }

    private:
        const WheelIntegral integral_;
    };

}  /* namespace loam */

#endif //FUSION_LOCALIZATION_WHEEL_FACTOR_HPP
