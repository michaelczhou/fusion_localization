//
// Created by zhouchang on 19-4-15.
//
/// \file math.hpp
/// \brief Mathematics.
/// \author RU WANG, rockidog@live.com
/// \date 2019-04-09
#ifndef FUSION_LOCALIZATION_MATH_HPP
#define FUSION_LOCALIZATION_MATH_HPP

#include <Eigen/Eigen>

namespace loam {

    template<typename derived_t>
    auto SkewSymmMatrix(const Eigen::MatrixBase<derived_t>& phi)
    -> Eigen::Matrix<typename derived_t::RealScalar, 3, 3> {
        return (Eigen::Matrix<typename derived_t::RealScalar, 3, 3>()
                <<      0.0, -phi.z(),  phi.y(),
                phi.z(),      0.0, -phi.x(),
                -phi.y(),  phi.x(),      0.0).finished();
    }

    inline Eigen::Quaterniond Expmap(const Eigen::Vector3d& phi) {
        if (phi.norm() < 1.0e-08)
            return Eigen::Quaterniond(1.0, phi.x()*0.5, phi.y()*0.5, phi.z()*0.5);
        return Eigen::Quaterniond(Eigen::AngleAxisd(phi.norm(), phi.normalized()));
    }

    inline Eigen::Vector3d Logmap(const Eigen::Quaterniond& q) {
        Eigen::AngleAxisd aa(q);
        return Eigen::Vector3d(aa.angle() * aa.axis());
    }

    template<typename derived_t>
    Eigen::Matrix3d RightJacobian(
            const Eigen::MatrixBase<derived_t>& phi, double double_epsilon) {
        using Eigen::Matrix3d;
        Matrix3d I = Matrix3d::Identity();
        double phi_norm2 = phi.squaredNorm();
        if (phi_norm2 <= double_epsilon)
            return I - 0.5 * SkewSymmMatrix(phi);
        double phi_norm = sqrt(phi_norm2);
        double phi_norm3 = phi_norm * phi_norm2;
        double cos_phi = cos(phi_norm);
        double sin_phi = sin(phi_norm);
        Matrix3d phi_hat = SkewSymmMatrix(phi);
        Matrix3d Jr = I - (1.0-cos_phi)/phi_norm2*phi_hat +
                      (phi_norm-sin_phi)/phi_norm3*phi_hat*phi_hat;
        return Jr;
    }

    template<typename derived_t>
    Eigen::Matrix3d RightJacobianInverse(
            const Eigen::MatrixBase<derived_t>& phi, double double_epsilon) {
        using Eigen::Matrix3d;
        Matrix3d I = Matrix3d::Identity();
        double phi_norm2 = phi.squaredNorm();
        if (phi_norm2 <= double_epsilon)
            return I + 0.5 * SkewSymmMatrix(phi);
        double phi_norm = sqrt(phi_norm2);
        double cos_phi = cos(phi_norm);
        double sin_phi = sin(phi_norm);
        Matrix3d phi_hat = SkewSymmMatrix(phi);
        Matrix3d Jr_inv = I + 0.5*SkewSymmMatrix(phi) +
                          (1.0/phi_norm2+0.5*(1.0+cos_phi)/(phi_norm*sin_phi)) *
                          phi_hat*phi_hat;
        return Jr_inv;
    }

}  /* namespace loam */

#endif //FUSION_LOCALIZATION_MATH_HPP
