//
// Created by zhouchang on 19-4-15.
//
/// \file wheel_integrator.hpp
/// \brief Wheel preintegration.
/// \author RU WANG, rockidog@live.com
/// \date 2019-04-10
#ifndef FUSION_LOCALIZATION_WHEEL_INTEGRATOR_HPP
#define FUSION_LOCALIZATION_WHEEL_INTEGRATOR_HPP

#include <array>
#include <iostream>
#include <iomanip>
#include <vector>
#include <Eigen/Eigen>
#include "./state_parameterization.hpp"

namespace loam {

/// \brief Raw wheel measurement.
///
/// - t: timestamp [second]
/// - r: gyroscope reading [rad/s, rad/s, rad/s]
/// - v: velocity reading [m/s, m/s, m/s]
    struct RawWheel {
        RawWheel() = default;

//        RawWheel(const std::vector<std::pair<double, Eigen::Vector3d>> &velVector,
//                 const std::vector<std::pair<double, Eigen::Vector3d>> &gyrVector)
//            : velVector(velVector), gryVector(gryVector){}

        RawWheel(double t,
                 double vx, double vy, double vz,
                 double rx, double ry, double rz)
            : t(t), arr({{vx, vy, vz, rx, ry, rz}})
        { }

        RawWheel(double t, const std::array<double, 6>& arr)
            : t(t), arr(arr)
        { }

        RawWheel(const RawWheel& other) : t(other.t), arr(other.arr)
        { }

        RawWheel& operator=(const RawWheel& other) {
            if (this != &other) {
                t = other.t;
                arr = other.arr;
                new (&v) Eigen::Map<Eigen::Vector3d const>(arr.data() + 0);
                new (&w) Eigen::Map<Eigen::Vector3d const>(arr.data() + 3);
            }
            return (*this);
        }

        double t = 0.0;
        std::array<double, 6> arr = { {0.0} };

        Eigen::Map<Eigen::Vector3d const> v = Eigen::Map<Eigen::Vector3d const>(arr.data() + 0);
        Eigen::Map<Eigen::Vector3d const> w = Eigen::Map<Eigen::Vector3d const>(arr.data() + 3);
    };

    inline std::ostream& operator<<(std::ostream& os, const RawWheel& wheel_reading) {
        os << std::fixed << std::setprecision(4) << wheel_reading.t;
        for (double v : wheel_reading.arr)
            os << " " << std::scientific << std::setprecision(4) << v;
        return os;
    }

    inline std::istream& operator>>(std::istream& is, RawWheel& wheel_reading) {
        is >> wheel_reading.t;
        for (double& v : wheel_reading.arr)
            is >> v;
        return is;
    }

/// \brief The preintegrated IMU measurement.
    struct WheelIntegral {
        WheelIntegral(
                const std::vector<RawWheel>& wheel_win,
                const State& integral_mean,
                const Eigen::Matrix<double, 9, 9>& integral_cov)
                :
                wheels(wheel_win), mean(integral_mean), covariance(integral_cov)
        { }

        std::vector<RawWheel> wheels;
        State mean;
        Eigen::Matrix<double, 9, 9> covariance;
    };

    class WheelPreintegrator {
    public:
        static WheelIntegral Integrate(
                const std::vector<RawWheel>& wheels, const Eigen::Matrix<double, 6, 1>& gawn_cov);
        static State PropagateState(const State& old, const WheelIntegral& integral);
    };

}  /* namespace loam */

#endif //FUSION_LOCALIZATION_WHEEL_INTEGRATOR_HPP
