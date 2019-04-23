//
// Created by zhouchang on 19-4-18.
//

#ifndef FUSION_LOCALIZATION_ESTIMATOR_HPP
#define FUSION_LOCALIZATION_ESTIMATOR_HPP

#include "./state_parameterization.hpp"
#include <opencv2/core/eigen.hpp>

class Estimator{
public:
    Estimator();

    enum SolverFlag
    {
        INITIAL, // 还未成功初始化
        NON_LINEAR // 已成功初始化，正处于紧耦合优化状态
    };

    SolverFlag solver_flag; // 系统的状态
    Eigen::Vector3d g;

    //外参
    double td; // wheel数据与laser数据时间戳的偏移值
    Eigen::Vector3d acc_0, gyr_0; // 最近一次接收到的wheel数据

    //ceres优化参数块

};
#endif //FUSION_LOCALIZATION_ESTIMATOR_HPP
