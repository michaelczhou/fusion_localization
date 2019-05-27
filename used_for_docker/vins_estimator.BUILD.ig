load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fusion_slam_estimator_lib",
    srcs = [
        "src/estimator/estimator.cpp",
        "src/estimator/feature_manager.cpp",
        "src/estimator/parameters.cpp",
        "src/factor/marginalization_factor.cpp",
        "src/factor/pose_local_parameterization.cpp",
        "src/factor/projectionOneFrameTwoCamFactor.cpp",
        "src/factor/projectionTwoFrameOneCamFactor.cpp",
        "src/factor/projectionTwoFrameTwoCamFactor.cpp",
        "src/featureTracker/feature_tracker.cpp",
        "src/initial/initial_aligment.cpp",
        "src/initial/initial_ex_rotation.cpp",
        "src/initial/initial_sfm.cpp",
        "src/initial/solve_5pts.cpp",
        "src/utility/CameraPoseVisualization.cpp",
        "src/utility/utility.cpp",
        "src/utility/visualization.cpp",
    ],
    hdrs = [
        "src/estimator/estimator.h",
        "src/estimator/feature_manager.h",
        "src/estimator/parameters.h",
        "src/factor/imu_factor.h",
        "src/factor/integration_base.h",
        "src/factor/marginalization_factor.h",
        "src/factor/pose_local_parameterization.h",
        "src/factor/projectionOneFrameTwoCamFactor.h",
        "src/factor/projectionTwoFrameOneCamFactor.h",
        "src/factor/projectionTwoFrameTwoCamFactor.h",
        "src/featureTracker/feature_tracker.h",
        "src/initial/initial_alignment.h",
        "src/initial/initial_ex_rotation.h",
        "src/initial/initial_sfm.h",
        "src/initial/solve_5pts.h",
        "src/utility/CameraPoseVisualization.h",
        "src/utility/tic_toc.h",
        "src/utility/utility.h",
        "src/utility/visualization.h",
    ],
    deps = [
        "//modules/fusion_slam/camera_models:fusion_slam_camera_models_lib",
        "@ceres",
        "@eigen",
        "@opencv3//:common",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "kitti_mono_imu_gps_fusion_slam",
    srcs = [
        "src/KITTIMonoImuGpsTest.cpp",
    ],
    linkopts = [
        "-lpthread",
    ],
    deps = [
        ":fusion_slam_estimator_lib",
        "//modules/common:log",
    ],
)

cpplint()
