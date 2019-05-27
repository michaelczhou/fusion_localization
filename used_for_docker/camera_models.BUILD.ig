load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fusion_slam_camera_models_lib",
    srcs = [
        "src/calib/CameraCalibration.cc",
        "src/camera_models/Camera.cc",
        "src/camera_models/CameraFactory.cc",
        "src/camera_models/CataCamera.cc",
        "src/camera_models/CostFunctionFactory.cc",
        "src/camera_models/EquidistantCamera.cc",
        "src/camera_models/PinholeCamera.cc",
        "src/camera_models/PinholeFullCamera.cc",
        "src/camera_models/ScaramuzzaCamera.cc",
        "src/chessboard/Chessboard.cc",
        "src/gpl/EigenQuaternionParameterization.cc",
        "src/gpl/gpl.cc",
        "src/sparse_graph/Transform.cc",
    ],
    hdrs = [
        "include/camodocal/calib/CameraCalibration.h",
        "include/camodocal/camera_models/Camera.h",
        "include/camodocal/camera_models/CameraFactory.h",
        "include/camodocal/camera_models/CataCamera.h",
        "include/camodocal/camera_models/CostFunctionFactory.h",
        "include/camodocal/camera_models/EquidistantCamera.h",
        "include/camodocal/camera_models/PinholeCamera.h",
        "include/camodocal/camera_models/PinholeFullCamera.h",
        "include/camodocal/camera_models/ScaramuzzaCamera.h",
        "include/camodocal/chessboard/Chessboard.h",
        "include/camodocal/chessboard/ChessboardCorner.h",
        "include/camodocal/chessboard/ChessboardQuad.h",
        "include/camodocal/chessboard/Spline.h",
        "include/camodocal/gpl/EigenQuaternionParameterization.h",
        "include/camodocal/gpl/EigenUtils.h",
        "include/camodocal/gpl/gpl.h",
        "include/camodocal/sparse_graph/Transform.h",
    ],
    deps = [
        "@eigen",
        "@opencv3//:common",
        "@ros//:ros_common",
    ],
)

cpplint()
