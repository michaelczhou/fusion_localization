load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fusion_slam_lib",
    srcs = [
    "src/estimation/wheel_integrator.cc",
    "src/utility/utility.cpp",
    "external/loguru.cpp",
    ],
    hdrs = [
    "src/estimation/lidar_factor.hpp",
    "src/estimation/math.hpp",
    "src/estimation/estimator.hpp",
    "src/estimation/state_parameterization.hpp",
    "src/estimation/wheel_factor.hpp",
    "src/estimation/wheel_integrator.hpp",
    "src/utility/kitti_adapter.hpp",
    "src/utility/utility.h",
    "external/loguru.hpp",
    "src/utility/sockvis.h",
    ],
    deps = [
        "//modules/common/proto:common_proto",
        "//modules/msgs/canbus/proto:canbus_proto",
        "//modules/msgs/drivers/camera/proto:image_proto",
        "//modules/msgs/drivers/novatel/proto:gnss_proto",
        "//modules/msgs/drivers/novatel/proto:ins_proto",
        "//modules/msgs/localization/proto:localization_proto",
        "//modules/msgs/drivers/lidar/proto:point_cloud_proto",
        "//modules/msgs/drivers/gnss/proto:gnss_ins_proto",

        "@ceres",
        "@pcl",
        "@eigen",
        "@opencv3//:common",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "laser_regist",
    srcs = [
        "src/node/lidar_registration.cc",
    ],
    linkopts = [
        "-lpthread",
    ],
    deps = [
        ":fusion_slam_lib",
        "//modules/common:log",
    ],
)

cc_binary(
    name = "laser_odo",
    srcs = [
        "src/node/lidar_odometry.cc",
    ],
    linkopts = [
        "-lpthread",
    ],
    deps = [
        ":fusion_slam_lib",
        "//modules/common:log",
    ],
)

cc_binary(
    name = "laser_map",
    srcs = [
        "src/node/lidar_mapping.cc",
    ],
    linkopts = [
        "-lpthread",
    ],
    deps = [
        ":fusion_slam_lib",
        "//modules/common:log",
    ],
)

cc_binary(
    name = "slam_display_node",
    srcs = [
        "src/slam_display/slam_display.cpp",
    ],
    deps = [":fusion_slam_lib"],
)


cpplint()
