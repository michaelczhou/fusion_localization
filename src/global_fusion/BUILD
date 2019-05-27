load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "global_fusion_lib",
    srcs = [
    "src/globalOptNode.cpp",
    "src/globalOpt.cpp",
    ],
    hdrs = [
    "src/Factors.h",
    "src/globalOpt.h",
    "src/tic_toc.h",
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
        "//modules/fusion_localization/src/global_fusion/ThirdParty/GeographicLib:GeographicLib",

        "@pcl",
        "@eigen",
        "@opencv3//:common",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "global_fusion",
    srcs = [
        "src/globalOptNode.cpp",
	    "src/globalOpt.cpp",
    ],
    linkopts = [
        "-lpthread",
    ],
    deps = [
        ":global_fusion_lib",
        "//modules/common:log",
    ],
)


cpplint()
