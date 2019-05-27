load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fusion_slam_global_fusion_lib",
    srcs = [
        "src/globalOpt.cpp",
    ],
    hdrs = [
        "ThirdParty/GeographicLib/include/Config.h",
        "ThirdParty/GeographicLib/include/Constants.hpp",
        "ThirdParty/GeographicLib/include/Geocentric.hpp",
        "ThirdParty/GeographicLib/include/LocalCartesian.hpp",
        "ThirdParty/GeographicLib/include/Math.hpp",
        "src/Factors.h",
        "src/globalOpt.h",
        "src/tic_toc.h",
    ],
    deps = [
        ":fusion_slam_geographic_lib",
        "@ceres",
        "@eigen",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "fusion_slam_geographic_lib",
    srcs = [
        "ThirdParty/GeographicLib/src/Geocentric.cpp",
        "ThirdParty/GeographicLib/src/LocalCartesian.cpp",
        "ThirdParty/GeographicLib/src/Math.cpp",
    ],
    hdrs = [
        "ThirdParty/GeographicLib/include/Config.h",
        "ThirdParty/GeographicLib/include/Constants.hpp",
        "ThirdParty/GeographicLib/include/Geocentric.hpp",
        "ThirdParty/GeographicLib/include/LocalCartesian.hpp",
        "ThirdParty/GeographicLib/include/Math.hpp",
    ],
    deps = [],
)

cc_binary(
    name = "global_fusion_node",
    srcs = ["src/globalOptNode.cpp"],
    deps = [
        ":fusion_slam_geographic_lib",
        ":fusion_slam_global_fusion_lib",
    ],
)

cpplint()
