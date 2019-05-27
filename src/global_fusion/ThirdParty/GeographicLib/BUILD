load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "GeographicLib",
    srcs = [
    "src/LocalCartesian.cpp",
    "src/Geocentric.cpp",
    "src/Math.cpp",
    ],
    hdrs = [
    "include/Config.h",
    "include/Constants.hpp",
    "include/Geocentric.hpp",
    "include/LocalCartesian.hpp",
    "include/Math.hpp",
    ],
    deps = [
        "//modules/common:log",
        "@ceres",
        "@pcl",
        "@eigen",
        "@opencv3//:common",
        "@ros//:ros_common",
    ],
)

cpplint()
