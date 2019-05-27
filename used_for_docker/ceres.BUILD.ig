package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "ceres",
    srcs = [
        "lib/libceres.a",
		],
    hdrs = glob([
        "include/ceres.h",
        "include/**/*.h",
        "include/**/**/*.h",
    ]),
    includes = ["include"],
	  include_prefix = "ceres",
    linkopts = [
    "-fopenmp",
    "-lpthread",
    "-lopenblas",
    "-lcholmod",
    "-lglog",
    "-lcxsparse",
    ],
    deps = ["//external:gflags",
    "@eigen",],
)
