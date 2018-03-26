load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_pointcloud_convert_calibration",
    srcs = [
        "calibration.cc",
        "online_calibration.cc"
    ],
    hdrs = [
        "calibration.h",
        "online_calibration.h"
    ],
    deps = [    
        "@yaml_cpp//:yaml",
        "@ros//:ros_common",
    ],
)