load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_pointcloud_util",
    srcs = [
        "util.cc",
    ],
    hdrs = [
        "util.h",
        "point_types.h",
        "const_variables.h"
    ],
    deps = [
        "@ros//:ros_common",
    ],
)
