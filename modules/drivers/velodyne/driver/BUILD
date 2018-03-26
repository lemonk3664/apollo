load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_driver",
    srcs = [
        "driver.cc",
        "driver16.cc",
        "driver64.cc"
    ],
    hdrs = [
        "driver.h",
    ],
    deps = [
        "@ros//:ros_common",
        "//modules/drivers/velodyne/driver/input:velodyne_driver_input",
        "//modules/drivers/velodyne/proto:velodyne_conf_proto",
    ],
)