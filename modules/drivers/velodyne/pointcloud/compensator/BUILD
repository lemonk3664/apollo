load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_pointcloud_compensator",
    srcs = [
        "compensator.cc",
    ],
    hdrs = [
        "compensator.h",
    ],
    deps = [
        "@ros//:ros_common",
        "//modules/drivers/velodyne/pointcloud/util:velodyne_pointcloud_util",
        "//modules/drivers/velodyne/proto:velodyne_conf_proto",
        "@eigen//:eigen",
        "@pcl//:pcl"
    ],
)