load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_pointcloud_pcd_exporter",
    srcs = [
        "pcd_exporter.cc",
    ],
    hdrs = [
        "pcd_exporter.h",
    ],
    deps = [
        "//modules/drivers/velodyne/pointcloud/util:velodyne_pointcloud_util",
        "@ros//:ros_common",
        "@eigen//:eigen",
        "@pcl//:pcl"
    ],
)