load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_pointcloud_convert_parser",
    srcs = [
        "velodyne_parser.cc",
        "velodyne16_parser.cc",
        "velodyne64_parser.cc",
    ],
    hdrs = [
        "velodyne_parser.h",
    ],
    deps = [
        "@ros//:ros_common",
        "//modules/drivers/velodyne/pointcloud/util:velodyne_pointcloud_util",
        "//modules/drivers/velodyne/pointcloud/convert/calibration:velodyne_pointcloud_convert_calibration",
        "@eigen//:eigen",
        "@pcl//:pcl"
    ],
)