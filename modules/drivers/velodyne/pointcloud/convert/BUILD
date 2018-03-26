load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velodyne_pointcloud_convert",
    srcs = [
        "convert.cc",
    ],
    hdrs = [
        "convert.h",
    ],
    deps = [
        "//modules/drivers/velodyne/pointcloud/util:velodyne_pointcloud_util",
        "//modules/drivers/velodyne/pointcloud/convert/calibration:velodyne_pointcloud_convert_calibration",
        "//modules/drivers/velodyne/pointcloud/convert/parser:velodyne_pointcloud_convert_parser",
        "//modules/drivers/velodyne/proto:velodyne_conf_proto",
        "@ros//:ros_common",
        "@eigen//:eigen",
        "@pcl//:pcl"
    ],
)