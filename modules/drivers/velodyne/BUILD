load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drivers_velodyne",
    srcs = [
        "velodyne.cc",
    ],
    hdrs = [
        "velodyne.h",
    ],
    deps = [
        "//modules/drivers/velodyne/proto:velodyne_conf_proto",
        "//modules/drivers/velodyne/gflags:velodyne_gflags",
        "//modules/drivers/velodyne/driver:velodyne_driver",
        "//modules/drivers/velodyne/pointcloud/convert:velodyne_pointcloud_convert",
        "//modules/drivers/velodyne/pointcloud/compensator:velodyne_pointcloud_compensator",
        "//modules/drivers/velodyne/pointcloud/dumper:velodyne_pointcloud_dumper",
        "//modules/drivers/velodyne/pointcloud/pcd_exporter:velodyne_pointcloud_pcd_exporter",
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/adapters:adapter",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/adapters:message_adapters",
        "//modules/common:apollo_app",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus/common:canbus_common",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "velodyne",
    srcs = ["main.cc"],
    deps = [
        ":drivers_velodyne",
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/monitor_log",
        "@ros//:ros_common",
    ],
)