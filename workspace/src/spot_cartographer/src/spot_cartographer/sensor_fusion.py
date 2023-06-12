import enum
import rospy
import functools
import tf2_ros
import numpy as np
import std_msgs.msg
import tf2_sensor_msgs
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from typing import (
    Optional,
    Sequence,
    NoReturn,
)


class CrucibleForSensorFusion:  # Not technically sensor fusion, but kind of.

    class DepthDirection(enum.Enum):
        LEFT        = "left"
        RIGHT       = "right"
        FRONT_LEFT  = "frontleft"
        FRONT_RIGHT = "frontright"

    def __init__(self) -> None:

        def update_depth_cloud_data(
            direction: str,
            cloud: PointCloud2,
        ) -> None:
            self.depth_clouds[direction] = cloud

        def update_orb_cloud_data(cloud: PointCloud2) -> None:
            self.orb_cloud = cloud

        self.depth_clouds = {
            direction: None for direction, _
            in CrucibleForSensorFusion.DepthDirection
        }
        self.depth_subscribers = {
            direction: rospy.Subscriber(
                f"/spot/depth/{sensor}/image",
                PointCloud2,
                functools.partial(update_depth_cloud_data, direction)
            )
            for direction, sensor in CrucibleForSensorFusion.DepthDirection
        }
        self.orb_cloud = None
        self.orb_subscriber = rospy.Subscriber(
            "orb_slam3/all_points",
            PointCloud2,
            update_orb_cloud_data,
        )
        self.cloud_publisher = rospy.Publisher(
            "/spotters/cartographer/all_points",
            PointCloud2,
            queue_size=1
        )
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def initiate_fusion(self) -> NoReturn:
        fusion_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            fusion_rate.sleep()
            self.reorient_and_fuse_clouds()

    def reorient_and_fuse_clouds(self) -> None:
        clouds = [
            reoriented for cloud in self.retrieve_active_depth_clouds()
            if (reoriented := self.reorient_depth_cloud(cloud)) is not None
        ]
        self.cloud_publisher.publish(fuse_clouds(clouds, "origin"))

    def retrieve_active_depth_clouds(self) -> Sequence[PointCloud2]:

        def is_recent(cloud: PointCloud2) -> bool:
            elapsed = eval - cloud.header.stamp  # TODO: Check that this works?
            return True if elapsed.to_sec() < 2.0 else False

        eval = rospy.Time.now()
        active_depth_clouds = [
            cloud for direction, _ in CrucibleForSensorFusion.DepthDirection
            if (cloud := self.depth_clouds[direction]) is not None and is_recent(cloud)
        ]
        return active_depth_clouds

    def reorient_depth_cloud(
        self,
        point_cloud: PointCloud2,
    ) -> Optional[PointCloud2]:
        target_frame_id: str = "origin"
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame_id,
                point_cloud.header.frame_id,
                point_cloud.header.time,
                rospy.Duration(0.5),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as error:
            rospy.logwarn(
                f"A lookup between {target_frame_id} and {point_cloud.header.frame_id} failed with {error}"
            )
        return tf2_sensor_msgs.do_transform_cloud(point_cloud, transform)


def fuse_clouds(clouds: Sequence[PointCloud2], target_frame_id: str) -> PointCloud2:
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = target_frame_id
    return pc2.create_cloud_xyz32(header, extract_and_fuse_points(clouds))


# NOTE: Might not work. Fingers crossed.
def extract_and_fuse_points(clouds: Sequence[PointCloud2]) -> PointCloud2:
    fields = ("x", "y", "z")
    points = (pc2.read_points(cloud, fields, skip_nans=True) for cloud in clouds)
    return np.concatenate((np.fromiter(cloud) for cloud in points), axis=0)


def main() -> NoReturn:
    rospy.init_node("SensorCrucible")
    crucible = CrucibleForSensorFusion()
    crucible.initiate_fusion()


if __name__ == "__main__":
    main()
