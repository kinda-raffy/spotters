import rospy
import tf2_ros
from numpy.typing import NDArray
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
from spot_geography.pipelines.point_cloud_pipeline import PointCloudProcessingFacility
from spot_geography.pipelines.occupancy_computations import generate_occupancy_grid
from typing import (
    Tuple,
    Optional,
    NoReturn,
)


class GeographyBroadcastStation:
    def __init__(self) -> None:
        self.cloud_facility = PointCloudProcessingFacility()
        self.position_channel = rospy.Publisher(
            "/spotters/geography/pos",
            PoseStamped,
            queue_size=1,
        )
        self.occupancy_channel = rospy.Publisher(
            "/spotters/geography/map",
            OccupancyGrid,
            queue_size=1,
        )
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def initiate_broadcast(self) -> NoReturn:

        def broadcast(grid: OccupancyGrid) -> None:
            array, position = self.generate_geography(grid)
            self.broadcast_geography(array, grid, position)

        rospy.Subscriber('/projected_map', OccupancyGrid, broadcast)
        rospy.spin()

    def generate_geography(self, grid: OccupancyGrid) -> Tuple[NDArray, PoseStamped]:
        pose = rospy.wait_for_message(
            "orb_slam3/camera_pose",
            PoseStamped,
            timeout=5,
        )
        occupancy = generate_occupancy_grid(grid)
        return occupancy, pose

    def broadcast_geography(
        self,
        data: NDArray,
        grid: OccupancyGrid,
        pose: PoseStamped,
    ) -> None:

        def broadcast_grid() -> None:
            occupancy = OccupancyGrid()
            occupancy.data = data
            occupancy.header = header
            occupancy.info.map_load_time = rospy.Time.now()
            occupancy.info.resolution = grid.info.resolution
            occupancy.info.width = grid.info.width
            occupancy.info.height = grid.info.height
            occupancy.info.origin = grid.info.origin
            self.occupancy_channel.publish(occupancy)

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = grid.header.frame_id
        self.position_channel.publish(self.prepare_pose_broadcast(pose))
        broadcast_grid()

    def prepare_pose_broadcast(self, pose: PoseStamped) -> Optional[PoseStamped]:
        try:
            trans = self.tf_buffer.lookup_transform(
                "origin",
                "camera",
                rospy.Time(),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logwarn("Your princess is in another castle.")
        return PoseStamped(
            pose.header,
            Pose(
                trans.transform.translation,
                trans.transform.rotation,
            )
        )


def main() -> NoReturn:
    rospy.init_node("SpotGeography", log_level=rospy.DEBUG)
    station = GeographyBroadcastStation()
    station.initiate_broadcast()


if __name__ == "__main__":
    main()
