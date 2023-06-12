import rospy
from numpy.typing import NDArray
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from spot_geography.pipelines.point_cloud_pipeline import PointCloudProcessingFacility
from spot_geography.pipelines.occupancy_computations import generate_occupancy_grid
from typing import Tuple, NoReturn


class GeographyBroadcastStation:
    def __init__(self) -> None:
        self.cloud_facility = PointCloudProcessingFacility()
        self.position_channel = rospy.Publisher(
            "/spotters/mapping/pos",
            PoseStamped,
            queue_size=1,
        )
        self.occupancy_channel = rospy.Publisher(
            "/spotters/mapping/map",
            OccupancyGrid,
            queue_size=1,
        )

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
        self.position_channel.publish(pose)
        broadcast_grid()


def main() -> NoReturn:
    rospy.init_node("SpotGeography", log_level=rospy.DEBUG)
    station = GeographyBroadcastStation()
    station.initiate_broadcast()


if __name__ == "__main__":
    main()
