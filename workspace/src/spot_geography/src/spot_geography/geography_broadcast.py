import rospy
from numpy.typing import NDArray
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import (
    PoseStamped,
    PoseArray,
    Pose
)
from spot_geography.pipelines.point_cloud_pipeline import PointCloudProcessingFacility
from spot_geography.pipelines.occupancy_computations import (
    generate_occupancy_grid,
    generate_positions_data,
)
from typing import (
    Optional,
    Tuple,
)


class GeographyBroadcastStation:
    def __init__(self) -> None:
        self.cloud_facility = PointCloudProcessingFacility()
        self.positions_channel = rospy.Publisher("/spotters/mapping/pos", PoseArray, queue_size=1)
        self.occupancy_channel = rospy.Publisher("/spotters/mapping/map", OccupancyGrid, queue_size=1)

    def initiate_broadcast(self) -> None:

        def broadcast(grid: OccupancyGrid) -> None:
            array, positions = self.generate_geography(grid)
            self.broadcast_geography(array, grid, positions)

        rospy.Subscriber('/projected_map', OccupancyGrid, broadcast)
        rospy.spin()

    def generate_geography(self, grid: OccupancyGrid) -> Tuple[NDArray, Optional[PoseArray]]:

        def update_robot_position(pose: PoseStamped) -> None:
            nonlocal position
            position = pose.pose

        position: Optional[Pose] = None
        pose = rospy.wait_for_message("orb_slam3/camera_pose", PoseStamped, timeout=4)
        update_robot_position(pose)
        occupancy = generate_occupancy_grid(grid)
        positions = generate_positions_data(grid.info.width, grid.info.height, position)
        return occupancy, positions

    def broadcast_geography(
        self,
        data: NDArray,
        grid: OccupancyGrid,
        positions: Optional[PoseArray] = None
    ) -> None:
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = grid.header.frame_id
        if positions is not None:
            positions.header = header
            self.positions_channel.publish(positions)
        occupancy = OccupancyGrid()
        occupancy.data = data
        occupancy.header = header
        occupancy.info.map_load_time = rospy.Time.now()
        occupancy.info.resolution = grid.info.resolution
        occupancy.info.width = grid.info.width
        occupancy.info.height = grid.info.height
        occupancy.info.origin = grid.info.origin
        self.occupancy_channel.publish(occupancy)


def main() -> None:
    rospy.init_node("spotters_geography_broadcast", log_level=rospy.DEBUG)
    station = GeographyBroadcastStation()
    station.initiate_broadcast()


if __name__ == "__main__":
    main()
