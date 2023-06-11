import math
import time
import itertools
import rospy
import rtree
import shapely
import shapely.ops
import numpy as np
from numpy.typing import NDArray
from typing import (
    Sequence,
    Optional,
    Tuple,
    Bool,
)
import scipy.spatial as spatial
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import (
    PoseStamped,
    PoseArray,
    Pose,
    Point,
)


class GeographyBroadcastStation:
    def __init__(self) -> None:
        rospy.init_node("geography_broadcast_station", anonymous=True)
        self.positions_channel = rospy.Publisher("/spotters/mapping/pos", PoseArray, queue_size=10)
        self.occupancy_channel = rospy.Publisher("/spotters/mapping/map", OccupancyGrid, queue_size=10)

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
        rospy.Subscriber("orb_slam3/camera_pose", PoseStamped, update_robot_position, queue_size=1)
        occupancy = generate_occupancy_grid(grid)
        positions = generate_positions_data(grid.data.width, grid.data.height, position)
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
        generated = OccupancyGrid()
        generated.data = data
        generated.header = header
        generated.info.map_load_time = rospy.Time.now()
        generated.info.resolution = grid.info.resolution
        generated.info.width =  grid.info.width
        generated.info.height = grid.info.height
        generated.info.origin = grid.info.origin
        self.occupancy_channel.publish(generated)


def generate_occupancy_grid(grid: OccupancyGrid) -> NDArray[np.int8]:
    values = np.array(grid).reshape((grid.info.height, grid.info.width))
    coordinates = np.column_stack(np.where(values == 100))
    alphas = generate_occlusion_polygons(coordinates)
    return construct_occupancy_grid(grid, alphas)


def generate_positions_data(width: int, height: int, pose: Pose) -> Optional[PoseArray]:
    if pose is None:
        return
    cartesian = np.rot90(np.stack(np.indices((height, width)), axis=2))
    cartesian[:,:,0] -= cartesian.shape[1] // 2
    cartesian[:,:,1] -= cartesian.shape[0] // 2
    poses: Sequence[Pose] = [pose]
    try:
        indices = np.stack(np.where((cartesian == [pose.x, pose.y]).all(axis=2)), axis=1)[0]
        position = Point()
        position.x = indices[0]
        position.y = indices[1]
        position.z = -1
        index = Pose()
        index.position = position
        index.orientation = pose.orientation
        poses.append(index)
    except IndexError:
        poses.append(None)
    array: PoseArray = PoseArray()
    array.poses = poses
    return array


def construct_occupancy_grid(
    grid: OccupancyGrid,
    alphas: shapely.GeometryCollection,
) -> NDArray[np.uint8]:
    index = rtree.index.Index()
    shapes = list()
    # Initialise the tree with all shapes.
    for ordinal, alpha in enumerate(alphas.geoms):
        index.insert(ordinal, alpha.bounds)
        shapes.append(alpha)
    length, height = grid.info.width, grid.info.height
    # Create blank grid and iterate over subgrids, labelling if occupied.
    array = np.zeros((height, length), np.uint8)
    for rank, file in itertools.product(
        np.arange(0, height, 3),
        np.arange(0, length, 3)
    ):
        point = shapely.Point(rank, file)
        indices = list(index.intersection((point.x, point.y, point.x, point.y)))
        for alpha in indices:
            if shapes[alpha].contains(point):
                array[
                    max(0, rank - 1) : min(rank + 2, length),
                    max(0, file - 1) : min(file + 2, height),
                ] = 100
                break
    return array.flatten()


def generate_occlusion_polygons(coordinates: NDArray, alpha: float = 1) -> None:

    def include_edge(vertex1: Sequence[int], vertex2: Sequence[int]) -> None:
        edge: Sequence[int] = (vertex1, vertex2)
        if edge in edges or reversed(edge) in edges:
            return
        edges.add(edge)
        points.append(feature_coordinates[[vertex1, vertex2]])

    feature_points = [shapely.Point(rank, file) for rank, file in coordinates]
    if len(feature_points) <= 3:
        return shapely.MultiPoint(feature_points).convex_hull
    feature_coordinates = np.array([point.coords[0] for point in feature_points])
    triangles = spatial.Delaunay(feature_coordinates)
    edges, points = set(), list()
    filter_start=time.time()
    for vertex1_index, vertex2_index, vertex3_index in triangles.vertices:
        coordinate1 = feature_coordinates[vertex1_index]
        coordinate2 = feature_coordinates[vertex2_index]
        coordinate3 = feature_coordinates[vertex3_index]
        if filter_triangle(coordinate1, coordinate2, coordinate3, alpha):
            include_edge(vertex1_index, vertex2_index)
            include_edge(vertex2_index, vertex3_index)
            include_edge(vertex3_index, vertex1_index)
    shapely_start = time.time()
    polygons = shapely.ops.polygonize(shapely.MultiLineString(points))
    shapely_time = time.time() - shapely_start
    filter_time = time.time() - filter_start - shapely_time
    print(f"Created shapely polygons in {shapely_time:.2f}.")
    print(f"Filtered triangle in {filter_time:.2f}.")
    return shapely.unary_union(polygons).buffer(2)


def filter_triangle(
    vertex1: Sequence[int],
    vertex2: Sequence[int],
    vertex3: Sequence[int],
    alpha: float,
) -> bool:

    def calculate_distance(coordinate1, coordinate2) -> float:
        return math.sqrt(  # TODO: Verify absolute value is correct.
            abs(coordinate1[0] - coordinate2[0]) ** 2 +
            abs(coordinate1[1] - coordinate2[1]) ** 2
        )

    length1 = calculate_distance(vertex1, vertex2)
    length2 = calculate_distance(vertex2, vertex3)
    length3 = calculate_distance(vertex3, vertex1)
    half_peri = (length1 + length2 + length3) / 2
    area = math.sqrt(
        half_peri
        * (half_peri - length1)
        * (half_peri - length2)
        * (half_peri - length3)
    )
    return (length1 * length2 * length3) / (4 * area) < 1 / alpha


def main() -> None:
    station = GeographyBroadcastStation()
    station.initiate_broadcast()


if __name__ == "__main__":
    main()
