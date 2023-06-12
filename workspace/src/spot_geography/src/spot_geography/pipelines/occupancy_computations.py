import rospy
import math
import time
import itertools
import rtree
import shapely
import shapely.ops
import numpy as np
from numpy.typing import NDArray
from typing import (
    Sequence,
)
from nav_msgs.msg import OccupancyGrid
from scipy.spatial import Delaunay


def generate_occupancy_grid(grid: OccupancyGrid) -> NDArray[np.int8]:
    start = time.time()
    rospy.logdebug(f"[Occupancy Grid] Finding occupied coordinates.")
    values = np.array(grid.data, np.int8).reshape((grid.info.height, grid.info.width))
    coordinates = np.column_stack(np.where(values == 100))
    alphas = generate_occlusion_polygons(coordinates)
    rospy.logdebug(f"[Occupancy Grid] Found occupied coordinates in {time.time() - start}.")
    return construct_occupancy_grid(grid, alphas)


def construct_occupancy_grid(
    grid: OccupancyGrid,
    alphas: shapely.GeometryCollection,
) -> NDArray[np.uint8]:
    start = time.time()
    rospy.logdebug(f"[Occupancy Grid] Generating occupancy grid.")
    index = rtree.index.Index()
    shapes = list()
    # Initialise the tree with all shapes.
    if isinstance(alphas, shapely.Polygon):
        index.insert(0, alphas.bounds)
        shapes.append(alphas)
    else:
        for ordinal, alpha in enumerate(alphas.geoms):
            index.insert(ordinal, alpha.bounds)
            shapes.append(alpha)
    length, height = grid.info.width, grid.info.height
    # Create blank grid and iterate over subgrids, labelling if occupied.
    array = np.zeros((height, length), np.uint8)
    for rank, file in itertools.product(
        np.arange(0, height, 3),
        np.arange(0, length, 3),
    ):
        point = shapely.Point(rank, file)
        indices = list(index.intersection((point.x, point.y, point.x, point.y)))
        for alpha in indices:
            if shapes[alpha].contains(point):
                array[
                    max(0, rank - 1) : min(rank + 2, height),
                    max(0, file - 1) : min(file + 2, length),
                ] = 100
                break
    rospy.logdebug(f"[Occupancy Grid] Generating occupancy grid in {time.time() - start}.")
    return array.flatten()


def generate_occlusion_polygons(
        coordinates: NDArray,
        alpha: float = 1,
        wall_buffer: float = 4.0, # Should correspond to distance from camera to SPOT behind
        buffer_side_count: int = 6
    ) -> None:

    def include_edge(vertex1: Sequence[int], vertex2: Sequence[int]) -> None:
        edge: Sequence[int] = (vertex1, vertex2)
        if edge in edges or reversed(edge) in edges:
            return
        edges.add(edge)
        points.append(feature_coordinates[[vertex1, vertex2]])

    rospy.logdebug(f"[Occupancy Grid] Generating points.")
    feature_points = [shapely.Point(rank, file) for rank, file in coordinates]
    if len(feature_points) <= 3:
        rospy.logdebug(f"[Occupancy Grid] Returning convex hull.")
        return shapely.MultiPoint(feature_points).convex_hull
    rospy.logdebug(f"[Occupancy Grid] Generating alpha shapes.")
    feature_coordinates = np.array([point.coords[0] for point in feature_points])
    triangles = Delaunay(feature_coordinates)
    edges, points = set(), list()
    for vertex1_index, vertex2_index, vertex3_index in triangles.vertices:
        coordinate1 = feature_coordinates[vertex1_index]
        coordinate2 = feature_coordinates[vertex2_index]
        coordinate3 = feature_coordinates[vertex3_index]
        if filter_triangle(coordinate1, coordinate2, coordinate3, alpha):
            include_edge(vertex1_index, vertex2_index)
            include_edge(vertex2_index, vertex3_index)
            include_edge(vertex3_index, vertex1_index)
    rospy.logdebug(f"[Occupancy Grid] Polygonizing alpha shapes.")
    polygons = shapely.ops.polygonize(shapely.MultiLineString(points))
    rospy.logdebug(f"[Occupancy Grid] Taking polygonal union.")
    return shapely.unary_union(polygons).buffer(wall_buffer, quad_segs=buffer_side_count)


def filter_triangle(
    vertex1: Sequence[int],
    vertex2: Sequence[int],
    vertex3: Sequence[int],
    alpha: float,
) -> bool:

    def calculate_distance(coordinate1, coordinate2) -> float:
        return math.sqrt(
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
