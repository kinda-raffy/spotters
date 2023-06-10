import math
import shapely
import shapely.ops
import numpy as np
import scipy.spatial as spatial
from typing import Sequence
from numpy.typing import NDArray
from nav_msgs.msg import OccupancyGrid
import time


def generate_occupancy_grid(grid: OccupancyGrid) -> NDArray:
    values = np.array(grid.data).reshape((grid.info.height, grid.info.width))
    coordinates = np.column_stack(np.where(values == 100))
    points = [shapely.Point(rank, file) for rank, file in coordinates]
    alphas = generate_occlusion_polygons(points)
    array = np.zeros((grid.info.height, grid.info.width), np.int8)
    start=time.time()
    for rank in np.arange(0, grid.info.height - 2, 3):
        for file in np.arange(0, grid.info.width - 2, 3):
            point = shapely.Point(rank, file)
            if shapely.within(point, alphas):
                array[rank][file] = 100
                array[rank + 1][file] = 100
                array[rank + 2][file] = 100
                array[rank][file + 1] = 100
                array[rank][file + 2] = 100
                array[rank + 1][file + 1] = 100
                array[rank + 2][file + 2] = 100
    print(f"Created occupancy grid in start {time.time() - start:.2f}.")
    return array.flatten()


def generate_occlusion_polygons(
        feature_points: Sequence[shapely.Point], alpha: float = 0.5) -> None:

    def include_edge(vertex1: Sequence[int], vertex2: Sequence[int]) -> None:
        edge: Sequence[int] = (vertex1, vertex2)
        if edge in edges or reversed(edge) in edges:
            return
        edges.add(edge)
        points.append(feature_coordinates[[vertex1, vertex2]])

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


def derive_cartesian_coordinates(coordinates: NDArray) -> None:
    array_indices = np.stack(np.indices(coordinates.shape), axis=2)
    cartesian_coordinates = np.rot90(array_indices, k=1)
    print(cartesian_coordinates)
