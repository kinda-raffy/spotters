import math
import itertools
import rtree
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
    alphas = generate_occlusion_polygons(coordinates)
    return construct_occupancy_grid(grid, alphas)


def construct_occupancy_grid(
    grid: OccupancyGrid,
    alphas: shapely.GeometryCollection,
) -> NDArray[np.uint8]:
    index = rtree.index.Index()
    shapes = list()
    # Initialise the tree with all shapes.
    for ordinal, alpha in enumerate(alphas.geoms):
        index.insert(ordinal, alpha)
        shapes.append(alpha)
    length, height = grid.info.width, grid.info.height
    # Create blank grid and iterate over subgrids, labelling if occupied.
    array = np.zeros((height, length), np.uint8)
    for rank, file in itertools.product(
        np.arange(0, height, 3),
        np.arange(0, length, 3)
    ):
        point = shapely.Point(rank, file)
        indices = list(index.intersection(point.x, point.y, point.x, point.y))
        for alpha in indices:
            if shapes[alpha].contains(point):
                array[
                    max(0, rank - 1) : min(rank + 2, length),
                    max(0, file - 1) : min(file + 2, height),
                ] = 100
                break
    return array.flatten()


def derive_cartesian_coordinates(coordinates: NDArray) -> None:
    array_indices = np.stack(np.indices(coordinates.shape), axis=2)
    cartesian_coordinates = np.rot90(array_indices, k=1)
    print(cartesian_coordinates)


def generate_occlusion_polygons(coordinates: NDArray, alpha: float = 1) -> None:

    def include_edge(vertex1: Sequence[int], vertex2: Sequence[int]) -> None:
        edge: Sequence[int] = (vertex1, vertex2)
        if edge in edges or reversed(edge) in edges:
            return
        edges.add(edge)
        points.append(feature_coordinates[[vertex1, vertex2]])

    feature_points = [shapely.Point(rank, file) for rank, file in coordinates]
    if feature_points.size <= 3:
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