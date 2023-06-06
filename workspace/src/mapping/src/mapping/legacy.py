import cv2
import rospy
import numpy as np
import open3d as o3d
import pprint
from cv_bridge import CvBridge
from scipy.spatial import ConvexHull
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt


from shapely.geometry import MultiPoint

import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import (
    PointCloud2,
    Image
)


class PointCloudFilter:
    def __init__(self):
        rospy.init_node('point_cloud_filter', anonymous=True)
        # Initialize point clouds.
        self.all_orb_points = o3d.geometry.PointCloud()
        # self.depth_points = o3d.geometry.PointCloud()

        self.pub = rospy.Publisher('/filtered_points', PointCloud2, queue_size=10)
        rospy.Subscriber('/orb_slam3/all_points', PointCloud2, self.callback_all_points)
        # TODO ~ Change to actual depth topic.
        # rospy.Subscriber('/spot/all_depth_points', PointCloud2, self.callback_depth_points)
        self.map_pub = rospy.Publisher('/spotters/convex_hull', OccupancyGrid, queue_size=1)
        rospy.Subscriber('/projected_map', OccupancyGrid, self.callback_projected_map)

        # Counter for skipping callbacks.
        self.counter = 0

        self.enable_global_registration = False
        self.enable_icp = False
        self.enable_bilateral_smoothing = False

    # --- Callbacks.
    def callback_all_points(self, data):
        if self.counter % 10 == 0:
            self.all_orb_points = self.convert_ros_to_o3d(data)
            self.process_point_clouds()
        self.counter += 1

    def callback_depth_points(self, data):
        if self.counter % 10 == 0:
            self.depth_points = self.convert_ros_to_o3d(data)
            # self.process_point_clouds()
        self.counter += 1

    def callback_projected_map(self, data):
        self.compute_map(data)

    # --- Pipelines.
    def compute_map(self, data):
        # Convert the occupancy grid data to a numpy array
        grid = np.array(data.data)

        # Reshape the grid to 2D
        grid = np.reshape(grid, (data.info.height, data.info.width))

        # Convert to binary (0 for free space, 1 for occupied)
        binary_grid = np.where(grid == 100, 1, 0)

        # Get the coordinates of the cells with value 1
        occupied_cells = np.argwhere(binary_grid == 1)

        # Create a MultiPoint object with these coordinates
        multipoint = MultiPoint(occupied_cells.tolist())

        # Display the shapely geometry
        rospy.loginfo(multipoint)

        # Update the scatter plot data
        x, y = multipoint.xy
        self.scat.set_offsets(np.c_[x, y])

        # # Binary conversion.
        # grid = np.array(msg.data, dtype=np.int8)
        # grid[grid > 0] = 1

        # # Convert to 2D array.
        # grid = grid.reshape(msg.info.height, msg.info.width)

        # # Get all non-zero coordinates.
        # points = np.transpose(np.nonzero(grid))
        # print("Number of points", points.size)

        # # Calculate the hull.
        # # hull = ConvexHull(points)

        # # Draw a line across the grid.
        # new_grid = np.zeros_like(grid, dtype=np.uint8)
        # for simplex in hull.simplices:
        #     cv2.line(new_grid, tuple(points[simplex[0]]), tuple(points[simplex[1]]), 1, thickness=1)

        # if not self.show_image:
        #     print("Showing image.")
        #     cv2.imshow("Map", new_grid)
        #     self.show_image = True

        # new_msg = OccupancyGrid()
        # new_msg.header = msg.header
        # new_msg.info = msg.info
        # new_msg.data = new_grid.flatten().tolist()
        # self.map_pub.publish(new_msg)

    def process_point_clouds(self):
        if self.enable_global_registration:
            print("[0.0] Performing Global Registration.")
            voxel_size = 0.05
            (source_down,
             target_down,
             source_fpfh,
             target_fpfh) = self.prepare_dataset(voxel_size)

            registration_results = self.execute_global_registration(
                source_down, target_down, source_fpfh, target_fpfh, voxel_size
            )
            # Refine alignment with ICP.
            if self.enable_icp:
                print("[0.1] Performing Global Registration.")
                radius_normal = voxel_size * 2
                registration_results = o3d.pipelines.registration.registration_icp(
                    source_down, target_down, radius_normal,
                    registration_results.transformation,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane()
                )
            # Apply transformation to depth points.
            self.depth_points.transform(registration_results.transformation)
            print("[0.2] Merging point clouds.")
            self.all_orb_points += self.depth_points

        if self.enable_bilateral_smoothing:
            print("[1.0] Removing outliers.")
            _, ind = self.all_orb_points.remove_radius_outlier(nb_points=16, radius=0.2)
            self.all_orb_points = self.all_orb_points.select_by_index(ind)

            print("[2.0] Smoothening points via bilateral filtering.")
            np_points = np.asarray(self.all_orb_points.points)
            filtered_points = self.bilateral_filter(np_points, sigma_spatial=0.02, sigma_value=0.1)
            self.all_orb_points.points = o3d.utility.Vector3dVector(filtered_points)

        print("[3.0] Removing secondary outliers.")
        _, ind = self.all_orb_points.remove_radius_outlier(nb_points=8, radius=0.07)
        self.all_orb_points = self.all_orb_points.select_by_index(ind)

        ros_cloud = self.convert_o3d_to_ros(self.all_orb_points)
        print("[-.-] Pipeline complete.\n")
        self.pub.publish(ros_cloud)

    # --- Processing.
    def prepare_dataset(self, voxel_size):
        source_down = self.depth_points.voxel_down_sample(voxel_size)
        target_down = self.all_orb_points.voxel_down_sample(voxel_size)

        radius_normal = voxel_size * 2
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            source_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            target_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

        return source_down, target_down, source_fpfh, target_fpfh

    def execute_global_registration(self, source_down, target_down, source_fpfh, target_fpfh, voxel_size):
        distance_threshold = voxel_size * 1.5
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3,  # RANSAC iteration
            [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
        )
        return result

    def bilateral_filter(self, points, sigma_spatial, sigma_value):
        # FIXME ~ This runs in polynomial time.
        filtered_points = np.zeros_like(points)
        for i in range(points.shape[0]):
            # Compute weights for neighboring points.
            diffs = points - points[i]
            spatial_dists = np.linalg.norm(diffs, axis=1)
            value_dists = np.abs(diffs[:, 2])  # assuming z is the "value" dimension.
            weights = np.exp(-spatial_dists**2 / (2 * sigma_spatial**2)) * np.exp(-value_dists**2 / (2 * sigma_value**2))

            # Compute weighted average.
            filtered_points[i] = np.sum(weights[:, None] * points, axis=0) / np.sum(weights)
        return filtered_points

    # --- Utilities.
    def convert_ros_to_o3d(self, data):
        pc = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(list(pc))
        return pcd

    def convert_o3d_to_ros(self, pcd):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'origin'
        points = [(x[0], x[1], x[2]) for x in np.asarray(pcd.points)]
        ros_cloud = pc2.create_cloud_xyz32(header, points)
        return ros_cloud


def main():
    filter = PointCloudFilter()
    rospy.spin()
