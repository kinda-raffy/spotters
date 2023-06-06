import rospy
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from mapping.hull import Alpha_Shaper
from mapping.hull_plotter import plot_alpha_shape

from cv_bridge import CvBridge
from shapely.geometry import MultiPoint

import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import (
    PointCloud2,
    Image
)


class Pipeline:
    def __init__(self):
        rospy.init_node('point_cloud_filter', anonymous=True)

        self.pub = rospy.Publisher('/filtered_points', PointCloud2, queue_size=10)
        self.map_pub = rospy.Publisher('/spotters/mapping/map', OccupancyGrid, queue_size=1)
        self.hull_pub = rospy.Publisher('/spotters/mapping/hull_image', Image, queue_size=100)
        rospy.Subscriber('/orb_slam3/all_points', PointCloud2, self.store_pipeline)
        rospy.Subscriber('/projected_map', OccupancyGrid, self.compute_pipeline)

        self.callback_counter = 0
        self.all_orb_points = o3d.geometry.PointCloud()
        self.enable_bilateral_smoothing = False

    # --- Callbacks.
    def store_pipeline(self, data):
        evaluation_cycle = self.callback_counter % 10 == 0
        if evaluation_cycle:
            self.all_orb_points = self._ros_to_o3d(data)
            self.store_pcd_map()
        self.callback_counter += 1

    def compute_pipeline(self, data):
        self.compute_pcd_map(data)

    # --- Pipelines.
    def compute_pcd_map(self, original_map):
        np_grid = np.array(original_map.data)
        np_grid = np.reshape(
            np_grid, (original_map.info.height, original_map.info.width)
        )  # Reshape to 2D.
        binary_grid = np.where(np_grid == 100, 1, 0)

        occupied_coordinates = np.argwhere(binary_grid == 1)
        # free_coordinates = np.argwhere(binary_grid == 0)

        pcd_map = np.zeros_like(binary_grid)
        # for coordinate in free_coordinates:
        #     pcd_map[coordinate[0], coordinate[1]] = 100
        
        _alpha = 25.0
        alpha_shaper = Alpha_Shaper(occupied_coordinates)
        alpha_shape = alpha_shaper.get_shape(alpha=_alpha)
        
        _convex = 1.0
        convex_shaper = Alpha_Shaper(occupied_coordinates)
        convex_shape = convex_shaper.get_shape(alpha=_convex)

        # Create the matplotlib visualization
        fig, ax = plt.subplots(1, 1)
        ax.scatter(*zip(*occupied_coordinates), c='black', s=0.5, alpha=0.3)
        plot_alpha_shape(ax, alpha_shape, "red")
        plot_alpha_shape(ax, convex_shape, "grey")  # Plot the convex shape
        ax.set_title(f"$\\alpha={_alpha:.3} convex={_convex:.3}$")
        ax.set_aspect('equal')

        # Create the matplotlib visualization
        # fig, ax = plt.subplots(1, 1)
        # ax.scatter(*zip(*occupied_coordinates), c='black', s=0.5, alpha=0.3)
        # plot_alpha_shape(ax, alpha_shape)
        # ax.set_title(f"$\\alpha={_alpha:.3}$")
        # ax.set_aspect('equal')

        # Convert the matplotlib figure to an image
        bridge = CvBridge()
        fig.canvas.draw()  # Render the matplotlib figure
        image = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        image_msg = bridge.cv2_to_imgmsg(image, encoding="rgb8")
        
        self.hull_pub.publish(image_msg)
        
        navigable_map = OccupancyGrid()
        navigable_map.header.stamp = rospy.Time.now()
        navigable_map.header.frame_id = original_map.header.frame_id
        navigable_map.info.map_load_time = rospy.Time.now()
        navigable_map.info.resolution = original_map.info.resolution
        navigable_map.info.width = original_map.info.width
        navigable_map.info.height = original_map.info.height
        navigable_map.info.origin = original_map.info.origin
        navigable_map.data = np.ravel(pcd_map).tolist()
        
        self.map_pub.publish(navigable_map)


    def store_pcd_map(self):
        pipeline_state: float = 0.0
        if self.enable_bilateral_smoothing:
            print(f"[{pipeline_state + 1.0}] Removing outliers.")
            _, ind = self.all_orb_points.remove_radius_outlier(nb_points=16, radius=0.2)
            self.all_orb_points = self.all_orb_points.select_by_index(ind)

            print(f"[{pipeline_state + 1.0}] Smoothening points via bilateral filtering.")
            np_points = np.asarray(self.all_orb_points.points)
            filtered_points = self.bilateral_filter(np_points, sigma_spatial=0.02, sigma_value=0.1)
            self.all_orb_points.points = o3d.utility.Vector3dVector(filtered_points)

        print(f"[{pipeline_state + 1.0}] Removing outliers.")
        _, ind = self.all_orb_points.remove_radius_outlier(nb_points=8, radius=0.07)
        self.all_orb_points = self.all_orb_points.select_by_index(ind)

        ros_cloud = self._o3d_to_ros(self.all_orb_points)
        print("[-.-] Pipeline complete.\n")
        self.pub.publish(ros_cloud)

    # --- Processing.
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
    def _ros_to_o3d(self, data):
        pc = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(list(pc))
        return pcd

    def _o3d_to_ros(self, pcd):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'origin'
        points = [(x[0], x[1], x[2]) for x in np.asarray(pcd.points)]
        ros_cloud = pc2.create_cloud_xyz32(header, points)
        return ros_cloud


def main():
    filter = Pipeline()
    rospy.spin()
