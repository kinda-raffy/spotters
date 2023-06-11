import time
import rospy
import numpy as np
import open3d as o3d
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class PointCloudProcessingFacility:
    def __init__(self):
        # TODO: Publish to a new topic.
        self.pub = rospy.Publisher('filtered_points', PointCloud2, queue_size=10)
        # TODO: Merged with point cloud data.
        rospy.Subscriber('/orb_slam3/all_points', PointCloud2, self.process_point_cloud)
        self.last_callback = 0
        self.callback_interval_sec = 1
        self.all_orb_points = o3d.geometry.PointCloud()
        self.enable_bilateral_smoothing = False

    def process_point_cloud(self, data):
        time_threshold = self.last_callback + self.callback_interval_sec
        current_time = time.time()
        evaluation_cycle = current_time > time_threshold
        if evaluation_cycle:
            self.all_orb_points = ros_to_o3d(data)
            self.store_point_cloud_map()
            self.last_callback = current_time

    def store_point_cloud_map(self):
        if self.enable_bilateral_smoothing:
            rospy.logdebug(f"[Point Cloud Facility] Deleting outliers.")
            _, ind = self.all_orb_points.remove_radius_outlier(nb_points=16, radius=0.2)
            self.all_orb_points = self.all_orb_points.select_by_index(ind)
            rospy.logdebug(f"[Point Cloud Facility] Smoothening points via bilateral filtering.")
            np_points = np.asarray(self.all_orb_points.points)
            filtered_points = bilateral_filter(np_points, sigma_spatial=0.02, sigma_value=0.1)
            self.all_orb_points.points = o3d.utility.Vector3dVector(filtered_points)
        rospy.logdebug(f"[Point Cloud Facility] Removing outliers.")
        _, ind = self.all_orb_points.remove_radius_outlier(nb_points=50, radius=0.2)
        self.all_orb_points = self.all_orb_points.select_by_index(ind)
        ros_cloud = o3d_to_ros(self.all_orb_points)
        rospy.logdebug("[Point Cloud Facility] Pipeline complete.\n")
        self.pub.publish(ros_cloud)


def ros_to_o3d(data):
    pc = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(list(pc))
    return pcd


def o3d_to_ros(pcd):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'origin'
    points = [(x[0], x[1], x[2]) for x in np.asarray(pcd.points)]
    ros_cloud = pc2.create_cloud_xyz32(header, points)
    return ros_cloud


def bilateral_filter(points, sigma_spatial, sigma_value):
    filtered_points = np.zeros_like(points)  # FIXME: Polynomial time.
    for index in range(points.shape[0]):
        diffs = points - points[index]  # Compute neighboring point weights.
        spatial_dists = np.linalg.norm(diffs, axis=1)
        value_dists = np.abs(diffs[:, 2])  # The z is the "value" dimension.
        weights = np.exp(-spatial_dists**2 / (2 * sigma_spatial**2)) * np.exp(-value_dists**2 / (2 * sigma_value**2))
        filtered_points[index] = np.sum(weights[:, None] * points, axis=0) / np.sum(weights)
    return filtered_points
