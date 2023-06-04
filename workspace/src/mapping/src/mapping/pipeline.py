import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import numpy as np


class PointCloudFilter:
    def __init__(self):
        rospy.init_node('point_cloud_filter', anonymous=True)
        self.pub = rospy.Publisher('/filtered_points', PointCloud2, queue_size=10)

        rospy.Subscriber('/orb_slam3/all_points', PointCloud2, self.callback_all_points)
        # TODO ~ Change to actual depth topic.
        # rospy.Subscriber('/spot/all_depth_points', PointCloud2, self.callback_depth_points)

        # Initialize point clouds.
        self.all_orb_points = o3d.geometry.PointCloud()
        # self.depth_points = o3d.geometry.PointCloud()

        # Counter for skipping callbacks.
        self.counter = 0

        self.enable_global_registration = False
        self.enable_icp = False
        self.enable_bilateral_smoothing = False

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

    def convert_ros_to_o3d(self, data):
        pc = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(list(pc))
        return pcd

    def process_point_clouds(self):
        # Fast Global Registration.
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

            # Bilateral Filtering.
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
