import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d


class pointCloudToWalkableVoxels:
    def __init__(self):
        rospy.init_node('point_cloud_to_mesh', anonymous=True)
        rospy.Subscriber("/orb_slam3/all_points", PointCloud2, self.cloud_callback)
        # rospy.Subscriber("ODOM", POINT, self.odom_callback)
        self.voxel_size = 0.1
        self.max_std_dev = 2.0
        self.position = []
        self.radius = 5.0
        self.height = 2.0

        try:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
        except Exception as e:
            print("Error initializing the Open3D Visualizer: ", e)

        rospy.spin()

    def odom_callback(self, data):
        self.position = [data.x, data.y, data.z]

    def cloud_callback(self, data):
        pc = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)

        try:
            pc_np = np.asarray(list(pc))
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pc_np)
        except Exception as e:
            print("EXCEPTION", e)

        # crop point cloud to cylinder
        bounds = o3d.TriangleMesh.create_cylinder(radius=self.radius, height=self.height, resolution=20, split=4, create_uv_map=False)
        cropped = pcd.crop_point_cloud(bounds)
        cropped_and_filtered = pcd.remove_radius_outlier(self, 20, 0.2, print_progress=False)


        convex_hull = cropped_and_filtered.compute_convex_hull()

        # Voxel downsampling.
        features_voxel = cropped_and_filtered.voxel_down_sample(voxel_size=self.voxel_size)

        # make dense voxel grid without convex_hull
        walkable = o3d.VoxelGrid.create_from_triangle_mesh(convex_hull, voxel_size=self.voxel_size)

        # TODO get walkable diff features_voxel

        try:
            # Update visualisation.
            self.vis.add_geometry(simplified_mesh)
            self.vis.update_geometry(simplified_mesh)
            self.vis.poll_events()
            self.vis.update_renderer()
        except Exception as e:
            print("Error updating the Open3D Visualizer: ", e)


def main():
    pointCloudToWalkableVoxels()


if __name__ == '__main__':
    try:
        pointCloudToWalkableVoxels()
    except rospy.ROSInterruptException as e:
        print("Trial experienced", e)
