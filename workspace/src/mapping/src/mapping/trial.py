import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d


class PointCloudToMesh:
    def __init__(self):
        rospy.init_node('point_cloud_to_mesh', anonymous=True)
        rospy.Subscriber("/orb_slam3/all_points", PointCloud2, self.callback)
        self.voxel_size = 0.1
        self.max_std_dev = 2.0

        try:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
        except Exception as e:
            print("Error initializing the Open3D Visualizer: ", e)

        rospy.spin()

    def callback(self, data):
        pc = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)

        # This part errors out weirdly. Something to do with np or Vector3dVector().
        try:
            pc_np = np.asarray(list(pc))
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pc_np)
        except Exception as e:
            print("EXCEPTION", e)

        # Voxel downsampling.
        down_pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        # Outlier removal.
        cl, _ = down_pcd.remove_statistical_outlier(
            nb_neighbors=10,
            std_ratio=self.max_std_dev
        )

        # Estimate normals. Figure out why this is required.
        cl.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30
            )
        )

        # Triangle mesh using Ball-Pivoting algorithm.
        # radii = [0.005, 0.01, 0.02, 0.04]
        # rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        #        cl, o3d.utility.DoubleVector(radii))

        # Mesh using Poisson.
        rec_mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(cl, depth=5)

        # Mesh simplification. Figure out why this is required.
        simplified_mesh = rec_mesh.simplify_quadric_decimation(100000)

        try:
            # Update visualisation.
            self.vis.add_geometry(simplified_mesh)
            self.vis.update_geometry(simplified_mesh)
            self.vis.poll_events()
            self.vis.update_renderer()
        except Exception as e:
            print("Error updating the Open3D Visualizer: ", e)


def main():
    PointCloudToMesh()


if __name__ == '__main__':
    try:
        PointCloudToMesh()
    except rospy.ROSInterruptException as e:
        print("Trial experienced", e)
