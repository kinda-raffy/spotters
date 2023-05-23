import rospy
from sensor_msgs.msg import (
    PointCloud2
)
import open3d as o3d
import numpy as np


def init_point_cloud(cloud: PointCloud2, device: o3d.core.Device):
    pcd = o3d.geometry.PointCloud()

    for point in cloud.data:
        pcd.extend(toPoint(point.x, point.y, point.z))

def toPoint(x: float, y: float, z: float):
    return [x, y, z]

def point_cloud_to_mesh(
        cloud: PointCloud2,
        gpu: bool = False,
        voxel_size: float = 0.05,
        neighbor_filter: int = 20,
        std_ratio: float = 2.0,
        depth: int = 5
    ):

    # set GPU or CPU
    device = o3d.core.Device("GPU:0") if gpu else o3d.core.Device("CPU:0")

    # init point cloud
    cloud = init_point_cloud(cloud, device)


    # remove outliers
    voxel_down_pcd = cloud.voxel_down_sample(voxel_size=voxel_size)
    no_outliers, _ = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=neighbor_filter, std_ratio=std_ratio)

    # create mesh
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(no_outliers, depth=depth)

    return mesh


def updated_point_cloud_callback(cloud: PointCloud2):
    point_cloud_to_mesh(cloud)


def main():
    rospy.init_node('publisher_node', anonymous=True)
    pub = rospy.Subscriber(
        "/orb_slam3/all_points", PointCloud2, updated_point_cloud_callback
    )

def debug():
    sample_ply_data = o3d.data.PLYPointCloud()
    pcd = o3d.io.read_point_cloud(sample_ply_data.path)
    # Flip it, otherwise the pointcloud will be upside down.
    pcd.transform(
        [
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 1]
        ]
    )
    print(pcd)
    print("point cloud")
    # o3d.visualization.draw([pcd])
    # remove outliers
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.1)
    no_outliers, _ = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=2.0)

    # create mesh
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(no_outliers, depth=5)
    o3d.visualization.draw([mesh])


if __name__ == "__main__":
    debug()