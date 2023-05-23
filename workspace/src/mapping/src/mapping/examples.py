import rospy
import open3d as o3d


def init_point_cloud():
    pass


def

def point_cloud_to_mesh(cloud: ROS_POINT_CLOUD, gpu: bool = False, sample_size: float = 0.05):
    # set GPU or CPU
    o3d_core = o3d.core()

    cloud = init_point_cloud(cloud)

    boxy_cloud = cloud.voxel_down_sample(voxel_size=0.05)
    # o3d.visualization.draw([downpcd])

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(boxy_cloud, depth=9)


if __name__ == "__main__":



    print('Running Poisson surface reconstruction ...')

    print('Displaying reconstructed mesh ...')
    o3d.visualization.draw([mesh])