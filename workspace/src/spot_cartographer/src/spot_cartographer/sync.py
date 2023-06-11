import rospy
from sensor_msgs.msg import PointCloud2, Empty


class PointCloudChangeTracker:
    def __init__(self) -> None:
        self.cloud_listener = rospy.Subscriber(
            "orb_slam3/all_points",
            PointCloud2,
            self.detect_significant_changes,
        )
        self.squealer = rospy.Publisher(
            "spotters/cartographer/merge",
            Empty,
            queue_size=1,
        )
        self.shrinker = rospy.Publisher(
            "spotters/cartographer/birth",
            Empty,
            queue_size=1,
        )
        self.based_loss_threshold = 100
        self.last_point_count = 0
        self.growth_aggregate = 0
        self.update_count = 0
        self.avg_point_count_increase = 0
        self.change_factor = 1.5

    def detect_significant_changes(self, cloud: PointCloud2) -> None:
        cloud_point_count = cloud.row_step * cloud.height
        change_in_points = cloud_point_count - self.last_point_count
        change_threshold = round(self.avg_point_count_increase * self.change_factor)
        dark_backward_and_abysm_of_time = Empty()
        if change_in_points > change_threshold:
            self.squealer.publish(dark_backward_and_abysm_of_time)
        elif change_in_points < self.based_loss_threshold:
            self.shrinker.publish(dark_backward_and_abysm_of_time)
        else:
            self.growth_aggregate += change_in_points
            self.update_count += 1
            self.avg_point_count_increase = round(self.growth_aggregate / self.update_count)
