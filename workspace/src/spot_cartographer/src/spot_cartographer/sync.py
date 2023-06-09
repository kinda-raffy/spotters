import rospy
import time
from std_msgs.msg import Empty
from sensor_msgs.msg import PointCloud2


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

        self.time_interval_sec = 2
        self.last_reset_time = time.time()
        self.based_loss_threshold = 1000
        self.last_point_count = 0
        self.growth_aggregate = 0
        self.update_count = 0
        self.avg_point_count_increase = 0
        self.change_factor = 1.5

    def detect_significant_changes(self, cloud: PointCloud2) -> None:
        now = time.time()
        if now < self.last_reset_time + self.time_interval_sec:
            return
        cloud_point_count = cloud.row_step * cloud.height
        change_in_points = cloud_point_count - self.last_point_count
        change_threshold = round(self.avg_point_count_increase * self.change_factor)
        void = Empty()
        if change_in_points > change_threshold:
            rospy.logdebug(f"detected map merge with change of size: {change_in_points}")
            self.squealer.publish(void)
            self.update_count = 0
            self.growth_aggregate = 0
            self.last_reset_time = now
        elif change_in_points < -self.based_loss_threshold:
            rospy.logdebug(f"detected new_map with loss of {change_in_points}")
            self.shrinker.publish(void)
            self.last_reset_time = now
        self.growth_aggregate += change_in_points
        self.update_count += 1
        self.avg_point_count_increase = round(self.growth_aggregate / self.update_count)
        self.last_point_count = cloud_point_count
