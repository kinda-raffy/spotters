import rospy
import std_msgs.msg
import std_srvs.srv
from spot_cartographer.point_cloud_tracker import PointCloudChangeTracker
from spot_cartographer.sensor_fusion import CrucibleForSensorFusion
from typing import (
    Optional,
    NoReturn,
)

class Cartographer:
    def __init__(self) -> None:
        self.service: str = "/octomap_server/reset"
        self.octree_connection: Optional[rospy.ServiceProxy] = None
        self.connect_octree_server()
        self.prefix: str = "spotters/cartographer"
        rospy.Subscriber(
            f"{self.prefix}/birth",
            std_msgs.msg.Empty,
            self.destroy_previous_octree,
        )
        rospy.Subscriber(
            f"{self.prefix}/merge",
            std_msgs.msg.Empty,
            self.destroy_previous_octree,
        )
        rospy.Subscriber(
            f"{self.prefix}/tracking_state",
            std_msgs.msg.Int8,
            self.echo_localisation_status,
        )
        self.lost_publisher = rospy.Publisher(
            f"{self.prefix}/lost",
            std_msgs.msg.Bool,
            queue_size=10,  # FIXME: Does this need a queue?
        )
        self.good_states = {2}
        self.lost_states = {3,4}
        self.previous_state = None

    def connect_octree_server(self) -> None:
        rospy.logdebug("[Cartographer] Waiting for octree connection.")
        rospy.wait_for_service(self.service)
        self.octree_connection = \
            rospy.ServiceProxy(self.service, std_srvs.srv.Empty, persistent=True)
        rospy.logdebug("[Cartographer] Octree connection established.")

    def destroy_previous_octree(self, _) -> None:
        try:
            if self.octree_connection is None:
                raise rospy.ServiceException("Octree connection not established.")
            self.octree_connection()

        except rospy.ServiceException as error:
            rospy.logerror(f"Failed to reset octree: {error}")
        except rospy.ROSException:
            self.connect_octree_server()  # Reconnect and try again.
            self.destroy_previous_octree()

    def echo_localisation_status(self, status: std_msgs.msg.Int8) -> None:
        state: int = status.data
        if state == self.previous_state:
            return
        if (value := state in self.good_states and self.previous_state in self.lost_states) or\
            state in self.lost_states and self.previous_state not in self.lost_states:
            echo = std_msgs.msg.Bool()
            echo.data = value
            self.lost_publisher.publish(echo)
        self.previous_state = state

    def manage(self) -> NoReturn:
        rospy.logdebug("[Cartographer] Creating point cloud tracker and crucible.")
        _ = PointCloudChangeTracker()
        _ = CrucibleForSensorFusion().initiate_fusion()


def main() -> NoReturn:
    rospy.init_node("spot_cartographer", log_level=rospy.DEBUG)
    cartographer = Cartographer()
    cartographer.manage()


if __name__ == "__main__":
    main()
