import rospy
from std_msgs.msg import Empty
from std_srvs.srv import Empty
from spot_cartographer.sync import PointCloudChangeTracker
from typing import (
    Optional,
    NoReturn,
)

class Cartographer:
    def __init__(self) -> None:
        self.service: str = "/octomap_server/reset"
        self.octree_connection: Optional[rospy.ServiceProxy] = None
        self.connect_octo_server()
        rospy.Subscriber(
            "spotters/cartographer/birth",
            Empty,
            self.destroy_previous_octree
        )
        rospy.Subscriber(
            "spotters/cartographer/merge",
            Empty,
            self.destroy_previous_octree
        )

    def connect_octo_server(self) -> None:
        rospy.logdebug("[Cartographer] Waiting for octree connection.")
        rospy.wait_for_service(self.service)
        self.octree_connection = \
            rospy.ServiceProxy(self.service, Empty, persistent=True)
        rospy.logdebug("[Cartographer] Octree connection established.")

    def destroy_previous_octree(self, _) -> None:
        try:
            if self.octree_connection is None:
                raise rospy.ServiceException("Octree connection not established.")
            self.octree_connection()
        except rospy.ServiceException as error:
            rospy.logerror(f"Failed to reset octree: {error}")
        except rospy.ROSException:
            self.connect_octo_server()  # Reconnect and try again.
            self.destroy_previous_octree()

    def manage(self) -> NoReturn:
        rospy.logdebug("[Cartographer] Managing.")
        _ = PointCloudChangeTracker()
        rospy.spin()


def main() -> NoReturn:
    rospy.init_node("spot_cartographer", log_level=rospy.DEBUG)
    cartographer = Cartographer()
    cartographer.manage()


if __name__ == "__main__":
    main()
