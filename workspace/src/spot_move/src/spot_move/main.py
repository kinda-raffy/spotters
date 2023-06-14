import rospy
import actionlib
import math

from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Duration
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import (
    PoseStamped, Pose
)
from spot_msgs.msg import (
    TrajectoryAction,
    TrajectoryGoal,
)
from tf.transformations import quaternion_from_euler
from typing import (
    NoReturn,
    Callable,
    List
)


def main() -> NoReturn:
    rospy.init_node('SpotMove', log_level=rospy.DEBUG)
    # TODO - Find optimal value.
    SpotMovement(path_resolution=5)
    rospy.spin()


class SpotMovement:

    def __init__(
        self,
        path_resolution: int = 10,
        timeout: int = 5,
    ) -> None:
        rospy.Subscriber("/spotters/navigator/path", Path, self.path_callback)
        rospy.logdebug("Listening to path messages.")

        assert timeout > 0, "For safety, the timeout must exceed 0."
        self.default_timeout: Duration = Duration()
        self.default_timeout.data = rospy.Duration(timeout)

        self.path_resolution = path_resolution
        self.path: List[PoseStamped] = list()
        self.last_sent_pose: PoseStamped = None
        self.handler = SpotTrajectoryHandler(self.goal_reached_strategy)
        rospy.logdebug("Movement initialized.")

    def path_callback(self, path: Path) -> NoReturn:
        rospy.logdebug("Executing a new path.")
        self.path = path.poses
        if self.handler.get_state() in [
            GoalStatus.PENDING,
            GoalStatus.ACTIVE
        ]:
            self.handler.cancel_goal()
        self.path = self.down_sample_path(self.path)
        rospy.logdebug(f"Downsampled to {len(self.path)}")
        self.send_next_trajectory()

    def down_sample_path(self, path: List[PoseStamped]) -> List[PoseStamped]:
        rospy.logdebug(f"Downsampling {len(path)=} poses.")
        path_is_too_small: bool = len(path) < self.path_resolution
        if path_is_too_small:
            rospy.logerr(f"Ignoring {len(path)} when {self.path_resolution}")
        return path[::self.path_resolution]

    def send_next_trajectory(self):
        # NOTE - Trajectories are evaluated by the goal_reached_strategy callback.
        if not self._empty_path():
            no_yaw_pose = self._pop_next_pose()
            quaternion = quaternion_from_euler(0,0,0)
            if self.last_sent_pose:
                yaw = self.extrapolate_turning_angle(self.last_sent_pose.pose, no_yaw_pose.pose)
                quaternion = quaternion_from_euler(0, 0, yaw)
            next_pose = PoseStamped(
                no_yaw_pose.header,
                Pose(
                    no_yaw_pose.pose.point,
                    quaternion
                )
            )
            self.last_sent_pose = next_pose
            rospy.logdebug("Sending next trajectory.")
            self.handler.send_trajectory_command(next_pose, self.default_timeout)

    def extrapolate_turning_angle(one: PoseStamped, two: PoseStamped) -> float:
        return math.atan2(two.x - one.x, two.y - one.y)

    def send_last_sent_pose(self) -> NoReturn:
        assert self.last_sent_pose is not None, \
            "Client claims a pose has been sent but no history of that pose exists."
        rospy.logdebug("Sending last trajectory.")
        self.handler.send_trajectory_command(self.last_sent_pose, self.default_timeout)

    def goal_reached_strategy(self, status, result) -> NoReturn:
        if status == GoalStatus.SUCCEEDED:
            rospy.logdebug("Pose reached successfully.")
            self.send_next_trajectory()
        else:
            rospy.logwarn(
                f"Pose traversal unsuccessful due to "
                + f"[ {result.message} ], sending the pose again."
            )
            # FIXME - Should this only be ran once?
            self.send_last_sent_pose()

    def _pop_next_pose(self) -> PoseStamped:
        rospy.logdebug("Getting next trajectory.")
        return self.path.pop(0)

    def _empty_path(self) -> bool:
        return self.path == list()


class SpotTrajectoryHandler:
    def __init__(
        self,
        arrival_callback: Callable [
            [], NoReturn
        ]
    ) -> None:
        self.goal_cb = arrival_callback
        self.client = actionlib.SimpleActionClient(
            '/spot/trajectory',
            TrajectoryAction
        )
        self.client.wait_for_server(timeout=rospy.Duration(60))
        rospy.logdebug("Connection established.")


    def send_trajectory_command(
        self,
        pose: PoseStamped,
        duration: Duration,
        precise_positioning: bool = True
    ) -> bool:
        goal = TrajectoryGoal()
        goal.target_pose = pose
        goal.duration = duration
        goal.precise_positioning = precise_positioning

        rospy.logdebug("Sending goal.")
        self.client.send_goal(goal, self.goal_cb)

    def get_result(self) -> None:
        rospy.logdebug("Getting goal result.")
        return self.client.get_result()

    def cancel_goal(self) -> None:
        rospy.logdebug("Cancelling goal.")
        # TODO: Check if the robot stops moving.
        self.client.cancel_goal()

    def get_state(self) -> GoalStatus:
        rospy.logdebug("Getting goal state.")
        return self.client.get_state()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
