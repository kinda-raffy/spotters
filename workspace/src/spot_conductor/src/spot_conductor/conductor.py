import rospy
import functools
from typing import Callable, Mapping
from random import randint, randrange
from math import radians
from enum import Enum
from std_msgs.msg import Header
from geometry_msgs.msg import (
    PoseStamped, Pose, Point, Quaternion
)
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler as from_euler


RECEIVE_GRID = "spotters/geography/map"
RECEIVE_POSE = "spotters/geography/pos"
RECEIVE_GOAL = "/move_base_simple/goal"
SEND_GOAL = "spotters/conductor/goal"
SEND_POSE = "/spotters/navigator/pose"
CANCEL_GOAL = "blah"  # TODO: Figure out what this is.


class Conductor:

    class SpotState(Enum):
        START, STUCK, IDLE, GOAL, RECOVERY = range(5)

    def __init__(self):

        def update_state(field: str, value) -> None:
            setattr(self, field, value)

        self.behaviours: Mapping[self.SpotState, Callable[[], None]] = {
            self.SpotState.START: self.initialise,
            self.SpotState.IDLE: self.idle,
            self.SpotState.GOAL: self.seek,
            self.SpotState.STUCK: self.unstick,
            self.SpotState.RECOVERY: self.recover,
        }
        self.rate: float = 1.0
        self.state = self.SpotState.START
        self.latest_pose = None
        self.latest_grid = None
        self.active_goal = None
        self.goal_failed = False
        rospy.Subscriber(
            RECEIVE_GRID,
            OccupancyGrid,
            functools.partial(update_state, "latest_grid"),
        )
        rospy.Subscriber(
            RECEIVE_POSE,
            PoseStamped,
            functools.partial(update_state, "latest_pose"),
        )
        rospy.Subscriber(
            RECEIVE_GOAL,
            PoseStamped,
            functools.partial(update_state, "active_goal"),
        )
        rospy.Subscriber(
            CANCEL_GOAL,
            Bool,  # TODO: Not necessarily final type.
            functools.partial(update_state, "goal_failed"),
        )
        self.goal_channel = rospy.Publisher(
            SEND_GOAL,
            PoseStamped,
            queue_size=1,
        )
        self.pose_channel = rospy.Publisher(
            SEND_POSE,
            PoseStamped,
            queue_size=1,
        )

    def loop(self):
        # These behaviors should be blocking
        self.conduct(self.SpotState.START)
        if not rospy.is_shutdown():
            rospy.sleep(1)
            self.determine_state()
            self.conduct(self.state)

    def conduct(self, state: SpotState):
        self.behaviours[state]()

    def determine_state(self):
        last_state = self.state
        if None in [self.latest_grid, self.latest_pose]:
            self.state = self.SpotState.START
        elif self.is_stuck():
            self.state = self.SpotState.STUCK
        elif self.active_goal is None:
            self.state = self.SpotState.IDLE
        elif self.active_goal_failed:
            self.state = self.SpotState.RECOVERY
        else:
            self.state = self.SpotState.GOAL
        if self.state != last_state:
            rospy.loginfo("[Conductor] changing state to {self.state}")


    # State Behaviours

    def initialise(self) -> None:
        # turn 360
        rospy.logdebug("[Conductor] Starting init stage")
        rospy.sleep(10)
        for _ in range(round(360 / 45)):
            self.turn_body(45)
            rospy.sleep(5)

        self.translate_body(0,0.5)
        rospy.sleep(2)
        self.turn_body(-15)
        rospy.sleep(2)
        self.turn_body(15)
        rospy.sleep(2)
        self.translate_body(0,-0.5)
        rospy.sleep(2)
        self.turn_body(-15)
        rospy.sleep(2)
        self.turn_body(15)

        self.loop()

    def idle(self) -> None:
        import time
        time.sleep(1)
        if randint(0, round(30 / self.rate)) == 0:
            angle = randrange(-45, 45, 15)
            self.turn(angle)
        self.loop()

    def unstick(self) -> None:
        # TODO: Define a recovery procedure. Needs further subscribers.
        rospy.Rate(1).sleep()
        self.loop()

    def seek(self) -> None:
        rate = rospy.Rate(2)
        self.goal_channel.publish(self.active_goal)
        while self.active_goal is not None and not self.goal_failed:
            rate.sleep()
        self.state = self.SpotState.RECOVERY \
            if self.goal_failed else self.SpotState.IDLE
        self.loop()

    def recover(self) -> None:
        # TODO: Requires navigation to publish failures.
        rospy.Rate(1).sleep()
        self.loop()

    # Movement Functions

    def turn_body(self, degrees_right: float):
        rotation = Quaternion(*from_euler(0, 0, radians(degrees_right)))
        self.post_body_pose(rotation=rotation)

    def translate_body(self, x: float, y: float):
        point = Point(x,y,0)
        self.post_body_pose(point=point)

    def post_body_pose(
            self,
            point: Point = Point(0,0,0),
            rotation: Quaternion = Quaternion(*from_euler(0, 0, 0))
        ) -> Header:
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "body"
        pose = Pose(point, rotation)
        poseStamped = PoseStamped(header, pose)
        self.pose_channel.publish(poseStamped)

def main():
    rospy.init_node("Conductor", log_level=rospy.DEBUG)
    conductor = Conductor()
    conductor.loop()
    rospy.spin()


if __name__ == "__main__":
    main()