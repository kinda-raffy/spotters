import rospy
from typing import Callable
from random import randint
from enum import Enum
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import (
    PoseStamped
)
from nav_msgs.msg import (
    OccupancyGrid,
)

MAP_TOPIC = "spotters/geography/map"
POS_TOPIC = "spotters/geography/pos"
RECEIVE_GOAL_TOPIC = "/move_base_simple/goal"
SEND_GOAL_TOPIC = "spotters/conductor/goal"

def main():
    rospy.init_node('Conductor', log_level=rospy.DEBUG)
    Conductor()
    rospy.spin()


class Conductor:

    class SpotState(Enum):
        INIT, STUCK, IDLE, GOAL, RECOVERY = range(5)

    def __init__(self):
        self.behaviours = {}
        rospy.Subscriber(MAP_TOPIC, OccupancyGrid, self.map_callback)
        rospy.Subscriber(POS_TOPIC, PoseStamped, self.curr_pos_callback)
        rospy.Subscriber(RECEIVE_GOAL_TOPIC, PoseStamped, self.goal_callback)
        self.goal_channel = rospy.Publisher( SEND_GOAL_TOPIC, PoseStamped, queue_size=1)
        self.latest_map = None
        self.latest_position: PoseStamped = None
        self.active_goal = None
        self.active_goal_failed = False
        self.state = self.SpotState.INIT
        self.idle_chance_denominator_seconds = 30
        self.ros_rate = 1.0
        # TODO: Nav failure subscriber

    def conduct(self, state: SpotState):
        action = self.behaviours[state]
        return action()

    def map_callback(self, map_msg):
        self.latest_map = map_msg

    def curr_pos_callback(self, pos_msg):
        self.latest_position = pos_msg

    def goal_callback(self, goal_msg):
        self.active_goal = goal_msg.data

    def loop(self):
        rate = rospy.Rate(self.ros_rate)
        # These behaviors should be blocking
        while not rospy.is_shutdown():
            state = self.determine_state()
            self.conduct(state)
        rate.sleep()


    def determine_state(self):
        last_state = self.state
        if None in [self.latest_map, self.latest_position]:
            self.state = self.SpotState.INIT
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

    def register_behaviour(self, behaviour: SpotState):
        def register(function: Callable[[], None]):
            self.behaviours[behaviour] = function
            return function
        return register

    @register_behaviour(SpotState.INIT)
    def startup():
        pass

    @register_behaviour(SpotState.IDLE)
    def idle(self):
        if randint(0, round(30/self.ros_rate)) == 0:
            # do something cool
            pass

    @register_behaviour(SpotState.STUCK)
    def stuck():
        pass

    @register_behaviour(SpotState.GOAL)
    def goal(self):
        self.goal_channel.publish(self.active_goal)

    @register_behaviour(SpotState.RECOVERY)
    def recovery():
        pass

if __name__ == "__main__":
    main()