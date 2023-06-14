import rospy
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
RECEIVE_GOAL_TOPIC = '/move_base_simple/goal'
SEND_GOAL_TOPIC = "spotters/conductor/goal"

def main():
    rospy.init_node('Conductor', log_level=rospy.DEBUG)
    Conductor()
    rospy.spin()

class State(Enum):
    INIT = -1
    STUCK = 0
    IDLE = 1
    GOAL = 2

class Conductor:
    def __init__(self):
        rospy.Subscriber(MAP_TOPIC, OccupancyGrid, self.map_callback)
        self.latest_map = None
        rospy.Subscriber(POS_TOPIC, PoseStamped, self.curr_pos_callback)
        self.latest_position: PoseStamped = None
        rospy.Subscriber(RECEIVE_GOAL_TOPIC, PoseStamped, self.goal_callback)
        self.active_goal = None
        # TODO: Nav failure subscriber

    def map_callback(self, map_msg):
        self.latest_map = map_msg

    def curr_pos_callback(self, pos_msg):
        self.latest_position = pos_msg

    def goal_callback(self, goal_msg):
        self.active_goal = goal_msg.data

    def loop(self):
        rate = rospy.Rate(1.0)
        # These behaviors should be blocking until success
        while not rospy.is_shutdown():
            state = self.check_state()
            if state == State.INIT:
                    self.startup_behaviour()
            elif state == State.STUCK:
                    self.stuck_behaviour()
            elif state == State.IDLE:
                    self.idle_behaviour()
            elif state == State.GOAL:
                    self.goal_behaviour()
        rate.sleep()

    def check_state(self):
        if None in [self.latest_map, self.latest_position]:
            self.state = State.INIT
            return
        health = self.map_state()
        if health == State.STUCK:
            self.state = State.STUCK
        if self.active_goal is None:
            self.state = State.IDLE
        self.state = State.GOAL

    def discern_map_health() -> State:
        pass

    def startup_behaviour():
        pass

    def idle_behaviour():
        pass

    def stuck_behaviour():
        pass

    def goal_behaviour():
        pass

if __name__ == "__main__":
    main()