import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import (
    Twist,
    PoseStamped,
    Pose,
    Vector3,
    Point,
    Quaternion
)

from spot_driver.spot_ros import SpotROS

def publish_message(pub, message, rate):
    while not rospy.is_shutdown():
        pub.publish(message)
        rate.sleep()

def publish_next_target():
    pub = rospy.Publisher('/spot_path', Point, queue_size=10)
    rate = rospy.Rate(10)
    message = Point(
        x=0,
        y=0,
        z=0
    )
    publish_message(pub, message, rate)
    pub.publish(Twist())

def main():
    SR = SpotROS()
    SR.main()

    rospy.init_node('publisher_node', anonymous=True)
    publish_next_target()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
