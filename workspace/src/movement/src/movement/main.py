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

def test_cmd_vel():
    pub = rospy.Publisher('/spot/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    message = Twist(
        linear=Vector3(0.0, 0.0, 0.0),
        angular=Vector3(0.0, 0.0, 0.3)
    )
    publish_message(pub, message, rate)
    pub.publish(Twist())

def test_pose():
    pub = rospy.Publisher('/spot/go_to_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)
    header = Header(seq=0, stamp=rospy.Time(0), frame_id='body')
    pose = Pose(
        position=Point(1.0, 0.0, 0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
    )
    message = PoseStamped(header=header, pose=pose)
    publish_message(pub, message, rate)
    pub.publish(PoseStamped())

def main():
    SR = SpotROS()
    SR.main()

    rospy.init_node('publisher_node', anonymous=True)
    test_cmd_vel()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
