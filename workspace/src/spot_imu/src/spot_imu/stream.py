import rospy
import requests
from sensor_msgs.msg import Imu
from typing import NoReturn
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3



class StreamIMU:

    def __init__(self) -> None:
        sensor_relay_ip = rospy.get_param("ip")
        sensor_relay_port = rospy.get_param("port")
        self.url = f"http://{sensor_relay_ip}:{sensor_relay_port}/imu"
        self.imu_pub = rospy.Publisher(
            "/spotters/imu",
            Imu,
            queue_size=10
        )

    def stream(self) -> NoReturn:
        rospy.logdebug(f"Streaming IMU data from {self.url}")
        while not rospy.is_shutdown():
            self.poll_server()

    def poll_server(self) -> NoReturn:
        try:
            self.get_imu_stream()
        except Exception as error:
            rospy.logerr(f"[SpotIMU] server connection failed with {error}")

    def get_imu_stream(self) -> NoReturn:
        response = requests.get(self.url, stream=True)
        if response.status_code == 200:
            self.read_stream(response)
        else:
            rospy.logerr(f"Request failed with status code {response.status_code}")

    def read_stream(self, response: requests.Response) -> NoReturn:
        for line in response.iter_lines():
            if line:
                decoded_line = line.decode('utf-8')
                imu_values = [float(value) for value in decoded_line.split(',')]

                imu_msg = Imu()
                imu_msg.header = Header()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = "map"

                # Orientation
                roll, pitch, yaw = imu_values[:3]
                quaternion = quaternion_from_euler(roll, pitch, yaw)
                imu_msg.orientation.x = quaternion[0]
                imu_msg.orientation.y = quaternion[1]
                imu_msg.orientation.z = quaternion[2]
                imu_msg.orientation.w = quaternion[3]

                # Angular velocity
                imu_msg.angular_velocity = Vector3(*imu_values[3:6])
                imu_msg.angular_velocity_covariance = [0] * 9
                imu_msg.angular_velocity_covariance[0] = -1 if imu_values[3:6] == [0, 0, 0] else 0

                # Linear acceleration
                imu_msg.linear_acceleration = Vector3(*imu_values[6:])
                imu_msg.linear_acceleration_covariance = [0] * 9
                imu_msg.linear_acceleration_covariance[0] = -1 if imu_values[6:] == [0, 0, 0] else 0

                self.imu_pub.publish(imu_msg)


def main() -> None:
    rospy.init_node('SpotIMU', log_level=rospy.DEBUG)
    streamer = StreamIMU()
    streamer.stream()


if __name__ == '__main__':
    main()