import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuSubscriber(Node):
    def __init__(self):
        super().__init__("imu_subscriber")
        self.subscription = self.create_subscription(
            Imu,
            "camera/imu",
            self.imu_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.subscription
        self.publisher_ = self.create_publisher(
            Imu, "imu/data_raw", qos_profile=qos.qos_profile_system_default
        )

    def imu_callback(self, msg):
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init()

    sub = ImuSubscriber()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
