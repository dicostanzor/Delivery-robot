import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class VSLAMCovarianceFix(Node):
    def __init__(self):
        super().__init__('vslam_odom_covariance_fix')
        self.pub = self.create_publisher(Odometry, '/visual_slam/tracking/odometry_fixed', 10)
        self.sub = self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry', self.callback, 10)

    def callback(self, msg):
        # Reasonable non-zero covariance values (tune later based on VSLAM performance)
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        pose_diag = [0.01, 0.01, 0.01, 0.02, 0.02, 0.02]
        twist_diag = [0.01, 0.01, 0.01, 0.02, 0.02, 0.02]
        for i in range(6):
            pose_cov[i * 6 + i] = pose_diag[i]
            twist_cov[i * 6 + i] = twist_diag[i]
        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = VSLAMCovarianceFix()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
