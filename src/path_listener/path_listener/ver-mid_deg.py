import rclpy
import math
import statistics
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class PathListener(Node):
    def __init__(self):
        super().__init__('path_listener')
        self.subscription = self.create_subscription(Path,'/local_plan',self.plan_cb,10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_cb,10)
        self.left_pub = self.create_publisher(Bool, '/turn_signal/left', 10)   
        self.right_pub = self.create_publisher(Bool, '/turn_signal/right', 10)
        
        self.current_deg = 0
        self.angular_threshold = 20 #degの閾値

    def odom_cb(self ,msg:Odometry):
        q = msg.pose.pose.orientation

        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        deg = math.degrees(yaw)
        self.current_deg = deg


    def plan_cb(self, msg: Path):
        num_points = len(msg.poses)
        self.get_logger().info(f'受信したPathの点数: {num_points}')

        y_ls = []

        for i in range(0,num_points):
            pose_stamp = msg.poses[i]

            w = pose_stamp.pose.orientation.w
            z = pose_stamp.pose.orientation.z
            yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
            y_ls.append(yaw)
        
        if not y_ls:
            return

        mid_yaw = statistics.median(y_ls)
        mid_deg = math.degrees(mid_yaw)

        if mid_deg > self.current_deg + self.angular_threshold:
            self.left_pub.publish(Bool(data=True))
            self.right_pub.publish(Bool(data=False))
            self.get_logger().info(f'left')

        elif mid_deg < self.current_deg - self.angular_threshold:
            self.left_pub.publish(Bool(data=False))
            self.right_pub.publish(Bool(data=True))
            self.get_logger().info(f'right')

        else:
            self.left_pub.publish(Bool(data=False))
            self.right_pub.publish(Bool(data=False))
            self.get_logger().info(f'straight')
        
        y_ls.clear()


    def determine_direction(self,deg):
        a=0



def main(args=None):
    rclpy.init(args=args)
    node = PathListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()