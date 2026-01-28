import rclpy
import math
import tf2_ros
import tf2_py as tf2
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Position(Node):
    def __init__(self):
        super().__init__('position')
        self.sub1 = self.create_subscription(Path,'/optimal_trajectory',self.optimal_trajectory_cb,10)
        self.sub2 = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_cb,10)
        self.blinker_pub = self.create_publisher(String,'/blinker_led_command',10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info('Position node started.')

        # position
        self.on_threshold = 20
        self.off_threshold = 10
        self.distance_threshold = 0.3
        self.target = 29

        self.path_command = 'straight'

        #cmd_vel
        self.velocity_on_threshold = 0.1
        self.velocity_off_threshold = 0.05
        self.cmd = 'off'

        self.last_command = None

        
    def optimal_trajectory_cb(self,msg:Path):
        if len(msg.poses) < self.target: #点が少ないときは無視
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='sirius3/base_footprint',
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time(),
            )
        except (tf2.LookupException, tf2.ExtrapolationException, tf2.ConnectivityException) as e:
            self.get_logger().warning(f'TF error: {e}')
            return
                 
        target_pt = PointStamped()
        target_pt.header = msg.header
        target_pt.point = msg.poses[self.target].pose.position

        tpt = do_transform_point(target_pt, transform).point

        angle = math.degrees(math.atan2(tpt.y, tpt.x))
        distance = math.sqrt(tpt.x**2 + tpt.y**2)
        #self.get_logger().info(f'distance={distance:.2f}, angle={angle:.2f}')

        if distance < self.distance_threshold:
            self.path_command = 'hazard'
            return

        if self.path_command == 'straight':
            if angle > self.on_threshold:
                self.path_command = 'left'
            elif angle < -self.on_threshold:
                self.path_command = 'right'
        elif self.path_command == 'left':
            if angle < self.off_threshold:
                self.path_command = 'straight'
        elif self.path_command == 'right':
            if angle > -self.off_threshold:
                self.path_command = 'straight'
        
        
        
    
    def cmd_vel_cb(self, msg: Twist):
        #cmd_velを監視して、旋回中はturningにする
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        #旋回判定
        if angular_z == 0 and linear_x == 0:
            self.cmd = 'off'
        elif linear_x < self.velocity_off_threshold:
            self.cmd = 'hazard'
        elif linear_x > self.velocity_on_threshold:
            self.cmd = self.path_command

        if self.cmd == self.last_command:
            return

        msg = String()
        msg.data = self.cmd
        self.blinker_pub.publish(msg)
        self.last_command = self.cmd

        self.get_logger().info(f'cmd={self.cmd},path_command={self.path_command}')

def main(args=None):
    rclpy.init(args=args)
    node = Position()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

