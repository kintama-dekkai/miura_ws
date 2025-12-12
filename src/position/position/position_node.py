import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_py as tf2
from tf2_geometry_msgs import do_transform_pose
from nav_msgs.msg import Path
from std_msgs.msg import String
import math


class Position(Node):
    def __init__(self):
        super().__init__('position')
        self.sub1 = self.create_subscription(Path,'/optimal_trajectory',self.optimal_trajectory_cb,10)
        self.turn_signal_pub = self.create_publisher(String,'/blinker_led_command',10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("distance, distance_x, distance_y, angle")

        self.last_command = None
        self.angular_threshold = 20
        self.distance_threshold = 0.5

        self.target = 29

        

    def optimal_trajectory_cb(self,msg:Path):
        if len(msg.poses) < 2: #点が少ないときは無視
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

        first_point = msg.poses[0].pose                     
        target_point = msg.poses[self.target].pose     

        first_point_transformed = do_transform_pose(first_point, transform)
        target_point_transformed = do_transform_pose(target_point, transform)

        fptp = first_point_transformed.position
        tptp = target_point_transformed.position
        
        distance_x = tptp.x - fptp.x
        distance_y = tptp.y - fptp.y
        distance = math.hypot(distance_x, distance_y)
        angle = math.degrees(math.atan2(distance_y, distance_x))


        if -self.angular_threshold <= angle <= self.angular_threshold or distance < self.distance_threshold:
            self.command = 'straight' 
        elif angle > self.angular_threshold:
            self.command = 'left'
        else:
            self.command = 'right'

        if self.command != self.last_command:
            led_msg = String()
            led_msg.data = self.command
            self.turn_signal_pub.publish(led_msg)
            self.last_command = self.command
        
        self.get_logger().info(f'{self.command}, {distance:.2f}, {distance_x:.2f}, {distance_y:.2f}, {angle:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = Position()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

