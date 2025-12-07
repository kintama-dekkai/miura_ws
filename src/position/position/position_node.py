import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_py as tf2
from tf2_geometry_msgs import do_transform_pose
from nav_msgs.msg import Path
from std_msgs.msg import String


class Position(Node):
    def __init__(self):
        super().__init__('position')
        self.sub1 = self.create_subscription(Path,'/optimal_trajectory',self.optimal_trajectory_cb,10)
        self.turn_signal_pub = self.create_publisher(String,'/blinker_led_command',10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    
    def optimal_trajectory_cb(self,msg:Path):
        if len(msg.poses) < 2: #点が少ないときは無視
            return

        self.get_logger().info(f'Path frame: {msg.header.frame_id}')
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='sirius3/base_footprint',
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time(),
            )
        except (tf2.LookupException, tf2.ExtrapolationException, tf2.ConnectivityException) as e:
            self.get_logger().warning(f'TF error: {e}')
            return

        first_point = msg.poses[0].pose                     # PoseStamped
        target_point = msg.poses[len(msg.poses)//2].pose     # PoseStamped

        first_point_transformed = do_transform_pose(first_point, transform)
        target_point_transformed = do_transform_pose(target_point, transform)

        fptp = first_point_transformed.position
        tptp = target_point_transformed.position

        self.get_logger().info(
            f'First point in base:  x={fptp.x:.2f}, y={fptp.y:.2f}'
        )
        self.get_logger().info(
            f'Target point in base: x={tptp.x:.2f}, y={tptp.y:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Position()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

