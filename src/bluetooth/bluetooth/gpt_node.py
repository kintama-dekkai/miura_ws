import rclpy 
from rclpy.node import Node
from nav_msgs.msg import Path

class Bumper(Node):
    def __init__(self):
        super().__init__('bumper_node')
        self.stop_sub = self.create_subscription(Path,'/plan',self.stop_callback(),10)

    def stop_callback(self,msg:Path):
        self.i = 0
        self.i += 1
        self.get_logger().info(f'stop signal received {self.i} times')

def main(args=None):
   rclpy.init(args=args)
   node = Bumper()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()
