import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Pub(Node):
    def __init__(self):
        super().__init__('pub')
        self.sub = self.create_subscription(String ,'/blinker_led_command',self.led_cb,  10)
        #self.timer = self.create_timer(0.5, self.timer_cb)
        self.command = None

    def led_cb(self, msg:String):
        if msg.data == 'left':
            self.get_logger().info('<<<<<|     ')
        elif msg.data == 'right':
            self.get_logger().info('     |>>>>>')
        elif msg.data == 'straight':
            self.get_logger().info('     |     ')
        else:
            self.get_logger().info(f'{msg.data}')
    
    #def timer_cb(self):
        


def main(args=None):
   rclpy.init(args=args)
   node = Pub()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()

