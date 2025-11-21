import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String


class Curvature_local(Node):
   def __init__(self):
       super().__init__('Curvature_local')
       self.sub1 = self.create_subscription(Path ,'/local_plan',self.local_plan_cb,10)
       self.turn_signal_pub = self.create_publisher(String,'/blinker_led_command',10)

       self.get_logger().info('turnsignal, curvature, len')
       self.angular_curvature = 1.0


   def curvature_from_three_points(self,p1, p2, p3):
       (x1,y1),(x2,y2),(x3,y3) = p1,p2,p3
       a = math.hypot(x2-x1, y2-y1)
       b = math.hypot(x3-x2, y3-y2)
       c = math.hypot(x3-x1, y3-y1)
       # area by shoelace
       area = 0.5 * abs(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))
       if area == 0 or a*b*c == 0:
           return 0.0  # 直線(または重複点)は曲率0
       R = (a*b*c) / (4.0 * area)
       k = 1.0 / R
       # sign from cross product of vectors (p2-p1) x (p3-p2)
       v1x, v1y = x2 - x1, y2 - y1
       v2x, v2y = x3 - x2, y3 - y2
       cross = v1x * v2y - v1y * v2x
       return math.copysign(k, cross)


   def local_plan_cb(self,msg:Path):
        if len(msg.poses) < 3:
           return
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        k = self.curvature_from_three_points(points[-3], points[-2], points[-1])
        turn_signal = String()
        if k > 0.01:
           turn_signal.data = 'right'
        elif k < -0.01:
           turn_signal.data = 'left'
        else:
           turn_signal.data = 'straight'
        self.turn_signal_pub.publish(turn_signal)
        self.get_logger().info(f'{turn_signal.data}, {k}, {len(msg.poses)}')


def main(args=None):
   rclpy.init(args=args)
   node = Curvature_local()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()

