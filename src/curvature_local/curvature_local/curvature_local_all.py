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

      self.angular_curvature = 1.0
      self.get_logger().info('turnsignal, curvature, len')


   def curvature_from_three_points(self,pose_stamped):
      max_area = 0.0
      k = 0
      cross = 0
      unko = []

      for i in range(len(pose_stamped) - 2):
         (x1,y1) = pose_stamped[i].pose.position.x, pose_stamped[i].pose.position.y
         (x2,y2) = pose_stamped[i+1].pose.position.x, pose_stamped[i+1].pose.position.y
         (x3,y3) = pose_stamped[i+2].pose.position.x, pose_stamped[i+2].pose.position.y
         
         area = 0.5 * abs(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))
         if area > max_area:
            a = math.hypot(x2-x1, y2-y1)
            b = math.hypot(x3-x2, y3-y2)
            c = math.hypot(x3-x1, y3-y1)
            R = (a*b*c) / (4.0 * area)
            k = 1.0 / R
            # sign from cross product of vectors (p2-p1) x (p3-p2)
            v1x, v1y = x2 - x1, y2 - y1
            v2x, v2y = x3 - x2, y3 - y2
            cross = v1x * v2y - v1y * v2x
            if k == 0:
               continue
            unko.append(math.copysign(int(k * 1000), cross))
      self.get_logger().info(f'{unko}')
      return math.copysign(k, cross)


   def local_plan_cb(self,msg:Path):
        if len(msg.poses) < 3:
           return
        pose_stamped = msg.poses
        k = self.curvature_from_three_points(pose_stamped)
        turn_signal = String()
        if k > self.angular_curvature:
           turn_signal.data = 'left'
        elif k < -self.angular_curvature:
           turn_signal.data = 'right'
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

