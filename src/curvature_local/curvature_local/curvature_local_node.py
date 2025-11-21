import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Curvature_local(Node):
   def __init__(self):
      super().__init__('Curvature_local')
      self.sub1 = self.create_subscription(Path ,'/local_plan',self.local_plan_cb,10)
      self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
      self.led_signal_pub = self.create_publisher(String,'/blinker_led_command',10)

      self.k_on = 1.0 #Siriusは0.1くらいがいいかな？予想
      self.k_off = 0.5
      self.k_smooth = 0.0
      self.led_signal = String()
      self.led_signal.data = 'straight'
      self.a = 0.7
      self.last_led_signal = None

      self.get_logger().info('turnsignal, curvature, len')


   def curvature_from_three_points(self,pose_stamped):
      max_area = 0.0
      k = 0
      cross = 0

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
      return math.copysign(k, cross)


   def local_plan_cb(self,msg:Path):
      if len(msg.poses) < 3:
         if self.lin_x == 0 and self.ang_z != 0:
            self.led_signal.data = 'turninng'
         else:
            self.led_signal.data = 'straight'
      else:
         pose_stamped = msg.poses
         self.k_new = self.curvature_from_three_points(pose_stamped)
         self.k_smooth = self.a * self.k_new + (1 - self.a) * self.k_smooth

         if self.k_smooth > self.k_on:
               self.led_signal.data = 'left'

         elif self.k_smooth < -self.k_on:
               self.led_signal.data = 'right'

         #しきい値うろちょろ問題解決用
         elif self.led_signal.data == 'left' and self.k_smooth < self.k_off:
               self.led_signal.data = 'straight'

         elif self.led_signal.data == 'right' and self.k_smooth > -self.k_off:
               self.led_signal.data = 'straight'

         #self.get_logger().info(f'{self.led_signal.data}, {self.k_smooth}, {len(msg.poses)}')

         if self.led_signal.data != self.last_led_signal:
            self.led_signal_pub.publish(self.led_signal)
            self.last_led_signal = self.led_signal.data
      

   def cmd_vel_cb(self, msg:Twist):
      self.lin_x = msg.linear.x
      self.ang_z = msg.angular.z
      self.get_logger().info(f'lin_x: {self.lin_x}, ang_z: {self.ang_z}')


def main(args=None):
   rclpy.init(args=args)
   node = Curvature_local()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()

