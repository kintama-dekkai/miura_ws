import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Curvature(Node):
    def __init__(self):
        super().__init__('Curvature_local')
        self.sub1 = self.create_subscription(Path ,'/optimal_trajectory',self.optimal_trajectory_cb, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.led_signal_pub = self.create_publisher(String,'/blinker_led_command',10)

        self.k_on = 0.15 #Siriusは0.1くらいがいいかな？予想
        self.k_off = 0.07
        self.a = 1.0

        self.k_smooth = 0.0
        self.led_signal = String()
        self.led_signal.data = 'straight'
        self.last_led_signal = None

        self.get_logger().info('turnsignal, curvature, len')


    def curvature_from_three_points(self,pose_stamped):
        k = 0
        cross = 0

        (x1,y1) = pose_stamped[0].pose.position.x, pose_stamped[0].pose.position.y
        (x2,y2) = pose_stamped[len(pose_stamped)//2].pose.position.x, pose_stamped[len(pose_stamped)//2].pose.position.y
        (x3,y3) = pose_stamped[-1].pose.position.x, pose_stamped[-1].pose.position.y
            
        area = 0.5 * abs(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))

        eps = 1e-6
        if area < eps:
            return 0.0

        a = math.hypot(x2-x1, y2-y1)
        b = math.hypot(x3-x2, y3-y2)
        c = math.hypot(x3-x1, y3-y1)

        if a < eps or b < eps or c < eps:
            return 0.0

        R = (a*b*c) / (4.0 * area)
        k = 1.0 / R
        # sign from cross product of vectors (p2-p1) x (p3-p2)
        v1x, v1y = x2 - x1, y2 - y1
        v2x, v2y = x3 - x2, y3 - y2
        cross = v1x * v2y - v1y * v2x
        if k == 0:
            return 0
        return math.copysign(k, cross)


    def optimal_trajectory_cb(self,msg:Path):
        if len(msg.poses) < 3:
            return
            
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


        start = msg.poses[0].pose.position
        end = msg.poses[-1].pose.position

        unko = math.hypot(start.x - end.x, start.y - end.y)

        self.get_logger().info(f'{self.led_signal.data}, {self.k_smooth}, {len(msg.poses)}, {unko}')

        if self.led_signal.data != self.last_led_signal:
            self.led_signal_pub.publish(self.led_signal)
            self.last_led_signal = self.led_signal.data

    
    def cmd_vel_cb(self, msg:Twist):
        self.lin_x = msg.linear.x
        self.ang_z = msg.angular.z
        if self.lin_x == 0 and self.ang_z != 0:
            self.led_signal.data = 'turning'
            
        


def main(args=None):
   rclpy.init(args=args)
   node = Curvature()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()

