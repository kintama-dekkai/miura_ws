import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CurvatureNew(Node):
    def __init__(self):
        super().__init__('CurvatureNew_local')
        self.sub1 = self.create_subscription(Path ,'/optimal_trajectory',self.optimal_trajectory_cb, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.led_signal_pub = self.create_publisher(String,'/blinker_led_command',10)

        self.k_on = 0.20
        self.k_off = 0.05
        self.a = 0.85
        self.distance_threshold = 0.5

        self.k_smooth = 0.0
        self.led_signal = String()
        self.led_signal.data = 'straight'
        self.last_led_signal = None
        self.optimal_flag = False
        self.minimum_area = 10

        self.get_logger().info('turnsignal, curvature, distance, state, area')

    def curvature_from_three_points(self,pose_stamped, length):
        eps = 1e-6
        middle = len(pose_stamped) // 2 - 1
        self.state = None
        self.area = None

        if length < 0.5:
            self.state = 'distance'
            self.optimal_flag = True
            return 0.0

        (x2,y2) = pose_stamped[middle].pose.position.x, pose_stamped[middle].pose.position.y
        
        i = middle
        distance = 0.0
        while i > 0:
            (a1, a2) = pose_stamped[i].pose.position.x, pose_stamped[i].pose.position.y
            (b1, b2) = pose_stamped[i-1].pose.position.x, pose_stamped[i-1].pose.position.y
            distance += math.hypot(a1 - b1, a2 - b2)
            if distance > self.distance_threshold:
                break
            i -= 1
        (x1, y1) = b1,b2

        i = middle
        distance = 0.0
        while i < len(pose_stamped) - 1:
            (a1, a2) = pose_stamped[i].pose.position.x, pose_stamped[i].pose.position.y
            (b1, b2) = pose_stamped[i+1].pose.position.x, pose_stamped[i+1].pose.position.y
            distance += math.hypot(b1 - a1, b2 - a2)
            if distance > self.distance_threshold:
                break
            i += 1
        (x3, y3) = b1, b2

        self.area = 0.5 * abs(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))
        #面積が小さいときは曲率0とする
        if self.area < eps:
            self. state = 'area'
            return 0.0 
        a = math.hypot(x2-x1, y2-y1)
        b = math.hypot(x3-x2, y3-y2)
        c = math.hypot(x3-x1, y3-y1)
        R = (a*b*c) / (4.0 * self.area)
        k = 1.0 / R
        # 外積で向きを判定
        v1x, v1y = x2 - x1, y2 - y1
        v2x, v2y = x3 - x2, y3 - y2
        cross = v1x * v2y - v1y * v2x
        if k == 0:
            self.state = 'k = 0'
            return 0.0

        return math.copysign(k, cross)


    def optimal_trajectory_cb(self,msg:Path):
        self.optimal_flarg = False
        if len(msg.poses) < 3:
            return

        start = msg.poses[0].pose.position
        end = msg.poses[-1].pose.position
        destance = math.hypot(start.x - end.x, start.y - end.y)
            
        pose_stamped = msg.poses
        k_new = self.curvature_from_three_points(pose_stamped, destance)

        self.k_smooth = self.a * k_new + (1 - self.a) * self.k_smooth

        if self.k_smooth > self.k_on:
            self.led_signal.data = 'left'
        elif self.k_smooth < -self.k_on:
            self.led_signal.data = 'right'
        #しきい値うろちょろ問題解決用
        elif self.led_signal.data == 'left' and self.k_smooth < self.k_off:
            self.led_signal.data = 'straight'
        elif self.led_signal.data == 'right' and self.k_smooth > -self.k_off:
            self.led_signal.data = 'straight'


        self.get_logger().info(f'{self.led_signal.data}, {self.k_smooth:.3f}, {destance:.3f}, {self.state}, {self.area}')


    #cmd_velを監視して、旋回中はturningにする
    def cmd_vel_cb(self, msg:Twist):
        self.lin_x = msg.linear.x
        self.ang_z = msg.angular.z
        if self.lin_x == self.ang_z == 0.0 and self.optimal_flarg:
            self.led_signal.data = 'finish'
        elif self.lin_x < 0.06 and self.ang_z != 0:
            self.led_signal.data = 'turning'
        elif self.led_signal.data == 'turning':
            self.led_signal.data = 'straight'

        #最後にpublishしたdataと違うときだけpublish
        if self.led_signal.data != self.last_led_signal:
            self.led_signal_pub.publish(self.led_signal)
            self.last_led_signal = self.led_signal.data


def main(args=None):
    rclpy.init(args=args)
    curvature_new_node = CurvatureNew()
    rclpy.spin(curvature_new_node)
    curvature_new_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()