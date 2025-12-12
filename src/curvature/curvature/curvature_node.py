import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import tf2_ros
import tf2_py as tf2
import tf_transformations
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped



class Curvature(Node):
    def __init__(self):
        super().__init__('Curvature_local')
        self.sub1 = self.create_subscription(Path ,'/optimal_trajectory',self.optimal_trajectory_cb, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.led_signal_pub = self.create_publisher(String,'/blinker_led_command',10)

        self.k_on = 0.11 #Siriusは0.1くらいがいいかな？予想
        self.k_off = 0.05
        self.a = 0.85
        self.d_ref = 2.4

        self.k_smooth = 0.0
        self.led_signal = String()
        self.led_signal.data = 'straight'
        self.last_led_signal = None
        self.optimal_flarg = False
        #tf
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.right_count = None
        self.left_count = None
        self.count = 0
        self.count_ofset = 10


        self.get_logger().info('turnsignal, curvature, k_raw, distance, state')


    def curvature_from_three_points(self,pose_stamped,):
        eps = 1e-6
        self.state = None
        self.area = None

        #始点と終点の距離が近いときは曲率0とする
        self.distance = math.hypot(pose_stamped[0].pose.position.x - pose_stamped[-1].pose.position.x, pose_stamped[0].pose.position.y - pose_stamped[-1].pose.position.y)
        if self.distance < 0.5:
            self.state = 'distance'
            self.optimal_flarg = True
            return 0.0

        #面積が小さいときは曲率0とする
        (x1,y1) = pose_stamped[15].pose.position.x, pose_stamped[15].pose.position.y
        (x2,y2) = pose_stamped[len(pose_stamped)//2].pose.position.x, pose_stamped[len(pose_stamped)//2].pose.position.y
        (x3,y3) = pose_stamped[45].pose.position.x, pose_stamped[45].pose.position.y
        self.area = 0.5 * abs(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))
        if self.area < eps:
            self. state = 'area'
            return 0.0 

        #辺の長さが短いときは曲率0とする
        a = math.hypot(x2-x1, y2-y1)
        b = math.hypot(x3-x2, y3-y2)
        c = math.hypot(x3-x1, y3-y1)
        if a < eps or b < eps or c < eps:
            self.state = 'a,b,c'
            return 0.0

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


    def majority_side_from_path(self, msg):
        ### TF取得 ###
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='sirius3/base_footprint',
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time(),
            )
        except (tf2.LookupException, tf2.ExtrapolationException, tf2.ConnectivityException) as e:
            self.get_logger().warning(f'TF error: {e}')
            return

        self.count = 0
        i = 0
        point_stamped = PointStamped()
        point_stamped.header = msg.header  # frame_id と stamp が必要

        while i < len(msg.poses):
            point_stamped.point = msg.poses[i].pose.position
            transformed = do_transform_point(point_stamped, transform)
            dy = transformed.point.y
            if dy < 0:
                self.count += 1
            elif dy > 0:
                self.count -= 1
            i+=1

        if self.count > self.count_ofset:
            return 'right'
        elif self.count < -self.count_ofset:
            return 'left'
        else:
            return 'straight'



    def optimal_trajectory_cb(self,msg:Path):
        self.optimal_flarg = False
        if len(msg.poses) < 3:
            return
            
        pose_stamped = msg.poses
        #k_new = self.curvature_from_three_points(pose_stamped)
        #self.k_smooth = self.a * k_new + (1 - self.a) * self.k_smooth

        majority = self.majority_side_from_path(msg)
        self.get_logger().info(f'majority: {majority}, count: {self.count}')

        """
        if self.k_smooth > self.k_on:
            self.led_signal.data = 'left'
        elif self.k_smooth < -self.k_on:
            self.led_signal.data = 'right'
        #しきい値うろちょろ問題解決用
        elif self.led_signal.data == 'left' and self.k_smooth < self.k_off:
            self.led_signal.data = 'straight'
        elif self.led_signal.data == 'right' and self.k_smooth > -self.k_off:
            self.led_signal.data = 'straight'
        """

        #self.get_logger().info(f'{self.led_signal.data}, {self.k_smooth:.3f}, {k_raw:.3f}, {self.destance:.3f}, {self.state}, {self.area}')


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
    node = Curvature()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()