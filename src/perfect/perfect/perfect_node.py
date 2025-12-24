import rclpy
import math
import tf2_ros
import tf2_py as tf2
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class PerfectNode(Node):
    def __init__(self):
        super().__init__('perfect_node')
        # Initialization code here
        self.sub_optimal_trajectory = self.create_subscription(Path, '/optimal_trajectory', self.optimal_trajectory_cb, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.blinker_pub = self.create_publisher(String, '/blinker_led_command', 10)

        # curvature 
        self.curvature_ofset = 15 # 曲率計算用のオフセット点数

        self.state = None
        self.area = None
        self.optimal_flag = False

        # majority 
        self.count_ofset = 10 # 判定用のオフセット
        self.robot_width = 0.67 # ロボットの幅(m)

        self.count = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transform = None

        # position
        self.target = 29 # 目標点のインデックス
        self.position_limit_angle = 90.0 # 位置角度の制限値(degree)

        #optimal_trajectory_cb
        self.turn_signal = None
        self.last_command = None
        self.path_blinker = 'off'
        self.last_path_time = None
        self.path_timeout = 3.0 # パスが更新されないときのタイムアウト(sec)

        #cmd_vel_cb
        self.velocity_threshold = 0.1 # 旋回とみなす閾値(rad/s, m/s)

        self.cmd_vel_flag = False


    def curvature(self, pose_stamped):
        eps = 1e-6
        middle = len(pose_stamped) // 2
        self.state = None
        self.area = None

        (x1,y1) = pose_stamped[middle - self.curvature_ofset].pose.position.x, pose_stamped[middle - self.curvature_ofset].pose.position.y
        (x2,y2) = pose_stamped[middle].pose.position.x, pose_stamped[middle].pose.position.y
        (x3,y3) = pose_stamped[middle + self.curvature_ofset].pose.position.x, pose_stamped[middle + self.curvature_ofset].pose.position.y

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

    
    def majority(self, msg):
        count = 0
        valid = 0
        half_width = self.robot_width / 2.0

        point_stamped = PointStamped()
        point_stamped.header = msg.header

        limit = min(30, len(msg.poses))
        for i in range(limit):
            point_stamped.point = msg.poses[i].pose.position
            transformed = do_transform_point(point_stamped, self.transform)
            dy = transformed.point.y
            if dy > half_width:
                count += 1
                valid += 1
            elif dy < -half_width:
                count -= 1
                valid += 1

        if valid == 0:
            return 0.0 

        return count / valid

    
    def position(self, msg):
        target_pt = PointStamped()
        target_pt.header = msg.header
        target_pt.point = msg.poses[self.target].pose.position

        tpt = do_transform_point(target_pt, self.transform).point

        angle = math.degrees(math.atan2(tpt.y, tpt.x))

        if abs(angle) > self.position_limit_angle:
            angle = math.copysign(self.position_limit_angle, angle)
        return  angle / 90


    def optimal_trajectory_cb(self, msg: Path):
        self.last_path_time = self.get_clock().now()
        if len(msg.poses) < 4: #点が少ないときは無視
            return

        ### TF取得 ###
        try:
            self.transform = self.tf_buffer.lookup_transform(
                target_frame='sirius3/base_footprint',
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time(),
            )
        except (tf2.LookupException, tf2.ExtrapolationException, tf2.ConnectivityException) as e:
            self.get_logger().warning(f'TF error: {e}')
            return

        
        #curvature = self.curvature(msg)
        majority = self.majority(msg)
        angle = self.position(msg)

        #判定
        if majority > 0.1:
            self.turn_signal = 'left'
        elif majority < -0.1:
            self.turn_signal = 'right'
        else:
            self.turn_signal = 'straight'

        self.get_logger().info(f'turn_signal: {self.turn_signal}, majority: {majority:.2f}, position: {angle:.2f}')

        

    def cmd_vel_cb(self, msg: Twist):
        #cmd_velを監視して、旋回中はturningにする
        linear_x = msg.linear.x
        now = self.get_clock().now()

        #パスがしばらく更新されていなければoffにする
        if (
            self.last_path_time is None or
            (now - self.last_path_time).nanoseconds * 1e-9 > self.path_timeout
            ):
            effective_path = 'off'
        else:
            effective_path = self.turn_signal

        #旋回判定
        if linear_x < self.velocity_threshold:
            cmd = 'hazard'
        else:
            cmd = effective_path

        if cmd == self.last_command:
            return

        msg = String()
        msg.data = cmd
        self.blinker_pub.publish(msg)
        self.last_command = cmd
        
        
            




def main():
    rclpy.init()
    node = PerfectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
