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
        sub_optimal_trajectory = self.create_subscription(Path, '/optimal_trajectory', self.optimal_trajectory_cb, 10)
        sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        pub_blinker_led_command = self.create_publisher(String, '/blinker_led_command', 10)

        # curvature 
        self.curvature_ofset = 15 # 曲率計算用のオフセット点数

        self.state = None
        self.area = None
        self.optimal_flag = False

        # majority 
        self.count_ofset = 10 # 判定用のオフセット

        self.count = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transform = None

        # position
        self.target = 29 # 目標点のインデックス
        self.position_limit_angle = 90.0 # 位置角度の制限値(degree)


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
        self.count = 0
        point_stamped = PointStamped()
        point_stamped.header = msg.header

        limit = min(30, len(msg.poses))
        for i in range(limit):
            point_stamped.point = msg.poses[i].pose.position
            transformed = do_transform_point(point_stamped, self.transform)
            dy = transformed.point.y
            if dy < 0:
                self.count += 1
            elif dy > 0:
                self.count -= 1

        return self.count / limit

    
    def position(self, msg):
        first_pt = PointStamped()
        first_pt.header = msg.header
        first_pt.point = msg.poses[0].pose.position

        target_pt = PointStamped()
        target_pt.header = msg.header
        target_pt.point = msg.poses[self.target].pose.position

        fpt = do_transform_point(first_pt, self.transform).point
        tpt = do_transform_point(target_pt, self.transform).point

        dx = tpt.x - fpt.x
        dy = tpt.y - fpt.y
        angle = math.degrees(math.atan2(dy, dx))

        if abs(angle) > self.position_limit_angle:
            angle = math.copysign(self.position_limit_angle, angle)
        return  angle / 90


    def optimal_trajectory_cb(self, msg: Path):
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

        score = 0.3 * majority + 0.7 * angle
        self.get_logger().info(f'score: {score:.2f}, majority: {majority:.2f}, position: {angle:.2f}')

        

    def cmd_vel_cb(self, msg: Twist):
        #cmd_velを監視して、旋回中はturningにする
        angular_z = msg.angular.z
        linear_x = msg.linear.x

        if abs(angular_z) > 0.1 and abs(linear_x) > 0.1:
            turning = True
        else:
            turning = False




def main():
    rclpy.init()
    node = PerfectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
