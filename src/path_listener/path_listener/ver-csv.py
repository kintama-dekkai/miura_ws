import rclpy
import math
import statistics
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist


import csv
import os
import time
from geometry_msgs.msg import PoseStamped

class PathListener(Node):
    def __init__(self):
        super().__init__('path_listener')
        self.sub1 = self.create_subscription(Path,'/local_plan',self.plan_cb,10)
        self.sub2 = self.create_subscription(Twist,'/cmd_vel',self.cmd_cb,10)
        self.turn_signal_pub = self.create_publisher(Int8,'/turn_signal',10)
        #self.eval_timer = self.create_timer(1, self.determine_direction)
        self.angular_threshold = 10 #degの閾値



        #------------------------------------------------------------------------------
        log_dir = os.path.expanduser('~/path_logs')
        os.makedirs(log_dir, exist_ok=True)
        fname = os.path.join(log_dir, f'angle_log_{int(time.time())}.csv')
        self._log_file = open(fname, 'w', newline='')
        self._csv_writer = csv.writer(self._log_file)
        # ヘッダ
        self._csv_writer.writerow(['t','num_points','path_length_m','start_deg','end_deg','diff_deg','median_deg',
                                'lin_vel','ang_vel','turn_signal','gt_label'])
        # 保持用（odom/cmd_velから読む場合に使う）
        self.latest_lin = 0.0
        self.latest_ang = 0.0
        self.latest_gt = -1  # ground truth（/gt_label を購読して更新するならそれを利用）

    def path_length(self, poses):
        L = 0.0
        for i in range(len(poses)-1):
            x1 = poses[i].pose.position.x
            y1 = poses[i].pose.position.y
            x2 = poses[i+1].pose.position.x
            y2 = poses[i+1].pose.position.y
            L += math.hypot(x2-x1, y2-y1)
        return L
        #------------------------------------------------------------------------

    def cmd_cb(self, msg: Twist):
        self.latest_lin = msg.linear.x
        self.latest_ang = msg.angular.z

    
    def angle_diff(self,a, b):  #2つの角度a,b[deg]の差を[-180,180]の範囲で返す
        d = (a - b + 180) % 360 - 180
        return d

    def deg(self,pose_stamp):
        w = pose_stamp.pose.orientation.w
        z = pose_stamp.pose.orientation.z
        yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
        deg = math.degrees(yaw)
        return deg


    def pub(self, char):
        msg = Int8()
        if char == 'left':
            msg.data = 1
        elif char == 'right':
            msg.data = 2
        elif char == 'straight':
            msg.data = 0
        else:
            return
        self.last_turn_signal = msg.data  # ← ログ用に保存
        self.get_logger().info(f'方向シグナルを送信: {char}')
        self.turn_signal_pub.publish(msg)


    def plan_cb(self, msg: Path):
        num_points = len(msg.poses)
        self.get_logger().info(f'受信したPathの点数: {num_points}')

        if num_points == 0:
            self.get_logger().warn('Pathが空です。スキップします。')
            return  

        if num_points < 3: #点が少ないときは無視
            return

        start_deg = self.deg(msg.poses[0])
        end_deg = self.deg(msg.poses[-1])

        comparsion_deg = self.angle_diff(end_deg,start_deg)

        if comparsion_deg > self.angular_threshold:
            self.pub('left')

        elif comparsion_deg < -self.angular_threshold:
            self.pub('right')

        else:
            self.pub('straight')


        #------------------------------------------------------------------------
        t = time.time()
        num_points = len(msg.poses)
        plength = self.path_length(msg.poses) if num_points>1 else 0.0
        start_deg = self.deg(msg.poses[0]) if num_points>0 else 0.0
        end_deg = self.deg(msg.poses[-1]) if num_points>0 else 0.0
        yaws = [self.deg(p) for p in msg.poses] if num_points>0 else []
        median_deg = statistics.median(yaws) if yaws else 0.0
        diff = self.angle_diff(end_deg, start_deg)

        # turn_signal 現状の出力（もし publish 直前の変数があるなら）
        turn_signal_val = self.last_turn_signal if hasattr(self,'last_turn_signal') else -1

        # 書き込む
        self._csv_writer.writerow([t, num_points, plength, start_deg, end_deg, diff, median_deg,
                                self.latest_lin, self.latest_ang, turn_signal_val, self.latest_gt])
        self._log_file.flush()

    def destroy_node(self):
        self._log_file.close()
        super().destroy_node()
        #-------------------------------------------------------------------------------



def main(args=None):
    rclpy.init(args=args)
    node = PathListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()