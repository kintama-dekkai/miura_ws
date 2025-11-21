import rclpy
import math
import statistics
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

class PathListener(Node):
    def __init__(self):
        super().__init__('path_listener')
        self.sub1 = self.create_subscription(Path,'/local_plan',self.plan_cb,10)
        self.sub2 = self.create_subscription(Odometry,'/odom',self.odom_cb,10)
        self.sub3 = self.create_subscription(Twist,'/cmd_vel', self.cmd_cb, 10)

        # 0 = straight , 1 = left , 2 = right
        self.turn_signal_pub = self.create_publisher(Int8,'/turn_signal',10)

        self.eval_timer = self.create_timer(1, self.determine_direction)
        
        self.current_deg = 0
        self.angular_threshold = 20 #degの閾値
        self.deg = 0 #0000000

    def cmd_cb(self,msg:Twist):
        self.lin_x = msg.linear.x
        self.ang_z = msg.angular.z

    def odom_cb(self ,msg:Odometry):#現在のロボットの向き
        q = msg.pose.pose.orientation

        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        deg = math.degrees(yaw)
        self.current_deg = deg
    
    def angle_diff(self,a, b):  #2つの角度a,b[deg]の差を[-180,180]の範囲で返す
        d = (a - b + 180) % 360 - 180
        return d


    def plan_cb(self, msg: Path):
        num_points = len(msg.poses)
        #self.get_logger().info(f'受信したPathの点数: {num_points}')

        y_ls = []

        for i in range(0,num_points):
            pose_stamp = msg.poses[i]

            w = pose_stamp.pose.orientation.w
            z = pose_stamp.pose.orientation.z
            yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
            y_ls.append(yaw)
        
        if not y_ls:
            return

        mid_yaw = statistics.median(y_ls) #yawをリストに入れて中央値を求めてる
        mid_deg = math.degrees(mid_yaw) #degにする

        comparsion_deg = self.angle_diff(math.degrees(y_ls[0]),mid_deg)

        if comparsion_deg > self.angular_threshold:
            msg = Int8()
            msg.data = 1  # left
            self.turn_signal_pub.publish(msg)
            self.get_logger().info('left')

        elif comparsion_deg < -self.angular_threshold:
            msg = Int8()
            msg.data = 2  # right
            self.turn_signal_pub.publish(msg)
            self.get_logger().info('right')

        else:
            msg = Int8()
            msg.data = 0  # straight
            self.turn_signal_pub.publish(msg)
            self.get_logger().info('straight')
            
        self.get_logger().info(f'{comparsion_deg}')
        y_ls.clear()


    def determine_direction(self):
        self.get_logger().info(f'currentdeg = {self.current_deg} ')
        self.get_logger().info(f'mid_deg = {self.deg}')




def main(args=None):
    rclpy.init(args=args)
    node = PathListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()