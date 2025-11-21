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
        self.sub1 = self.create_subscription(Path,'/local_plan_map',self.local_plan_map_cb,10)
        self.sub2 = self.create_subscription(Path,'/plan',self.plan_cb,10)
        self.turn_signal_pub = self.create_publisher(Int8,'/turn_signal',10)
        #self.eval_timer = self.create_timer(1, self.determine_direction)
        self.angular_threshold = 20 #degの閾値
    
    def angle_diff(self,a, b):  #2つの角度a,b[deg]の差を[-180,180]の範囲で返す
        d = (a - b + 180) % 360 - 180
        return d

    def deg(self,pose_stamp):
        w = pose_stamp.pose.orientation.w
        z = pose_stamp.pose.orientation.z
        yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
        deg = math.degrees(yaw)
        return deg


    def pub(self,char):
        msg = Int8()
        if char == 'left':
            msg.data = 1
        elif char == 'right':
            msg.data = 2
        elif char == 'straight':
            msg.data = 0
        else:
            return
        self.get_logger().info(f'方向シグナルを送信: {char}')
        self.turn_signal_pub.publish(msg)


    def plan_cb(self,msg: Path):
        num_points = len(msg.poses)
        self.get_logger().info(f'{num_points}')
        if num_points < 5:
            return

        for i in range(num_points - 1):
            deg = self.deg(msg.poses[i])
    
    def local_plan_map_cb(self,msg:Path):
        num_points = len(msg.poses)
        self.get_logger().info(f'受信したPathの点数: {num_points}')

        if num_points < 2: #点が少ないときは無視
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

         



def main(args=None):
    rclpy.init(args=args)
    node = PathListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()