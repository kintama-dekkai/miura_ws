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
        self.sub1 = self.create_subscription(Path,'/local_plan',self.local_plan_cb,10)
        self.sub2 = self.create_subscription(Path,'/plan',self.plan_cb,10)

        # 0 = straight , 1 = left , 2 = right
        self.turn_signal_pub = self.create_publisher(Int8,'/turn_signal',10)

        self.eval_timer = self.create_timer(1, self.determine_direction)
        
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
        num_points = len(msg.pose)
        if num_points < 5:
            return
        
        


    def local_plan_cb(self, msg: Path):
        num_points = len(msg.poses)
        self.get_logger().info(f'受信したPathの点数: {num_points}')

        if num_points < 2: #点が少ないときは無視
            return

        #--------------------経路の長さを求める------------------
        total_length = 0.0  # 経路の長さを初期化

        for i in range(num_points - 1):
            p1 = msg.poses[i].pose.position
            p2 = msg.poses[i+1].pose.position

            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dist = math.sqrt(dx**2 + dy**2)  # 2点間の距離
            total_length += dist              # 合計に加算

        self.get_logger().info(f"Path length: {total_length:.3f} [m]")
        #----------------------------------------------------------------

        start_deg = self.deg(msg.poses[0])
        end_deg = self.deg(msg.poses[-1])

        comparsion_deg = self.angle_diff(end_deg,start_deg)

        if comparsion_deg > self.angular_threshold:
            self.pub('left')

        elif comparsion_deg < -self.angular_threshold:
            self.pub('right')

        else:
            self.pub('straight')
            
        self.get_logger().info(f'{comparsion_deg}')

def main(args=None):
    rclpy.init(args=args)
    node = PathListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()