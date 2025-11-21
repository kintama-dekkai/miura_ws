import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String


class PathListener(Node):
    def __init__(self):
        super().__init__('path_listener')
        self.sub1 = self.create_subscription(Path,'/local_plan',self.local_plan_cb,10)
        self.turn_signal_pub = self.create_publisher(String,'/led_command',10)
 
        self.angular_threshold = 20 #degの閾値
        self.last_pub = None
        self.get_logger().info('num_points:compersion_deg:string')
        self.get_logger().info('trajectoriesBA-JON')

    
    def angle_diff(self,a, b):  #2つの角度a,b[deg]の差を[-180,180]の範囲で返す
        d = (a - b + 180) % 360 - 180
        return d

    def deg(self,pose_stamp):
        w = pose_stamp.pose.orientation.w
        z = pose_stamp.pose.orientation.z
        yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
        deg = math.degrees(yaw)
        return deg


    def pub(self,string):
        if self.last_pub == string:
            return
        else:
            self.last_pub = string

        msg = String()
        msg.data = string
        self.turn_signal_pub.publish(msg)
        self.get_logger().info('送信')

    
    def local_plan_cb(self,msg:Path):
        num_points = len(msg.poses)
        if num_points < 2: #点が少ないときは無視
            return

        start_deg = self.deg(msg.poses[0])
        end_deg = self.deg(msg.poses[-1])

        comparsion_deg = self.angle_diff(end_deg,start_deg)

        if abs(comparsion_deg) < 5:
            string = 'C:0,0,255'
            deg = '0~5'
        elif abs(comparsion_deg) < 10:
            string = 'C:0,255,255'
            deg = '5~10'
        elif abs(comparsion_deg) < 15:
            string = 'C:0,255,0'
            deg = '10~15'
        elif abs(comparsion_deg) < 20:
            string = 'C:255,255,0'
            deg = '15~20'
        else:
            string = 'C:255,0,0'
            deg = '20~'


            
        self.pub(string)
        self.get_logger().info(f'{num_points} : {comparsion_deg:.1f} : {deg}')

         



def main(args=None):
    rclpy.init(args=args)
    node = PathListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
