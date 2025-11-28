import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
from bleak import BleakClient
import asyncio
import threading

LEFT= "7C:2C:67:64:BD:3A"  # ← これがXIAO-ESP32-C6のMAC
RIGHT = "7C:2C:67:64:A6:46"
CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"  # ESP32側のUUID

class BLEControlNode(Node):
    def __init__(self):
        super().__init__('ble_control_node')
        self.blinker = self.create_subscription(String,'/blinker_led_command',self.blinker_cb,10)
        self.stop_sub = self.create_subscription(Bool,'/stop',self.stop_cb,1)

        self.left_command = None
        self.right_command = None
        self.blinking_mode = False
        self.left_connect = False
        self.right_connect = False
        self.stop = False

            # BLE接続処理を非同期で起動
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()
        self.left_client = BleakClient(LEFT)
        self.right_client = BleakClient(RIGHT)
        asyncio.run_coroutine_threadsafe(self.connect_both(), self.loop)


    def blinker_cb(self,msg:String):
        self.get_logger().info(f'{msg.data}')
        self.last = True 
        if msg.data is None:
            return
        if msg.data == 'left':
            self.left_command = "C:255,255,0"
            self.right_command = "C:0,0,0"
            self.blinking_mode = True
        elif msg.data == 'right':
            self.left_command = "C:0,0,0"
            self.right_command = "C:255,255,0"
            self.blinking_mode = True
        elif msg.data == 'turning':
            self.left_command = "C:255,255,0"
            self.right_command = "C:255,255,0"
            self.blinking_mode = True
        else:
            self.left_command ="M:1"
            self.right_command = "M:1"
            self.blinking_mode = False


    def stop_cb(self,msg:Bool):
            self.stop = msg.data
            if self.stop:
                self.get_logger().info("emergency!  emergency!")
            else:
                self.get_logger().info("safety")

    
    def timer_callback(self):
        #どっちかの接続が切れてるとき
        if not self.left_client.is_connected:
            self.get_logger().warn("BLE_left切断検知、再接続します...")
            if hasattr(self, "timer") and self.timer is not None:
                #タイマー破壊
                self.destroy_timer(self.timer)
                self.timer = None
            asyncio.run_coroutine_threadsafe(self.connect_ble(LEFT,"left"), self.loop)
            return
        elif not self.right_client.is_connected:
            self.get_logger().warn("BLE_right切断検知、再接続します...")
            if hasattr(self, "timer") and self.timer is not None:
                #タイマー破壊
                self.destroy_timer(self.timer)
                self.timer = None
            asyncio.run_coroutine_threadsafe(self.connect_ble(RIGHT,"right"), self.loop)
            return
        else:
            if self.stop:
                asyncio.run_coroutine_threadsafe(self.left_client.write_gatt_char(CHAR_UUID, 'C:255,0,0'.encode('utf-8')),self.loop)
                asyncio.run_coroutine_threadsafe(self.right_client.write_gatt_char(CHAR_UUID, 'C:255,0,0'.encode('utf-8')),self.loop)
                return
            
            if self.left_command is None or self.right_command is None:
                self.get_logger().info("No signal")
                return
            
            if not self.blinking_mode or self.last:
                asyncio.run_coroutine_threadsafe(self.left_client.write_gatt_char(CHAR_UUID, self.left_command.encode('utf-8')),self.loop)
                asyncio.run_coroutine_threadsafe(self.right_client.write_gatt_char(CHAR_UUID, self.right_command.encode('utf-8')),self.loop)
                self.last = False
            else:
                asyncio.run_coroutine_threadsafe(self.left_client.write_gatt_char(CHAR_UUID, 'C:0,0,0'.encode('utf-8')),self.loop)
                asyncio.run_coroutine_threadsafe(self.right_client.write_gatt_char(CHAR_UUID, 'C:0,0,0'.encode('utf-8')),self.loop)
                self.last = True
                     

    async def connect_both(self):
        await self.connect_ble(LEFT,"left")
        await asyncio.sleep(3)
        await self.connect_ble(RIGHT,"right")


    async def connect_ble(self,ADDRESS,side):
        if side == "left":
            self.left_connect = False
        else:
            self.right_connect = False
        while True:
            try:
                print(f"{side}:preparation")
                client = BleakClient(ADDRESS)

                await client.connect()

                if side == "left":
                    self.left_client = client
                    self.left_connect = True
                else:
                    self.right_client = client
                    self.right_connect = True
                await asyncio.sleep(3)

                if client.is_connected:
                    self.get_logger().info(f"{side}:BLE接続成功")
                    if self.left_connect and self.right_connect:
                        if not hasattr(self,"timer") or self.timer is None:
                            self.timer = self.create_timer(0.5,self.timer_callback)
                    break
                else:
                    self.get_logger().error("BLE接続失敗(is_connected = False)")
            except Exception as e:
                self.get_logger().error(f"接続エラー: {e}")

            self.get_logger().info("3秒後に再接続")
            await asyncio.sleep(3)  # 3秒待って再試行


def main(args=None):
    rclpy.init(args=args)
    node = BLEControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        futures = []
        if node.left_client:
            futures.append(asyncio.run_coroutine_threadsafe(node.left_client.disconnect(), node.loop))
        if node.right_client:
            futures.append(asyncio.run_coroutine_threadsafe(node.right_client.disconnect(), node.loop))
        for f in futures:
            f.result()
        rclpy.shutdown()

if __name__ == '__main__':
    main()