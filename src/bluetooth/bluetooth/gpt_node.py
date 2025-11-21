import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bleak import BleakClient
import asyncio

ADDRESS = "7C:2C:67:64:BD:3A"  # ← これがXIAO-ESP32-C6のMAC
CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"  # ESP32側のUUID

class BLEControlNode(Node):
    def __init__(self):
        super().__init__('ble_control_node')
        self.sub = self.create_subscription(String, '/led_command', self.command_callback, 10)


        # BLE接続処理を非同期で起動
        self.loop = asyncio.get_event_loop()
        self.client = BleakClient(ADDRESS)
        self.loop.run_until_complete(self.connect_ble())


    def command_callback(self, msg):
        # トピックで受け取ったデータをBLEで送信
        self.loop.create_task(self.send_command(msg.data))

    async def connect_ble(self):
        try:
            await self.client.connect()
            if self.client.is_connected:
                self.get_logger().info("✅ BLE接続成功！")
            else:
                self.get_logger().error("❌ BLE接続失敗")
        except Exception as e:
            self.get_logger().error(f"接続エラー: {e}")

    async def send_command(self, command):
        try:
            await self.client.write_gatt_char(CHAR_UUID, command.encode('utf-8'))
            self.get_logger().info(f"📡 送信: {command}")
        except Exception as e:
            self.get_logger().error(f"送信エラー: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BLEControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.loop.run_until_complete(node.client.disconnect())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
