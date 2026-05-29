import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2

class PCMonitorNode(Node):
    def __init__(self):
        super().__init__('pc_monitor_node')
        self.bridge = CvBridge()
        self.latest_image = None
        
        # ロボットから画像を受け取るSubscriber
        self.image_sub = self.create_subscription(
            msg_type=Image,
            topic="image_to_smartphone",
            callback=self.image_callback,
            qos_profile=qos_profile_sensor_data,
        )
        
        # ロボットへ判断を送るPublisher
        self.judgment_pub = self.create_publisher(
            msg_type=Bool, topic="user_judgment", qos_profile=10
        )
        
        self.get_logger().info('PCモニターを起動しました。画像を受信待機中...')

    def image_callback(self, msg):
        self.latest_image = msg

    def update_window(self):
        if self.latest_image is not None:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            cv2.imshow("Robot Camera (Press 'y' for True, 'n' for False)", cv_image)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('y'):
            self.send_judgment(True)
        elif key == ord('n'):
            self.send_judgment(False)

    def send_judgment(self, result):
        msg = Bool()
        msg.data = result
        self.judgment_pub.publish(msg)
        self.latest_image = None
        
        # 送信したら画面を閉じる（次の画像が来るまで）
        cv2.destroyAllWindows()
        
        judgment_str = "True (タイプ)" if result else "False (違う)"
        self.get_logger().info(f'判定 [{judgment_str}] をロボットへ送信しました！')

def main(args=None):
    rclpy.init(args=args)
    node = PCMonitorNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.update_window()
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
