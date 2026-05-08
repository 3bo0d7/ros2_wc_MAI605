import io
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

import qrcode
from PIL import Image as PILImage
import numpy as np


class SimQrCamera(Node):
    def __init__(self):
        super().__init__('sim_qr_camera_node')

        self.declare_parameter('qr_text', 'BIN_A')
        self.declare_parameter('frame_id', 'qr_camera_frame')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('wait_for_scan_trigger', True)

        self.qr_text = self.get_parameter('qr_text').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = float(self.get_parameter('publish_rate').value)
        self.wait_for_scan_trigger = bool(self.get_parameter('wait_for_scan_trigger').value)
        self.scan_ready = not self.wait_for_scan_trigger

        self.image_pub = self.create_publisher(
            Image,
            '/qr_sort/sim_camera/image_raw',
            10,
        )

        self.text_pub = self.create_publisher(
            String,
            '/qr_sort/sim_camera/qr_text',
            10,
        )

        self.create_subscription(Bool, '/qr_sort/scan_ready', self.on_scan_ready, 10)

        self.image_msg = self.make_qr_image(self.qr_text)
        self.timer = self.create_timer(1.0 / rate, self.publish_image)

        if self.wait_for_scan_trigger:
            self.get_logger().info(
                f'Simulated QR camera armed with QR text "{self.qr_text}". '
                'Waiting for /qr_sort/scan_ready before publishing images.'
            )
        else:
            self.get_logger().info(
                f'Simulated QR camera publishing QR text "{self.qr_text}" on '
                '/qr_sort/sim_camera/image_raw'
            )

    def make_qr_image(self, text):
        qr = qrcode.QRCode(
            version=2,
            error_correction=qrcode.constants.ERROR_CORRECT_M,
            box_size=12,
            border=4,
        )
        qr.add_data(text)
        qr.make(fit=True)

        img = qr.make_image(fill_color='black', back_color='white').convert('RGB')

        # Put QR code on a larger white canvas to make detection easier.
        canvas = PILImage.new('RGB', (480, 480), 'white')
        img = img.resize((360, 360))
        canvas.paste(img, (60, 60))

        arr = np.asarray(canvas, dtype=np.uint8)

        msg = Image()
        msg.height = arr.shape[0]
        msg.width = arr.shape[1]
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = arr.shape[1] * 3
        msg.data = arr.tobytes()
        return msg

    def on_scan_ready(self, msg):
        if not msg.data:
            return

        if not self.scan_ready:
            self.get_logger().info(
                'Scan trigger received. Publishing QR images for zbar_ros decode.'
            )
        self.scan_ready = True

    def publish_image(self):
        if not self.scan_ready:
            self.get_logger().info(
                'Waiting for robot scan pose trigger before publishing QR image.',
                throttle_duration_sec=5.0,
            )
            return

        now = self.get_clock().now().to_msg()

        self.image_msg.header.stamp = now
        self.image_msg.header.frame_id = self.frame_id
        self.image_pub.publish(self.image_msg)

        text_msg = String()
        text_msg.data = self.qr_text
        self.text_pub.publish(text_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimQrCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
