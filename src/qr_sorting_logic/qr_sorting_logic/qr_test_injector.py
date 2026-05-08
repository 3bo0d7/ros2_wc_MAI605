"""Publishes a fake QR decode to /barcode for quick testing."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class QrTestInjector(Node):
    def __init__(self) -> None:
        super().__init__('qr_test_injector')
        # This helper bypasses zbar_ros and publishes directly to /barcode.
        # It is useful for fast decision-node tests, but it is not used by the
        # main demo launch because the project demonstration keeps zbar_ros in
        # the perception pipeline.
        self.declare_parameter('qr_text', 'BIN_A')
        self.publisher = self.create_publisher(String, '/barcode', 10)
        self.timer = self.create_timer(0.5, self.publish_once)
        self.sent = False

    def publish_once(self) -> None:
        if self.sent:
            return
        msg = String()
        msg.data = str(self.get_parameter('qr_text').value)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published fake QR decode: {msg.data}')
        self.sent = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QrTestInjector()
    try:
        rclpy.spin_once(node, timeout_sec=2.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
