
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.declare_parameter(
            'image_path',
            '/home/nmmal/ros2_cv_ws/test_images/blocks.jpg'
        )

        self.declare_parameter(
            'image_topic',
            '/camera/color/image_raw'
        )

        image_path = self.get_parameter('image_path').value
        image_topic = self.get_parameter('image_topic').value

        self.image = cv2.imread(image_path)

        if self.image is None:
            raise FileNotFoundError(
                f'Could not load image: {image_path}'
            )

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            image_topic,
            10
        )

        self.timer = self.create_timer(
            0.2,
            self.publish_image
        )

        self.get_logger().info(
            f'Loaded image: {image_path}'
        )

        self.get_logger().info(
            f'Publishing to {image_topic} at 5 FPS'
        )

    def publish_image(self):
        msg = self.bridge.cv2_to_imgmsg(
            self.image,
            encoding='bgr8'
        )

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ImagePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
