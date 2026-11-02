import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer
from rosbot_msgs.msg import BlockDetection

class BlockDetector(Node):
    def __init__(self):
        super().__init__('block_detector')

        self.bridge = CvBridge()

        self.declare_parameter(
            'image_topic',
            '/depth_cam/color/image_raw'
        )

        self.declare_parameter(
            'depth_topic',
            '/depth_cam/depth/image_raw'
        )

        self.declare_parameter(
            'camera_info_topic',
            '/depth_cam/color/camera_info'
        )

        image_topic = self.get_parameter(
            'image_topic'
        ).value

        depth_topic = self.get_parameter(
            'depth_topic'
        ).value

        camera_info_topic = self.get_parameter(
            'camera_info_topic'
        ).value

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )

        self.color_sub = Subscriber(
            self,
            Image,
            image_topic
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            depth_topic
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.08
        )

        self.sync.registerCallback(
            self.image_callback
        )

        self.debug_pub = self.create_publisher(
            Image,
            '~/debug_image',
            10
        )

        self.detection_pub = self.create_publisher(
            BlockDetection,
            '~/block_detections',
            10
        )

        self.color_ranges = {
            'red': [
                ((0, 110, 90), (12, 255, 255)),
                ((170, 110, 90), (179, 255, 255))
            ],
            'green': [
                ((35, 80, 60), (89, 255, 255))
            ],
            'blue': [
                ((90, 90, 70), (115, 255, 255))
            ]
        }

        self.draw_colors = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0)
        }

        self.get_logger().info(
            f'RGB: {image_topic}'
        )

        self.get_logger().info(
            f'Depth: {depth_topic}'
        )

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def get_depth(self, depth_image, contour):
        mask = np.zeros(
            depth_image.shape,
            dtype=np.uint8
        )

        cv2.drawContours(
            mask,
            [contour],
            -1,
            255,
            -1
        )

        kernel = np.ones(
            (5, 5),
            dtype=np.uint8
        )

        mask = cv2.erode(
            mask,
            kernel,
            iterations=1
        )

        depth_values = depth_image[
            mask > 0
        ]

        depth_values = depth_values[
            depth_values > 0
        ]

        if len(depth_values) == 0:
            return None

        depth_raw = np.median(
            depth_values
        )

        depth_m = depth_raw * 0.001

        return depth_m

    def pixel_to_3d(self, u, v, z):
        if (
            self.fx is None
            or self.fy is None
            or self.cx is None
            or self.cy is None
        ):
            return None

        x = (
            (u - self.cx)
            * z
            / self.fx
        )

        y = (
            (v - self.cy)
            * z
            / self.fy
        )

        return x, y, z

    def image_callback(
        self,
        color_msg,
        depth_msg
    ):
        frame = self.bridge.imgmsg_to_cv2(
            color_msg,
            desired_encoding='bgr8'
        )

        depth_image = self.bridge.imgmsg_to_cv2(
            depth_msg,
            desired_encoding='passthrough'
        )

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        height, width = frame.shape[:2]
        image_area = height * width

        detections = []

        for name, ranges in self.color_ranges.items():
            mask = np.zeros(
                (height, width),
                dtype=np.uint8
            )

            for lower, upper in ranges:
                mask |= cv2.inRange(
                    hsv,
                    np.array(
                        lower,
                        dtype=np.uint8
                    ),
                    np.array(
                        upper,
                        dtype=np.uint8
                    )
                )

            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (3, 3)
            )

            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                kernel
            )

            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                kernel
            )

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            candidates = []

            for contour in contours:
                area = cv2.contourArea(
                    contour
                )

                if (
                    area < 100
                    or area > image_area * 0.20
                ):
                    continue

                x, y, w, h = cv2.boundingRect(
                    contour
                )

                aspect_ratio = (
                    w / max(h, 1)
                )

                if not (
                    0.4
                    <= aspect_ratio
                    <= 2.5
                ):
                    continue

                hull = cv2.convexHull(
                    contour
                )

                hull_area = cv2.contourArea(
                    hull
                )

                if hull_area == 0:
                    continue

                solidity = (
                    area / hull_area
                )

                if solidity < 0.85:
                    continue

                perimeter = cv2.arcLength(
                    contour,
                    True
                )

                if perimeter == 0:
                    continue

                polygon = cv2.approxPolyDP(
                    contour,
                    0.035 * perimeter,
                    True
                )

                if not (
                    4
                    <= len(polygon)
                    <= 8
                ):
                    continue

                rect = cv2.minAreaRect(
                    contour
                )

                rect_area = (
                    rect[1][0]
                    * rect[1][1]
                )

                if rect_area == 0:
                    continue

                rectangularity = (
                    area / rect_area
                )

                if rectangularity < 0.60:
                    continue

                moments = cv2.moments(
                    contour
                )

                if moments['m00'] == 0:
                    continue

                u = int(
                    moments['m10']
                    / moments['m00']
                )

                v = int(
                    moments['m01']
                    / moments['m00']
                )

                score = (
                    0.5 * solidity
                    + 0.5 * rectangularity
                )

                depth = self.get_depth(depth_image, contour)

                if depth is None:
                    continue

                rect_width, rect_height = rect[1]

                physical_width = rect_width * depth / self.fx
                physical_height = rect_height * depth / self.fy

                min_size = 0.02
                max_size = 0.06

                if not (
                    min_size <= physical_width <= max_size
                    and min_size <= physical_height <= max_size
                ):
                    continue

                position = None

                if depth is not None:
                    position = self.pixel_to_3d(
                        u,
                        v,
                        depth
                    )

                candidates.append({
                    'color': name,
                    'center': (u, v),
                    'rect': rect,
                    'score': score,
                    'area': area,
                    'depth': depth,
                    'position': position
                })

            candidates.sort(
                key=lambda candidate:
                candidate['score'],
                reverse=True
            )

            detections.extend(
                candidates
            )

        for detection in detections:
            name = detection['color']

            u, v = detection['center']

            color = self.draw_colors[
                name
            ]

            box = cv2.boxPoints(
                detection['rect']
            ).astype(int)

            cv2.drawContours(
                frame,
                [box],
                0,
                color,
                2
            )

            cv2.circle(
                frame,
                (u, v),
                4,
                color,
                -1
            )

            cv2.putText(
                frame,
                f"{name}: "
                f"{detection['score']:.2f}",
                (u + 8, v - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                2
            )

            position = detection[
                'position'
            ]

            if position is not None:
                x, y, z = position

                msg = BlockDetection()
                msg.header = color_msg.header
                msg.color = name

                msg.position.x = x
                msg.position.y = y
                msg.position.z = z

                self.detection_pub.publish(
                    msg
                )

                cv2.putText(
                    frame,
                    (
                        f"X:{x:.2f} "
                        f"Y:{y:.2f} "
                        f"Z:{z:.2f}m"
                    ),
                    (u + 8, v),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.40,
                    color,
                    1
                )

        debug_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        debug_msg.header = (
            color_msg.header
        )

        self.debug_pub.publish(
            debug_msg
        )


def main(args=None):
    rclpy.init(args=args)

    node = BlockDetector()

    try:
        rclpy.spin(node)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()