from std_msgs.msg import (
    String,
    Int32MultiArray,
    MultiArrayDimension,
    MultiArrayLayout,
)
from nav_msgs.msg import Path

import rclpy
from rclpy.node import Node

from .letters import LETTER

TOPIC_WORDS_TO_WRITE = "words_to_write"
TOPIC_SHAPES_TO_DRAW = "shapes_to_draw"
SHAPE_TOPIC = "write_traj"


to_draw = []


class UIBackend(Node):
    def __init__(self):
        super().__init__("ui_backend")
        self.publisher_shape_to_draw = self.create_publisher(
            Int32MultiArray, TOPIC_SHAPES_TO_DRAW, 10
        )
        self.subscription_words = self.create_subscription(
            String, TOPIC_WORDS_TO_WRITE, self.callback_words_to_write, 10
        )
        self.subscription_traj = self.create_subscription(
            Path, SHAPE_TOPIC, self.on_traj, 10
        )

    # takes a list [(x, y), (x, y), ... , (x, y)] as input,
    # returns a msg ready to publish
    def pack_writing_pts(self, pts):
        unpacked_pts = []
        for p in pts:
            unpacked_pts.append(p[0])
            unpacked_pts.append(p[1])

        dim1 = MultiArrayDimension()
        dim1.label = "pts"
        dim1.size = int(len(unpacked_pts) / 2)
        dim1.stride = len(unpacked_pts)

        dim2 = MultiArrayDimension()
        dim2.label = "pt"
        dim2.size = 2
        dim2.stride = 2

        layout = MultiArrayLayout()
        layout.dim = [dim1, dim2]
        layout.data_offset = 0

        pt_msg = Int32MultiArray()
        pt_msg.layout = layout
        pt_msg.data = unpacked_pts

        return pt_msg

    # takes a list [(x, y), (x, y), ... , (x, y)] as input,
    # offset each point by arguement (x, y)
    def offset_shape(shape, x, y):
        return [(p[0] + x, p[1] + y) for p in shape]

    def callback_words_to_write(self, data):
        offset = 0
        for letter in data.data:
            if letter in LETTER.keys():
                print(letter)
                for stroke in LETTER[letter]:
                    # publish_shape_to_draw.publish(
                    #     pack_writing_pts(offset_shape(stroke, offset, 0))
                    # )
                    pass
                offset += 100

    def on_traj(self, traj_path):
        """
        nav_msgs.msg.Path: traj_path
        transform the nao learning traj to a size suitable for writing
        """
        print("traj received")
        points = []
        for pose in traj_path.poses:
            x = int(pose.pose.position.x * 2000)
            y = int(pose.pose.position.y * -2000 + 400)
            points.append((x, y))
        to_draw.append(points)
        print(to_draw)
        msg = self.pack_writing_pts(points)
        self.publisher_shape_to_draw.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    backend_node = UIBackend()
    rclpy.spin(backend_node)
    backend_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
