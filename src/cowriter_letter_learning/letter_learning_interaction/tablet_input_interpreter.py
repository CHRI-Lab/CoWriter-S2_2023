#!/usr/bin/env python3

import os
import logging
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import String, Empty, Int32MultiArray
from geometry_msgs.msg import PointStamped
from typing import List, Optional
from interface.msg import Shape as ShapeMsg  # type: ignore

from interface.srv import *

from letter_learning_interaction.include.shape_learner_manager import (
    ShapeLearnerManager,
    Shape,
)
from letter_learning_interaction.include.shape_modeler import ShapeModeler

import rclpy
from rclpy.node import Node

word_logger = logging.getLogger("word_logger")


class TabletInputInterpreter(Node):
    """
    Interprets interaction events from the tablet and converts them
    into appropriate messages for shape_learning interaction nodes.

    Attributes:
        publish_shapes (rospy.Publisher):
            Publisher for processed user shapes.

        word_logger (logging.Logger): Logger for word demonstrations.

        position_to_shape_mapping_method (str): Method used to map user
            demonstrations to intended shape.

        shape_preprocessing_method (str):
            Method used to preprocess user-drawn shapes.

        strokes (List[np.ndarray]):
            List of strokes representing a user-drawn shape.

        active_shape_for_demonstration_type (int):
            Active shape type for the user's demonstrations.
    """

    def __init__(self, path: Optional[str] = None):
        super().__init__("tablet_input_interpreter")
        # self.publish_shapes = self.create_publisher(
        #     ShapeMsg, "user_shapes_processed", 10
        # )
        self.clear_all_shapes_service = self.create_client(
            ShapeAtLocation, "shape_at_location"
        )
        self.clear_all_shapes_service.wait_for_service(timeout_sec=1.0)


        GESTURE_TOPIC = "gesture_info"
        # only 1 argument

        # self.get_parameter(
        #     "gesture_info_topic", "gesture_info"
        # ).value
        USER_DRAWN_SHAPES_TOPIC = "user_drawn_shapes"
        # self.declare_parameter(
        #     "user_drawn_shapes_topic", "user_drawn_shapes"
        # ).value
        PROCESSED_USER_SHAPE_TOPIC = "user_shapes_processed"
        # self.declare_parameter(
        #     "processed_user_shape_topic", "user_shapes_processed"
        # ).value

        # Init shape publisher and tablet interpreter
        self.publish_shapes = self.create_publisher(
            ShapeMsg, PROCESSED_USER_SHAPE_TOPIC, 10
        )

        # Listen for gesture representing active demo shape
        self.create_subscription(
            PointStamped, GESTURE_TOPIC, self.on_set_active_shape_gesture, 10
        )

        # Listen for user-drawn shapes
        self.create_subscription(
            Int32MultiArray,
            USER_DRAWN_SHAPES_TOPIC,
            self.user_shape_preprocessor,
            10,
        )

        self.word_logger = logging.getLogger("word_logger")
        self.configure_logging(path)
        self.position_to_shape_mapping_method: str = (
            "basedOnClosestShapeToPosition"
        )
        self.shape_preprocessing_method: str = "merge"
        self.strokes: List = []
        self.active_shape_for_demonstration_type: Optional[int] = None

        self.shape_at_location_client = self.create_client(
            ShapeAtLocation, "shape_at_location"
        )
        self.possible_to_display_shape_client = self.create_client(
            IsPossibleToDisplayNewShape, "possible_to_display_shape"
        )
        self.closest_shapes_to_location_client = self.create_client(
            ClosestShapesToLocation, "closest_shapes_to_location"
        )
        self.display_shape_at_location_client = self.create_client(
            DisplayShapeAtLocation, "display_shape_at_location"
        )
        self.index_of_location_client = self.create_client(
            IndexOfLocation, "index_of_location"
        )

    def shape_at_location(self, request):
        while not self.clear_all_shapes_service.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info("Service is not available, waiting...")

        future = self.clear_all_shapes_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def configure_logging(self, path: Optional[str] = None) -> None:
        """
        Configures logging for the word demonstrations logger.

        Args:
            path (str, optional): Path to log file. Defaults to None.
        """
        if path is None:
            path = "/tmp"
        if path:
            if os.path.isdir(path):
                path = os.path.join(path, "words_demonstrations.log")
            handler = logging.FileHandler(path)
            handler.setLevel(logging.DEBUG)
            formatter = logging.Formatter(
                "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
            )
            handler.setFormatter(formatter)
        else:
            handler = logging.NullHandler()

        self.word_logger.addHandler(handler)
        self.word_logger.setLevel(logging.DEBUG)

    # ---------------------------------------------------- LISTENING FOR USER SHAPE

    def user_shape_preprocessor(self, message):
        """
        Processes incoming user-drawn shape messages.

        Args:
            message (Path): A ROS Path message containing the shape's
                            points.
        """
        self.get_logger().info("received word")
        pts = [
            (message.data[i * 2], message.data[i * 2 + 1])
            for i in range(int(len(message.data) / 2))
        ]
        nb_pts = pts[0][1]
        x = []
        y = []
        index = 0
        i = 0
        while i < len(pts[1:]):
            x.append(pts[1:][i][0])
            y.append(pts[1:][i][1])
            index += 1
            i += 1
            if index == nb_pts:
                shape = []
                index = 0
                if i < len(pts[1:]):
                    nb_pts = pts[1:][i][1]
                for j in x:
                    shape.append(j)
                for j in y:
                    shape.append(-j)
                self.strokes.append(np.reshape(shape, (-1, 1)))
                x = []
                y = []
                i += 1
        self.on_user_drawn_shape_received()

    # ------------------------------------------------------- PROCESSING USER SHAPE
    def on_user_drawn_shape_received(self):
        """
        Handles the processing and publishing of a user-drawn shape.

        Args:
            strokes (List[np.ndarray]): List of strokes representing a
                                        user-drawn shape.
            shape_preprocessing_method (str): Method used to preprocess
                                              user-drawn shapes.
            position_to_shape_mapping_method (str): Method used to map
                user demonstrations to intended shape.
        """
        # Log all the strokes
        xy_paths = []
        for stroke in self.strokes:
            stroke = stroke.flatten().tolist()
            number_of_pts: int = len(stroke) // 2
            # Stroke is formatted as [x0, ..., xn, y0, ..., yn]
            # Use number_of_pts indexing and zip to get [(x0, y0), ..., (xn, yn)]
            xy_paths.append(
                list(zip(stroke[:number_of_pts], stroke[number_of_pts:]))
            )
        self.get_logger().info(str(xy_paths))

        # Preprocess to turn multiple strokes into one path
        if self.shape_preprocessing_method == "merge":
            path = self.process_shape_merge_strokes()
        elif self.shape_preprocessing_method == "longestStroke":
            path = self.process_shape_longest_stroke()
        else:
            path = self.process_shape_first_stroke()

        # Publish shape message
        demo_shape_received = Shape(path=path)
        # self.get_logger().info(np.float32(demo_shape_received.path))


        shape_message = self.make_shape_message(demo_shape_received)
        self.publish_shapes.publish(shape_message)
        self.get_logger().info("published processed shape")

    # ---------------------------------------- FORMATTING SHAPE OBJECT INTO ROS MSG
    def make_shape_message(self, shape: Shape) -> ShapeMsg:
        """
        Converts a Shape object into a ROS Shape message.

        Args:
            shape (ShapeLearnerManager.Shape): The shape object to
                                               convert.

        Returns:
            ShapeMsg: The corresponding ROS Shape message.
        """
        shape_message = ShapeMsg()
        if shape.path is not None:
            self.get_logger().info("path is not none")
            self.get_logger().info(str(shape.path))
            shape_message.path = [float( point) for point in shape.path]
        if shape.shape_id is not None:
            shape_message.shape_id = shape.shape_id
        if shape.shape_type is not None:
            shape_message.shape_type = shape.shape_type
        if shape.shape_type_code is not None:
            shape_message.shape_type_code = shape.shape_type_code
        if shape.params_to_vary is not None:
            shape_message.params_to_vary = shape.params_to_vary
        if shape.param_values is not None:
            shape_message.param_values = shape.param_values

        return shape_message

    # ------------------------------------------------- SHAPE PREPROCESSING METHODS
    def process_shape_longest_stroke(self) -> np.ndarray:
        """
        Selects the longest stroke from the user-drawn shape's strokes.

        Returns:
            np.ndarray: The longest stroke in the user-drawn shape.
        """
        length_longest_stroke: int = 0
        for stroke in self.strokes:
            stroke_length: int = stroke.shape[0]
            if stroke_length > length_longest_stroke:
                longest_stroke: np.ndarray = stroke
                length_longest_stroke: int = stroke_length

        return longest_stroke  # type: ignore

    def process_shape_merge_strokes(self) -> np.ndarray:
        """
        Merges all the strokes of a user-drawn shape into a single path

        Returns:
            np.ndarray:
                The merged path representing the user-drawn shape.
        """
        x_shape: List[np.ndarray] = []
        y_shape: List[np.ndarray] = []
        for stroke in self.strokes:
            number_of_pts = stroke.shape[0] // 2
            x_shape.extend(stroke[:number_of_pts, 0])
            y_shape.extend(stroke[number_of_pts:, 0])

        return np.array(x_shape + y_shape)

    def process_shape_first_stroke(self) -> np.ndarray:
        """
        Selects the first stroke from the user-drawn shape's strokes.

        Returns:
            np.ndarray: The first stroke in the user-drawn shape.
        """
        return self.strokes[0]

    # ----------------- PROCESS GESTURES FOR SETTING ACTIVE SHAPE FOR DEMONSTRATION
    def on_set_active_shape_gesture(self, message: PointStamped):
        """
        Handles the gesture input to set the active shape for
        demonstration.

        Args:
            message (PointStamped): A PointStamped message containing
                                    the gesture's position.
        """
        gesture_location = [message.point.x, message.point.y]
        request = shapeAtLocationRequest()
        request.location.x = gesture_location[0]
        request.location.y = gesture_location[1]
        response = self.shape_at_location(request)
        shape_type_code = response.shape_type_code
        shape_id = response.shape_id

        if shape_type_code != -1 and shape_id != -1:  # type: ignore
            self.active_shape_for_demonstration_type = shape_type_code  # type: ignore

            self.get_logger().info(
                f"Setting active shape to shape \
                {shape_type_code}"
            )  # type: ignore


def main(args=None):
    rclpy.init(args=args)
    node = TabletInputInterpreter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
