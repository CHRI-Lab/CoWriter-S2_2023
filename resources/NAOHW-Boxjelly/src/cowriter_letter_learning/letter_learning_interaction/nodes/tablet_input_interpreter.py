#!/usr/bin/env python3

"""
ROS node for interpreting tablet input events & generating appropriate
messages for shape_learning interaction nodes.

This node listens for interaction events from a tablet and processes
them into messages for shape_learning interaction nodes.
It is responsible for handling user-drawn shapes, touch and long-touch
gestures, and determining which shape is being shown by the
display_manager.

Key Features:
- Processes user-drawn shapes, represented as a series of Path messages
of strokes, by keeping only the longest stroke and determining the
intended shape based on the display_manager.
- Handles tablet gesture inputs to set the active shape for
  demonstration, determining which shape to prioritize if the
  demonstration is drawn next to multiple shapes (using the
  'based_on_closest_shape_to_position' method).
- Processes touch and long-touch gestures to provide feedback for the
  learning algorithm.

Node:
    tablet_input_interpreter

Subscribed Topics:
    gesture_info (geometry_msgs/PointStamped):
        Tablet gestures representing the active shape for demonstration
    user_drawn_shapes (nav_msgs/Path):
        User-drawn raw shapes on the tablet.

Published Topics:
    user_shapes_processed (letter_learning_interaction/Shape):
        Processed user-drawn shapes after preprocessing and analysis.

Services:
    shape_at_location (letter_learning_interaction/shape_at_location):
        Service for querying the shape at a given location.
    possible_to_display_shape (letter_learning_interaction/possible_to_display_shape): 
        Service for checking if it's possible to display a shape.
    closest_shapes_to_location (letter_learning_interaction/closest_shapes_to_location):
        Service for querying the closest shapes to a location.
    display_shape_at_location (letter_learning_interaction/display_shape_at_location):
        Service for displaying a shape at a given location.
    index_of_location (letter_learning_interaction/index_of_location):
        Service for getting the index of a given location.

Parameters:
    ~gesture_info_topic (str, default: "gesture_info"):
        Topic name for receiving gesture information.
    ~user_drawn_shapes_topic (str, default: "user_drawn_shapes"):
        Topic name for receiving user-drawn shapes.
    ~processed_user_shape_topic (str, default: "user_shapes_processed"):
        Topic name for publishing processed user shapes.
"""

import os.path
import logging
import rospy
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import String, Empty, Int32MultiArray
from geometry_msgs.msg import PointStamped
from typing import List, Optional
import os.path
import sys
from letter_learning_interaction.msg import Shape as ShapeMsg  # type: ignore
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from srv import *
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../include'))
from shape_learner_manager import ShapeLearnerManager, Shape
from shape_modeler import ShapeModeler



word_logger = logging.getLogger("word_logger")


class TabletInputInterpreter:
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

    def __init__(self, publish_shapes: rospy.Publisher, path: Optional[str] = None):
        self.publish_shapes = publish_shapes
        self.word_logger = logging.getLogger("word_logger")
        self.configure_logging(path)
        self.position_to_shape_mapping_method: str = 'basedOnClosestShapeToPosition'
        self.shape_preprocessing_method: str = "merge"
        self.strokes: List = []
        self.active_shape_for_demonstration_type: Optional[int] = None

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
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
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
        pts = [(message.data[i * 2], message.data[i * 2 + 1]) for i in range(int(len(message.data) / 2))]
        nb_pts = pts[0][1]
        x = []
        y = []
        index = 0
        i = 0
        while i < len(pts[1:]):
            x.append(pts[1:][i][0])
            y.append(pts[1:][i][1])
            index+=1
            i+=1
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
                i+=1
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
                list(zip(stroke[:number_of_pts], stroke[number_of_pts:])))
        word_logger.info(str(xy_paths))

        # Preprocess to turn multiple strokes into one path
        if self.shape_preprocessing_method == 'merge':
            path = self.process_shape_merge_strokes()
        elif self.shape_preprocessing_method == 'longestStroke':
            path = self.process_shape_longest_stroke()
        else:
            path = self.process_shape_first_stroke()

        # Publish shape message
        demo_shape_received = Shape(path=path)
        shape_message = self.make_shape_message(demo_shape_received)
        self.publish_shapes.publish(shape_message)
        

# ---------------------------------------- FORMATTING SHAPE OBJECT INTO ROS MSG
    @staticmethod
    def make_shape_message(shape: Shape) -> ShapeMsg:
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
            shape_message.path = shape.path
        if shape.shape_id is not None:
            shape_message.shapeID = shape.shape_id
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
        try:
            shape_at_location = rospy.ServiceProxy(
                'shape_at_location', shape_at_location)  # type: ignore
            request = shapeAtLocationRequest()
            request.location.x = gesture_location[0]
            request.location.y = gesture_location[1]
            response = shape_at_location(request)
            shape_type_code = response.shape_type_code
            shape_id = response.shape_id
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

        if shape_type_code != -1 and shape_id != -1:  # type: ignore
            self.active_shape_for_demonstration_type = shape_type_code  # type: ignore

            rospy.loginfo(f'Setting active shape to shape \
                {shape_type_code}')  # type: ignore


if __name__ == "__main__":

    rospy.init_node("tablet_input_interpreter")

    # Name of topic to get gestures representing the active shape for demonstration
    GESTURE_TOPIC = rospy.get_param('~gesture_info_topic', 'gesture_info')

    # Name of topic to get user drawn raw shapes on
    USER_DRAWN_SHAPES_TOPIC = rospy.get_param(
        '~user_drawn_shapes_topic', 'user_drawn_shapes')

    # Name of topic to publish processed shapes on
    PROCESSED_USER_SHAPE_TOPIC = rospy.get_param(
        '~processed_user_shape_topic', 'user_shapes_processed')

    # Init shape publisher and tablet interpreter
    publish_shapes = rospy.Publisher(
        PROCESSED_USER_SHAPE_TOPIC, ShapeMsg, queue_size=10)
    interpreter = TabletInputInterpreter(publish_shapes)

    # Listen for gesture representing active demo shape
    gesture_subscriber = rospy.Subscriber(
        GESTURE_TOPIC, PointStamped, interpreter.on_set_active_shape_gesture)

    # Listen for user-drawn shapes
    shape_subscriber = rospy.Subscriber(
        USER_DRAWN_SHAPES_TOPIC, Int32MultiArray, interpreter.user_shape_preprocessor)

    # initialise display manager for shapes (manages positioning of shapes)
    rospy.wait_for_service('shape_at_location')
    rospy.wait_for_service('possible_to_display_shape')
    rospy.wait_for_service('closest_shapes_to_location')
    rospy.wait_for_service('display_shape_at_location')
    rospy.wait_for_service('index_of_location')

    rospy.spin()
