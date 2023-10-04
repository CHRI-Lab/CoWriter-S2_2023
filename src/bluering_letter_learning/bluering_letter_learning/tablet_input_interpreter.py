#!/usr/bin/env python

"""
Listens for interaction events from the tablet and converts them into
appropriate messages for shape_learning interaction nodes.

Currently implemented:
- Receiving user-drawn shapes (demonstrations for learning alg.) as a series
of Path messages of strokes and processing the shape by keeping only the 
longest stroke and determining which shape being shown by the display_manager 
the demonstration was for.
- Receiving the location of a gesture on the tablet which represents which 
shape to give priority to if the demonstration was drawn next to multiple
shapes (if using the 'basedOnClosestShapeToPosition' method to map user demo to
intended shape).

Implemented but not in use: 
- Receiving touch and long-touch gestures and converting that to feedback for
the learning algorithm from when it was touch-feedback only.
"""
import os.path
import rclpy
from rclpy.node import Node
import numpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped

from shape_learning.shape_learner_manager import Shape

from interface.srv import ShapeAtLocation
from interface.msg import Shape as ShapeMsg

import logging


class TabletInputInterpreter(Node):
    def __init__(self):
        super().__init__("tablet_input_interpreter")

        self.wordLogger = logging.getLogger("word_logger")
        self.configure_logging()

        self.strokes = []

        self.get_logger.info("[tablet_input_interpreter] Starting...")
        # ? Name of topic to get gestures representing the active shape for demonstration
        GESTURE_TOPIC = self.declare_parameter(
            "~gesture_info_topic", "gesture_info"
        ).value
        # ? Name of topic to get user drawn raw shapes on
        USER_DRAWN_SHAPES_TOPIC = self.declare_parameter(
            "~user_drawn_shapes_topic", "user_drawn_shapes"
        ).value
        # ? Name of topic to publish processed shapes on
        PROCESSED_USER_SHAPE_TOPIC = self.declare_parameter(
            "~processed_user_shape_topic", "user_shapes_processed"
        ).value

        # ? listen for gesture representing active demo shape
        self.create_subscription(
            PointStamped, GESTURE_TOPIC, self.onSetActiveShapeGesture
        )
        # ? listen for user-drawn shapes
        self.create_subscription(
            Path, USER_DRAWN_SHAPES_TOPIC, self.userShapePreprocessor
        )

        self.pub_shapes = self.create_publisher(
            ShapeMsg, PROCESSED_USER_SHAPE_TOPIC, 10
        )

    # HACK: should properly configure the path from an option
    def configure_logging(self, path="/tmp"):
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

        self.wordLogger.addHandler(handler)
        self.wordLogger.setLevel(logging.DEBUG)

    # ---------------------------------------------------- LISTENING FOR USER SHAPE
    def userShapePreprocessor(self, message):
        # rospy.loginfo(f'[tablet_input_interpreter][userShapePreprocessor] strokes = {strokes}')

        if (
            len(message.poses) == 0
        ):  # ? a message with 0 poses signifies the shape has no more strokes
            if len(self.strokes) > 0:
                self.onUserDrawnShapeReceived()
            else:
                self.get_logger.info(
                    "[tablet_input_interpreter][userShapePreprocessor] empty demonstration. ignoring"
                )

            self.strokes = []

        else:  # ? new stroke in shape - add it
            self.get_logger.info(
                f"[tablet_input_interpreter][userShapePreprocessor] Got stroke to write with {len(message.poses)} points"
            )
            x_shape = []
            y_shape = []
            for poseStamped in message.poses:
                x_shape.append(poseStamped.pose.position.x)
                y_shape.append(-poseStamped.pose.position.y)

            numPointsInShape = len(x_shape)

            # ? format as necessary for shape_modeler (x0, x1, x2, ..., y0, y1, y2, ...)'
            shape = []
            shape[0:numPointsInShape] = x_shape
            shape[numPointsInShape:] = y_shape

            shape = numpy.reshape(
                shape, (-1, 1)
            )  # ? explicitly make it 2D array with only one column
            self.strokes.append(shape)
            # rospy.loginfo(f'[tablet_input_interpreter][userShapePreprocessor] shape = {shape}')

    # ------------------------------------------------------- PROCESSING USER SHAPE
    def onUserDrawnShapeReceived(self, shapePreprocessingMethod="merge"):
        #### Log all the strokes
        xypaths = []
        for stroke in self.strokes:
            stroke = stroke.flatten().tolist()
            nbpts = int(len(stroke) / 2)
            # print(f'[tablet_input_interpreter][onUserDrawnShapeReceived] stroke = {stroke}')
            print(
                f"[tablet_input_interpreter][onUserDrawnShapeReceived] len(stroke) = {len(stroke)} | len(stroke)/2 = {len(stroke)/2} | nbpts = {nbpts}"
            )
            # print(f'[tablet_input_interpreter][onUserDrawnShapeReceived] stroke[:nbpts] = {stroke[:nbpts]} | len = {len(stroke[:nbpts])}')
            # print(f'[tablet_input_interpreter][onUserDrawnShapeReceived] stroke[nbpts:] = {stroke[nbpts:]} | len = {len(stroke[nbpts:])}')
            xypaths.append(zip(stroke[:nbpts], stroke[nbpts:]))
        self.wordLogger.info(str(xypaths))
        ####

        # preprocess to turn multiple strokes into one path
        if shapePreprocessingMethod == "merge":
            path = self.processShape_mergeStrokes(self.strokes)
        elif shapePreprocessingMethod == "longestStroke":
            path = self.processShape_longestStroke(self.strokes)
        else:
            path = self.processShape_firstStroke(self.strokes)

        demoShapeReceived = Shape(path=path)
        shapeMessage = self.makeShapeMessage(demoShapeReceived)
        self.pub_shapes.publish(shapeMessage)

    ###-------- FORMATTING SHAPE OBJECT INTO ROS MSG --------###
    # expects a ShapeLearnerManager.Shape as input
    def makeShapeMessage(self, shape):
        shapeMessage = ShapeMsg()
        if shape.path is not None:
            shapeMessage.path = shape.path
        if shape.shape_id is not None:
            shapeMessage.shape_id = shape.shapeID
        if shape.shape_type is not None:
            shapeMessage.shape_type = shape.shapeType
        if shape.shape_type_code is not None:
            shapeMessage.shape_type_code = shape.shapeType_code
        if shape.params_to_vary is not None:
            shapeMessage.params_to_vary = shape.paramsToVary
        if shape.param_values is not None:
            shapeMessage.param_values = shape.paramValues

        return shapeMessage

    # ------------------------------------------------- SHAPE PREPROCESSING METHODS
    def processShape_longestStroke(self, strokes):
        length_longestStroke = 0
        for stroke in strokes:
            strokeLength = stroke.shape[
                0
            ]  # how many rows in array: number of points
            if strokeLength > length_longestStroke:
                longestStroke = stroke
                length_longestStroke = strokeLength
        return longestStroke

    def processShape_mergeStrokes(self, strokes):
        x_shape = []
        y_shape = []
        for stroke in strokes:
            nbpts = int(stroke.shape[0] / 2)
            x_shape.extend(stroke[:nbpts, 0])
            y_shape.extend(stroke[nbpts:, 0])

        return numpy.array(x_shape + y_shape)

    def processShape_firstStroke(self, strokes):
        return strokes[0]

    # ----------------- PROCESS GESTURES FOR SETTING ACTIVE SHAPE FOR DEMONSTRATION
    def onSetActiveShapeGesture(self, message):
        gestureLocation = [message.point.x, message.point.y]
        # map gesture location to shape drawn
        shape_at_location = self.create_client(
            ShapeAtLocation, "shape_at_location"
        )
        request = ShapeAtLocation.Request()
        request.location.x = gestureLocation[0]
        request.location.y = gestureLocation[1]
        while not shape_at_location.wait_for_service(timeout_sec=1.0):
            self.get_logger.info("service not available, waiting again...")
        future = shape_at_location.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        shapeType_code = response.shape_type_code
        shapeID = response.shape_id

        if shapeType_code != -1 and shapeID != -1:
            self.get_logger.info(
                "Setting active shape to shape " + str(shapeType_code)
            )


def main(args=None):
    rclpy.init(args=args)
    tablet_input_interpreter = TabletInputInterpreter()
    rclpy.spin(tablet_input_interpreter)
    tablet_input_interpreter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
