#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
import numpy
from scipy import interpolate
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from shape_learning.shape_modeler import ShapeModeler


class ShapeHelper:
    """
    Helpers to generate strokes for a letter
    """

    def __init__(
        self,
        frame,
        numDesiredShapepoints,
        numPointsShapeModeler,
        t0,
        delayBeforeExecuting,
        generatedWordLogger,
    ) -> None:
        self.frame = frame
        self.numDesiredShapepoints = numDesiredShapepoints
        self.numPointsShapeModeler = numPointsShapeModeler
        self.downsampleFactor = float(numPointsShapeModeler - 1) / float(
            numDesiredShapepoints - 1
        )
        self.t0 = t0
        self.delayBeforeExecuting = delayBeforeExecuting
        self.generatedWordLogger = generatedWordLogger
        return

    def downsampleShape(self, shape) -> numpy.ndarray:
        # downsample user-drawn shape so appropriate size for shapeLearner
        numPointsInShape = int(len(shape) / 2)
        x_shape = shape[0:numPointsInShape]
        y_shape = shape[numPointsInShape:]

        if isinstance(
            x_shape, numpy.ndarray
        ):  # convert arrays to lists for interp1d
            x_shape = (x_shape.T).tolist()[0]
            y_shape = (y_shape.T).tolist()[0]

        # make shape have the same number of points as the shape_modeler
        t_current = numpy.linspace(0, 1, numPointsInShape)
        t_desired = numpy.linspace(0, 1, self.numPointsShapeModeler)
        f = interpolate.interp1d(t_current, x_shape, kind="linear")
        x_shape = f(t_desired)
        f = interpolate.interp1d(t_current, y_shape, kind="linear")
        y_shape = f(t_desired)

        shape = []
        shape[0 : self.numPointsShapeModeler] = x_shape
        shape[self.numPointsShapeModeler :] = y_shape

        shape = ShapeModeler.normaliseShapeHeight(numpy.array(shape))
        # explicitly make it 2D array with only one column
        shape = numpy.reshape(shape, (-1, 1))

        return shape

    def make_bounding_box_msg(
        self, bbox, selected=False
    ) -> Float64MultiArray():
        bb = Float64MultiArray()
        bb.layout.data_offset = 0
        dim = MultiArrayDimension()
        # we use the label of the first dimension to carry the selected/not selected infomation
        dim.label = "bb" if not selected else "select"
        bb.layout.dim = [dim]

        x_min, y_min, x_max, y_max = bbox
        bb.data = [x_min, y_min, x_max, y_max]

        # print(f'[ShapeHelper][make_bounding_box_msg] bb = {bb}')

        return bb

    def make_traj_msg(self, shapedWord, deltaT, log=False) -> Path():
        traj = Path()
        traj.header.frame_id = self.frame
        traj.header.stamp = rospy.Time.now() + rospy.Duration(
            self.delayBeforeExecuting
        )

        pointIdx = 0
        paths = shapedWord.get_letters_paths()
        # rospy.loginfo(f'[ShapeHelper][make_traj_msg] paths : {paths} | len(paths) = {len(paths)} | log : {log}')

        if log:
            self.generatedWordLogger.info(
                "%s" % [[(x, -y) for x, y in path] for path in paths]
            )

        for path in paths:
            # rospy.loginfo(f'[ShapeHelper][make_traj_msg]   [ ] path : {path}')
            first = True
            for x, y in path:
                point = PoseStamped()

                point.pose.position.x = x
                point.pose.position.y = y
                point.header.frame_id = self.frame
                # TODO allow for variable time between points for now
                point.header.stamp = rospy.Time(self.t0 + pointIdx * deltaT)

                # rospy.loginfo(f'[ShapeHelper][make_traj_msg]      [ ] x, y = {x}, {y}')

                if first:
                    point.header.seq = 1
                    first = False

                traj.poses.append(deepcopy(point))

                # rospy.loginfo(f'[ShapeHelper][make_traj_msg]      [+] len(traj.poses) = {len(traj.poses)}')

                pointIdx += 1

        # rospy.loginfo(f'[ShapeHelper][make_traj_msg]   [+] traj set')
        return traj
