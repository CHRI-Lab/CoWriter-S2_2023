# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

import numpy as np
import sys

# print(sys.path)

from .shape_learner import SettingsStruct
from .shape_learner_manager import ShapeLearnerManager
from .shape_modeler import ShapeModeler

import rclpy
from rclpy.node import Node
from scipy import interpolate

from interface.msg import *
from interface.srv import *

import matplotlib
import matplotlib.pyplot as plt

# matplotlib.use('TkAgg')  # Use the TkAgg backend instead of the PyCharm backend

ALPHABETS_LOWER = 'abcdefghijklmnopqrstuvwxyz'

def generateSettings(
        shapeType,
        init_datasetDirectory='./src/nao_writing/nao_writing/letter_model_datasets/alexis_set_for_children',
        update_datasetDirectory='./src/nao_writing/nao_writing/letter_model_datasets/alexis_set_for_children',
        demo_datasetDirectory='./src/nao_writing/nao_writing/letter_model_datasets/alexis_set_for_children',
):
    paramsToVary = [3]
    initialBounds_stdDevMultiples = np.array([[-6, 6]])
    doGroupwiseComparison = True
    initialParamValue = np.NaN
    initialBounds = np.array([[np.NaN, np.NaN]])
    init_datasetFile = init_datasetDirectory + '/' + shapeType + '.dat'
    update_datasetFile = update_datasetDirectory + '/' + shapeType + '.dat'
    demo_datasetFile = demo_datasetDirectory + '/' + shapeType + '.dat'

    cwd = os.getcwd()
    print(cwd)

    if not os.path.exists(init_datasetFile):
        raise RuntimeError("Dataset not found for shape " + shapeType)

    if not os.path.exists(update_datasetFile):
        try:
            with open(update_datasetFile, 'w') as f:
                pass
        except IOError:
            raise RuntimeError("no writing permission for file " + update_datasetFile)

    if not os.path.exists(demo_datasetFile):
        try:
            with open(demo_datasetFile, 'w') as f:
                pass
        except IOError:
            raise RuntimeError("no writing permission for file " + demo_datasetFile)

    try:
        datasetParam = init_datasetDirectory + '/params.dat'
        with open(datasetParam, 'r') as f:
            line = f.readline()
            test = line.replace('[', '').replace(']\n', '') == shapeType
            while test == False:
                line = f.readline()
                if line:
                    test = line.replace('[', '').replace(']\n', '') == shapeType
                else:
                    break
            if test:
                u = f.readline().replace('\n', '')
                initialParamValue = [(float)(s) for s in u.split(',')]
            else:
                initialParamValue = [0.0, 0.0, 0.0, 0.0, 0.0]
                print("parameters not found for shape " + shapeType + '\n' + 'Default : 0.0')

    except IOError:
        raise RuntimeError("no reading permission for file" + datasetParam)

    settings = SettingsStruct(shape_learning=shapeType,
                              paramsToVary=paramsToVary,
                              doGroupwiseComparison=True,
                              initDatasetFile=init_datasetFile,
                              updateDatasetFiles=[update_datasetFile, demo_datasetFile],
                              paramFile=datasetParam,
                              initialBounds=initialBounds,
                              initialBounds_stdDevMultiples=initialBounds_stdDevMultiples,
                              initialParamValue=initialParamValue,
                              minParamDiff=0.4)
    return settings

def generateSettings_no_update(
        shapeType,
        init_datasetDirectory='./src/nao_writing/nao_writing/letter_model_datasets/alexis_set_for_children',
):
    paramsToVary = [3]
    initialBounds_stdDevMultiples = np.array([[-6, 6]])
    doGroupwiseComparison = True
    initialParamValue = np.NaN
    initialBounds = np.array([[np.NaN, np.NaN]])
    init_datasetFile = init_datasetDirectory + '/' + shapeType + '.dat'

    cwd = os.getcwd()
    print(cwd)

    if not os.path.exists(init_datasetFile):
        raise RuntimeError("Dataset not found for shape " + shapeType)

    try:
        datasetParam = init_datasetDirectory + '/params.dat'
        with open(datasetParam, 'r') as f:
            line = f.readline()
            test = line.replace('[', '').replace(']\n', '') == shapeType
            while test == False:
                line = f.readline()
                if line:
                    test = line.replace('[', '').replace(']\n', '') == shapeType
                else:
                    break
            if test:
                u = f.readline().replace('\n', '')
                initialParamValue = [(float)(s) for s in u.split(',')]
            else:
                initialParamValue = [0.0, 0.0, 0.0, 0.0, 0.0]
                print("parameters not found for shape " + shapeType + '\n' + 'Default : 0.0')

    except IOError:
        raise RuntimeError("no reading permission for file" + datasetParam)

    settings = SettingsStruct(shape_learning=shapeType,
                              paramsToVary=paramsToVary,
                              doGroupwiseComparison=True,
                              initDatasetFile=init_datasetFile,
                              updateDatasetFiles=['', ''],
                              paramFile=datasetParam,
                              initialBounds=initialBounds,
                              initialBounds_stdDevMultiples=initialBounds_stdDevMultiples,
                              initialParamValue=initialParamValue,
                              minParamDiff=0.4)
    return settings



# input_interpreter node
class LearningWord(Node):

    def __init__(self):
        super().__init__('learning_word')
        self.subscription = self.create_subscription(
            Strokes,
            'strokesMessage',
            self.strokes_message_callback, 10)
        self.srv = self.create_service(LearnShape, 'learn_shape', self.learn_shape_callback)
        self.srv = self.create_service(GetDemo, 'get_demo', self.get_demo_callback)

    def strokes_message_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.shape_type)
        self.show_shape(msg)


    def get_word_manager(self, word_to_learn, fn=generateSettings):
        wordManager = ShapeLearnerManager(fn)
        wordSeenBefore = wordManager.newCollection(word_to_learn)  # add word to ShapeLearnerManager -> boolean if
        # the collection has been seen before
        return wordManager

    # the shape_learner_manager is capable of processing multiple shapes in sequence
    # however, here we only process one shape (sequence of only one shape) for simplicity
    def learn_shape_callback(self, request, response):
        self.get_logger().info(f'LearnShape gets request: {request.strokes.shape_type}')

        strokes = request.strokes

        path = self.strokes_to_path(strokes)

        word_to_learn = strokes.shape_type
        wordManager = self.get_word_manager(word_to_learn)
        # the collection has been seen before

        for i in range(len(word_to_learn)):
            ref_shape = wordManager.startNextShapeLearner()  # shape used as reference, each time look for next char
            # and corresponding shape reference
            shape_record_type = wordManager.respondToDemonstration(i, path)  # return a Shape, build with RecordType
            shape_path_normalised = ShapeModeler.normaliseShape(shape_record_type.path)

        response.shape = self.create_shape_message(strokes, shape_path_normalised)
        self.get_logger().info(f'Incoming request: {request.strokes.shape_type}')
        return response

    def get_demo_callback(self, request, response):
        self.get_logger().info('getting demo words.....')
        wordManager = self.get_word_manager(request.word, fn=generateSettings_no_update)

        shape_lst = []

        for i in range(len(request.word)):
            ref_shape = wordManager.startNextShapeLearner()  # shape used as reference, each time look for next char
            shape = Shape()
            print(ref_shape.path, type(ref_shape.path))
            shape.path = ref_shape.path.flatten().tolist()
            shape_lst.append(shape)
        shapes = Shapes()
        shapes.shapes = shape_lst
        shapes.word_to_learn = request.word
        response.shapes = shapes

        self.get_logger().info(f'Incoming request for demo: {request.word}')

        return response

    def create_shape_message(self, strokes, shape_path_normalised):
        shape = Shape()
        shape.path = shape_path_normalised.flatten().tolist()
        if strokes.shape_id is not None:
            shape.shape_id = strokes.shape_id
        if strokes.shape_type is not None:
            shape.shape_type = strokes.shape_type
        if strokes.shapetype_code is not None:
            shape.shapetype_code = strokes.shapetype_code
        if strokes.params_to_vary is not None:
            shape.params_to_vary = strokes.params_to_vary
        if strokes.param_values is not None:
            shape.param_values = strokes.param_values
        return shape

    def strokes_to_path(self, strokes, method='merge'):
        # preprocess to turn multiple strokes into one path
        if method == 'merge':
            path = processShape_mergeStrokes(strokes)
        elif method == 'longestStroke':
            path = processShape_longestStroke(strokes)
        else:
            path = processShape_firstStroke(strokes)

        path = downsampleShape(path, xyxyFormat=False)
        path = np.reshape(path, (-1, 1))
        return path

    def show_shape(self, strokes):
        path = self.strokes_to_path(strokes)

        word_to_learn = strokes.shape_type
        wordManager = ShapeLearnerManager(generateSettings)
        wordSeenBefore = wordManager.newCollection(word_to_learn)  # add word to ShapeLearnerManager

        fig, axs = plt.subplots(len(word_to_learn), 1, figsize=(6, 6 * len(word_to_learn)))
        # Ensure axs is always a list
        if len(word_to_learn) == 1:
            axs = [axs]

        for i in range(len(word_to_learn)):
            ref_shape = wordManager.startNextShapeLearner()  # shape used as reference, each time look for next char
            # and corresponding shape reference
            shape = wordManager.respondToDemonstration(i, path)  # return a Shape()

            shape = ShapeModeler.normaliseShape(shape.path)
            numPointsInShape = int(len(shape) / 2)
            x_shape = shape[0:numPointsInShape]
            y_shape = shape[numPointsInShape:]

            axs[i].plot(x_shape, -y_shape, c=np.random.rand(3))
            axs[i].axis([-1, 1, -1, 1])
            axs[i].set_title(f'Word {i + 1}')

        plt.show()

        # further logic to be done
        # here I only plot it with plt for visualization
        # plot_path(path, self.strokes.shape_type)


def processShape_mergeStrokes(strokes):
    x_pts = []
    y_pts = []
    for s in strokes.strokes:
        stroke = np.array(s.stroke)
        x_pts.extend(stroke[0::2])
        y_pts.extend(stroke[1::2])

    return np.array(x_pts + y_pts)


def processShape_firstStroke(strokes):
    return strokes[0]


def processShape_longestStroke(strokes):
    length_longestStroke = 0
    for stroke in strokes:
        strokeLength = len(stroke.stroke)
        if strokeLength > length_longestStroke:
            longestStroke = stroke
            length_longestStroke = strokeLength
    return longestStroke


def downsampleShape(shape, numDesiredPoints=70, xyxyFormat=False, numPoints_shapeModeler=70):
    numPointsInShape = int(len(shape) / 2)
    if (xyxyFormat):
        # make xyxy format
        x_shape = shape[::2]
        y_shape = shape[1::2]
    else:
        x_shape = shape[0:numPointsInShape]
        y_shape = shape[numPointsInShape:]

    # if isinstance(x_shape,np.ndarray): #convert arrays to lists for interp1d
    #     x_shape = (x_shape.T).tolist()[0]
    #     y_shape = (y_shape.T).tolist()[0]

    # make shape have the same number of points as the shape_modeler
    t_current = np.linspace(0, 1, numPointsInShape)
    t_desired = np.linspace(0, 1, numDesiredPoints)
    f = interpolate.interp1d(t_current, x_shape, kind='cubic')
    x_shape = f(t_desired)
    f = interpolate.interp1d(t_current, y_shape, kind='cubic')
    y_shape = f(t_desired)

    shape = []
    shape[0:numPoints_shapeModeler] = x_shape
    shape[numPoints_shapeModeler:] = y_shape

    return shape



def main(args=None):
    rclpy.init(args=args)

    learning_word = LearningWord()

    rclpy.spin(learning_word)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    learning_word.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
