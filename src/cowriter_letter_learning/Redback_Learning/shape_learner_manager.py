"""
Manages a collection of shape_learners, with long-term memory about the
history of previous collections seen. An example is managing shape_learners
which represent letters, and the collections represent words.
"""

import logging

import numpy as np

from .shape_modeler import ShapeModeler
from .shape_learner import ShapeLearner, SettingsStruct

shapeLogger = logging.getLogger("shape_logger")
import os.path


from recordtype import recordtype  # for mutable namedtuple (dict might also work)

from scipy import interpolate

ALPHABET = 'abcdefghijklmnopqrstuvwxyz'

boundExpandingAmount = 0
usePrevParamsWhenShapeReappears = True

Shape = recordtype('Shape', [('path', None), ('shapeID', None), ('shapeType', None), ('shapeType_code', None),
                             ('paramsToVary', None), ('paramValues', None)])


def configure_logging(path):
    if path:
        if os.path.isdir(path):
            path = os.path.join(path, "shapes.log")
        handler = logging.FileHandler(path)
        handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
    else:
        handler = logging.NullHandler()

    shapeLogger.addHandler(handler)
    shapeLogger.setLevel(logging.DEBUG)


# ##--------------------------------------------- WORD LEARNING FUNCTIONS
class ShapeLearnerManager:
    def __init__(self, generateSettingsFunction, shapes_logging_path="shapes.log"):

        configure_logging(shapes_logging_path)
        shapeLogger.info("**************** NEW SESSION ***************")

        self.generateSettings = generateSettingsFunction
        self.shapesLearnt = []  # learned chars
        self.shapeLearners_all = []
        self.shapeLearners_currentCollection = []
        self.settings_shapeLearners_all = []
        self.settings_shapeLearners_currentCollection = []
        self.shapeLearnersSeenBefore_currentCollection = []
        self.currentCollection = ""
        self.collectionsLearnt = []
        self.nextShapeLearnerToBeStarted = 0

    def initialiseShapeLearners(self):
        self.shapeLearners_currentCollection = []
        self.settings_shapeLearners_currentCollection = []
        self.shapeLearnersSeenBefore_currentCollection = []
        for i in range(len(self.currentCollection)):
            shapeType = self.currentCollection[i]

            # check if shape has been learnt before
            try:
                shapeType_index = self.shapesLearnt.index(shapeType)
                newShape = False
            except ValueError:
                newShape = True
            self.shapeLearnersSeenBefore_currentCollection.append(not newShape)
            if (newShape):
                settings = self.generateSettings(shapeType)

                shapeLearner = ShapeLearner(settings)
                self.shapesLearnt.append(shapeType)
                self.shapeLearners_all.append(shapeLearner)
                self.settings_shapeLearners_all.append(settings)
                self.shapeLearners_currentCollection.append(self.shapeLearners_all[-1])
                self.settings_shapeLearners_currentCollection.append(self.settings_shapeLearners_all[-1])

            else:
                # use the bounds determined last time
                previousBounds = self.shapeLearners_all[shapeType_index].getParameterBounds()
                newInitialBounds = previousBounds
                newInitialBounds[0, 0] -= boundExpandingAmount;  # USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
                newInitialBounds[0, 1] += boundExpandingAmount;  # USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
                self.shapeLearners_all[shapeType_index].setParameterBounds(newInitialBounds)
                self.shapeLearners_currentCollection.append(self.shapeLearners_all[shapeType_index])
                self.settings_shapeLearners_currentCollection.append(self.settings_shapeLearners_all[shapeType_index])

    def startNextShapeLearner(self):
        # start learning
        if (self.nextShapeLearnerToBeStarted < len(self.currentCollection)):
            shapeType = self.currentCollection[self.nextShapeLearnerToBeStarted]
            shapeType_code = self.nextShapeLearnerToBeStarted
            shape_index = self.indexOfShapeInCurrentCollection(shapeType)

            if usePrevParamsWhenShapeReappears \
                    and self.shapeLearnersSeenBefore_currentCollection[
                self.nextShapeLearnerToBeStarted]:  # shape has been seen before
                [path, paramValues] = self.shapeLearners_currentCollection[shape_index].getLearnedShape()
                shapeLogger.info("%s: continuing learning. Current params: %s. Path: %s" % (
                    shapeType, paramValues.flatten().tolist(), path.flatten().tolist()))
            else:
                [path, paramValues] = self.shapeLearners_currentCollection[shape_index].startLearning()
                shapeLogger.info("%s: starting learning. Initial params: %s. Path: %s" % (
                    shapeType, paramValues.flatten().tolist(), path.flatten().tolist()))

            paramsToVary = self.settings_shapeLearners_currentCollection[shape_index].paramsToVary
            self.nextShapeLearnerToBeStarted += 1
            shape = Shape(path=path, shapeID=0, shapeType=shapeType,
                          shapeType_code=shapeType_code, paramsToVary=paramsToVary, paramValues=paramValues)
            return shape
        else:
            raise RuntimeError('Don\'t know what shape learner you want me to start...')

    def feedbackManager(self, shapeIndex_messageFor, bestShape_index, noNewShape):
        shape_messageFor = self.shapeAtIndexInCurrentCollection(shapeIndex_messageFor)
        if (shape_messageFor not in ALPHABET):
            shapeLogger.warning('Ignoring message because not for valid shape type')
            return -1
        else:
            if (noNewShape):  # just respond to feedback, don't make new shape
                self.shapeLearners_currentCollection[shapeIndex_messageFor].respondToFeedback(bestShape_index)
                return 1
            else:
                [numItersConverged, newPath, newParamValues] = self.shapeLearners_currentCollection[
                    shapeIndex_messageFor].generateNewShapeGivenFeedback(bestShape_index)
            paramsToVary = self.settings_shapeLearners_currentCollection[shapeIndex_messageFor].paramsToVary
            shape = Shape(path=newPath, shapeID=[], shapeType=shape_messageFor,
                          shapeType_code=shapeIndex_messageFor, paramsToVary=paramsToVary, paramValues=newParamValues)
            return numItersConverged, shape

    def respondToDemonstration(self, shapeIndex_messageFor, shape):
        shape_messageFor = self.shapeAtIndexInAllShapesLearnt(shapeIndex_messageFor)
        if (shape_messageFor not in ALPHABET):
            shapeLogger.warning('Ignoring demonstration because not for valid shape type')
            return -1
        else:
            newPath, newParamValues, params_demo = self.shapeLearners_currentCollection[
                shapeIndex_messageFor].respondToDemonstration(shape)

            shapeLogger.info("%s: new demonstration.         Params: %s. Path: %s" % (
                shape_messageFor, params_demo.flatten().tolist(), shape.flatten().tolist()))

            paramsToVary = self.settings_shapeLearners_currentCollection[shapeIndex_messageFor].paramsToVary
            shape = Shape(path=newPath,
                          shapeID=[],
                          shapeType=shape_messageFor,
                          shapeType_code=shapeIndex_messageFor,
                          paramsToVary=paramsToVary,
                          paramValues=newParamValues)
            shapeLogger.info("%s: new generated model.       Params: %s. Path: %s" % (
                shape_messageFor, newParamValues.flatten().tolist(), newPath.flatten().tolist()))
            return shape

    def indexOfShapeInCurrentCollection(self, shapeType):
        try:
            shapeType_index = self.currentCollection.index(shapeType)
        except ValueError:  # unknown shape
            shapeType_index = -1
        return shapeType_index

    def indexOfShapeInAllShapesLearnt(self, shapeType):
        try:
            shapeType_index = self.shapesLearnt.index(shapeType)
        except ValueError:  # unknown shape
            shapeType_index = -1
        return shapeType_index

    def shapeAtIndexInCurrentCollection(self, shapeType_index):
        try:
            shapeType = self.currentCollection[shapeType_index]
        except ValueError:  # unknown shape
            shapeType = -1
        return shapeType

    def shapeAtIndexInAllShapesLearnt(self, shapeType_index):
        try:
            shapeType = self.shapesLearnt[shapeType_index]
        except ValueError:  # unknown shape
            shapeType = -1
        return shapeType

    def shapesOfCurrentCollection(self):

        shapes = []

        for idx, shape_learner in enumerate(self.shapeLearners_currentCollection):
            path, paramValues = shape_learner.getLearnedShape()
            paramsToVary = shape_learner.paramsToVary
            shapeName = self.shapeAtIndexInCurrentCollection(idx)
            code = self.indexOfShapeInAllShapesLearnt(shapeName)

            shape = Shape(path=path,
                          shapeID=[],
                          shapeType=shapeName,
                          shapeType_code=code,
                          paramsToVary=paramsToVary,
                          paramValues=paramValues)

            shapes.append(shape)

        return shapes

    def newCollection(self, collection):
        print(f'get collection: {collection}')
        self.currentCollection = ""
        # check, for each letter, that we have the corresponding dataset
        for l in collection:
            try:
                self.generateSettings(l)
            except RuntimeError:
                # no dataset for this letter!
                shapeLogger.error("No dataset available for letter <%s>. Skipping this letter." % l)
                continue

            self.currentCollection += l

        self.nextShapeLearnerToBeStarted = 0

        shapeLogger.info("Starting to work on word <%s>" % collection)

        try:
            self.collectionsLearnt.index(self.currentCollection)
            collectionSeenBefore = True
        except ValueError:
            collectionSeenBefore = False
            self.collectionsLearnt.append(self.currentCollection)

        self.initialiseShapeLearners()

        return collectionSeenBefore

    def resetParameterBounds(self, shapeType_index):
        currentBounds = self.shapeLearners_currentCollection[shapeType_index].getParameterBounds()

        # change bounds back to the initial ones
        newBounds = self.shapeLearners_currentCollection[shapeType_index].initialBounds
        self.shapeLearners_currentCollection[shapeType_index].setParameterBounds(newBounds)
        shapeLogger.debug(
            'Changing bounds on shape ' + self.shapeAtIndexInCurrentCollection(shapeType_index) + ' from ' + str(
                currentBounds) + ' to ' + str(newBounds))

    def generateSimulatedFeedback(self, shapeType_index, newShape, newParamValue):
        return self.shapeLearners_currentCollection[shapeType_index].generateSimulatedFeedback(newShape, newParamValue)

    def save_all(self, shapeIndex_messageFor):
        shape_messageFor = self.shapeAtIndexInAllShapesLearnt(shapeIndex_messageFor)
        if (shape_messageFor not in ALPHABET):
            shapeLogger.warning('Ignoring demonstration because not for valid shape type')
            return -1
        else:
            self.shapeLearners_currentCollection[shapeIndex_messageFor].save_all()

    def save_demo(self, shapeIndex_messageFor):
        shape_messageFor = self.shapeAtIndexInAllShapesLearnt(shapeIndex_messageFor)
        if (shape_messageFor not in ALPHABET):
            shapeLogger.warning('Ignoring demonstration because not for valid shape type')
            return -1
        else:
            self.shapeLearners_currentCollection[shapeIndex_messageFor].save_demo()

    def save_params(self, shapeIndex_messageFor):
        shape_messageFor = self.shapeAtIndexInAllShapesLearnt(shapeIndex_messageFor)
        if (shape_messageFor < 0):
            shapeLogger.warning('Ignoring demonstration because not for valid shape type')
            return -1
        else:
            self.shapeLearners_currentCollection[shapeIndex_messageFor].save_params()


def generateSettings(
        shapeType,
        init_datasetDirectory='./letter_model_datasets/alexis_set_for_children',
        update_datasetDirectory='./letter_model_datasets/alexis_set_for_children',
        demo_datasetDirectory='./letter_model_datasets/alexis_set_for_children',
):
    paramsToVary = [3]
    initialBounds_stdDevMultiples = np.array([[-6, 6]])
    doGroupwiseComparison = True
    initialParamValue = np.NaN
    initialBounds = np.array([[np.NaN, np.NaN]])
    init_datasetFile = init_datasetDirectory + '/' + shapeType + '.dat'
    update_datasetFile = update_datasetDirectory + '/' + shapeType + '.dat'
    demo_datasetFile = demo_datasetDirectory + '/' + shapeType + '.dat'

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


def downsampleShape(shape,numDesiredPoints=70,xyxyFormat=False):
    numPointsInShape = int(len(shape)/2)
    if(xyxyFormat):
        #make xyxy format
        x_shape = shape[0::2]
        y_shape = shape[1::2]
    else:
        x_shape = shape[0:numPointsInShape]
        y_shape = shape[numPointsInShape:]

    if isinstance(x_shape,np.ndarray): #convert arrays to lists for interp1d
        x_shape = (x_shape.T).tolist()[0]
        y_shape = (y_shape.T).tolist()[0]

    #make shape have the same number of points as the shape_modeler
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


def showShape(shape, shapeIndex):
    shape = ShapeModeler.normaliseShape(shape.path)

    numPointsInShape = int(len(shape) / 2)
    x_shape = shape[0:numPointsInShape]
    y_shape = shape[numPointsInShape:]

    # plt.plot(x_shape, -y_shape, c=np.random.rand(3, 1))
    plt.plot(x_shape, -y_shape, c=np.random.rand(3))
    plt.axis([-1, 1, -1, 1])

    plt.show()

    # plt.figure(shapeIndex+1)
    # plt.clf()
    # ShapeModeler.normaliseAndShowShape(shape.path)


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import matplotlib

    matplotlib.use('TkAgg')  # Use the TkAgg backend instead of the PyCharm backend
    # matplotlib.use('Agg')

    # os.environ["DISPLAY"] = "172.25.96.1:0:"

    numPoints_shapeModeler = 70

    user_input = [1, 2, 2, 3, 4, 6, 8, 9]
    user_input = [564.0, -316.0, 562.0, -320.0, 556.0, -331.0, 548.0, -339.99999999999994, 539.0, -349.0, 532.0,
                  -357.00000000000006, 526.0, -361.0, 507.0, -373.99999999999994, 501.00000000000006,
                  -377.99999999999994, 491.99999999999994, -384.0, 467.0, -393.0, 460.0, -394.0, 448.0, -396.0, 430.0,
                  -397.0, 414.0, -397.0, 405.00000000000006, -397.0, 389.0, -394.0, 382.0, -392.00000000000006, 367.0,
                  -383.0, 364.0, -381.0, 352.0, -371.0, 346.0, -366.0, 332.0, -346.0, 331.0, -341.99999999999994, 326.0,
                  -330.0, 324.0, -325.00000000000006, 321.0, -313.0, 318.0, -299.0, 315.0, -290.99999999999994, 312.0,
                  -273.99999999999994, 311.0, -267.0, 310.0, -255.99999999999997, 310.0, -252.0, 310.0,
                  -245.00000000000003, 311.0, -234.0, 314.0, -225.00000000000003, 317.0, -216.0, 320.0,
                  -212.00000000000003, 327.0, -202.0, 330.0, -199.0, 334.0, -197.0, 338.0, -194.00000000000003, 344.0,
                  -189.99999999999997, 351.0, -186.99999999999997, 357.0, -186.0, 364.0, -185.0, 381.0, -184.0, 389.0,
                  -184.0, 392.0, -184.0, 407.99999999999994, -186.0, 412.00000000000006, -186.99999999999997,
                  426.00000000000006, -193.00000000000003, 441.0, -200.0, 454.00000000000006, -209.00000000000003,
                  475.0, -222.99999999999997, 479.0, -226.00000000000003, 493.0, -238.99999999999997,
                  498.99999999999994, -248.0, 503.0, -253.99999999999997, 507.0, -261.0, 509.0, -264.0, 514.0, -276.0,
                  514.0, -279.0, 517.0, -286.0, 519.0, -290.99999999999994, 520.0, -293.0, 523.0, -300.0, 523.0, -303.0,
                  525.0, -309.99999999999994, 525.0, -313.0, 525.0, -314.0, 525.0, -316.0, 526.0, -317.0, 526.0, -320.0,
                  527.0, -323.00000000000006, 527.0, -325.00000000000006, 528.0, -325.00000000000006, 528.0, -326.0,
                  528.0, -327.0, 529.0, -327.0, 529.0, -329.0, 529.0, -328.0, 527.0, -323.00000000000006, 525.0,
                  -321.00000000000006, 523.0, -316.0, 521.0, -312.0, 520.0, -307.99999999999994, 516.0, -300.0, 514.0,
                  -292.00000000000006, 511.0, -284.0, 509.0, -277.0, 508.00000000000006, -268.0, 507.0, -262.0, 507.0,
                  -259.00000000000006, 507.0, -253.99999999999997, 507.0, -248.0, 507.0, -237.99999999999997,
                  508.00000000000006, -235.0, 509.0, -231.0, 511.0, -225.00000000000003, 512.0, -221.99999999999997,
                  518.0, -214.0, 518.0, -213.0, 523.0, -205.99999999999997, 526.0, -203.0, 528.0, -200.0, 531.0, -197.0,
                  535.0, -194.00000000000003, 536.0, -194.00000000000003]

    user_input = downsampleShape(user_input, numPoints_shapeModeler, xyxyFormat=True)
    user_input = np.reshape(user_input, (-1, 1));
    word_to_learn = 'a'

    # generateSettings('a')

    wordManager = ShapeLearnerManager(generateSettings)
    wordSeenBefore = wordManager.newCollection(word_to_learn)

    # plt.ion()
    # plt.figure()

    for i in range(len(word_to_learn)):
        shape = wordManager.startNextShapeLearner()
        # showShape(shape, i)

    print(user_input)
    # user_input = np.array([[466.],
    #                        [454.25433802],
    #                        [441.25271647],
    #                        [421.36911743],
    #                        [411.54339729],
    #                        [394.95526649],
    #                        [374.84761533],
    #                        [357.09148236],
    #                        [338.88664694],
    #                        [320.08446158],
    #                        [299.90720144],
    #                        [275.99171656],
    #                        [264.39988714],
    #                        [253.68349396],
    #                        [242.38014011],
    #                        [232.46533914],
    #                        [226.92132872],
    #                        [223.71466511],
    #                        [221.85408745],
    #                        [221.87563138],
    #                        [223.92829805],
    #                        [228.7688539],
    #                        [232.93118678],
    #                        [236.68269516],
    #                        [244.21173067],
    #                        [253.19185481],
    #                        [262.32775092],
    #                        [268.67099717],
    #                        [279.62408421],
    #                        [291.37723998],
    #                        [305.01603822],
    #                        [332.87144466],
    #                        [358.97327],
    #                        [412.25960821],
    #                        [428.65361097],
    #                        [440.35292235],
    #                        [448.33952148],
    #                        [449.74935359],
    #                        [457.99002299],
    #                        [466.50098172],
    #                        [471.16892902],
    #                        [476.30418786],
    #                        [482.30170709],
    #                        [484.07078544],
    #                        [486.22374433],
    #                        [487.94027283],
    #                        [490.52315284],
    #                        [490.92869916],
    #                        [492.04882903],
    #                        [491.97115159],
    #                        [492.00224994],
    #                        [491.99893695],
    #                        [492.01334783],
    #                        [491.99183062],
    #                        [491.42609404],
    #                        [489.14356792],
    #                        [487.72859385],
    #                        [484.71569229],
    #                        [485.02178532],
    #                        [485.00509085],
    #                        [484.99796636],
    #                        [485.00027038],
    #                        [485.03168858],
    #                        [484.85037836],
    #                        [487.27760847],
    #                        [489.64013332],
    #                        [495.52606372],
    #                        [497.56384122],
    #                        [500.87268922],
    #                        [502.],
    #                        [-440.],
    #                        [-444.192157],
    #                        [-447.62838941],
    #                        [-449.14115502],
    #                        [-450.07756904],
    #                        [-450.00629859],
    #                        [-450.14824868],
    #                        [-448.149649],
    #                        [-444.96217092],
    #                        [-438.99713083],
    #                        [-432.46842538],
    #                        [-422.20376969],
    #                        [-416.07671158],
    #                        [-408.41067589],
    #                        [-400.46847425],
    #                        [-388.34740383],
    #                        [-377.81594455],
    #                        [-365.12649524],
    #                        [-349.49492893],
    #                        [-337.60519769],
    #                        [-317.17609299],
    #                        [-298.59937859],
    #                        [-286.98435646],
    #                        [-275.78874427],
    #                        [-259.55003389],
    #                        [-246.99000304],
    #                        [-235.60854465],
    #                        [-231.58924323],
    #                        [-225.58311784],
    #                        [-221.09806357],
    #                        [-218.50519896],
    #                        [-218.13153842],
    #                        [-221.78687754],
    #                        [-245.11810761],
    #                        [-255.56622345],
    #                        [-264.27047665],
    #                        [-272.78346454],
    #                        [-276.60623741],
    #                        [-287.03031716],
    #                        [-305.26533777],
    #                        [-318.61684392],
    #                        [-336.71215232],
    #                        [-357.00377704],
    #                        [-363.61407177],
    #                        [-372.44042018],
    #                        [-379.55059114],
    #                        [-395.92585402],
    #                        [-401.16448877],
    #                        [-406.1868903],
    #                        [-411.28380211],
    #                        [-422.07021774],
    #                        [-425.52589394],
    #                        [-428.83023882],
    #                        [-429.93740616],
    #                        [-429.17043259],
    #                        [-419.92327244],
    #                        [-414.09359373],
    #                        [-400.58644588],
    #                        [-386.20057416],
    #                        [-367.68817342],
    #                        [-357.72982207],
    #                        [-346.2006924],
    #                        [-326.55989474],
    #                        [-317.51207081],
    #                        [-301.38269268],
    #                        [-289.12255718],
    #                        [-277.21816308],
    #                        [-270.67032057],
    #                        [-264.88764106],
    #                        [-263.]])

    user_input = np.array([[394.],
     [391.15246789],
     [387.12457817],
     [381.34172407],
     [376.18277563],
     [373.67941513],
     [363.30959002],
     [356.79349524],
     [343.76725757],
     [335.14824273],
     [319.92387532],
     [306.77334847],
     [296.23855419],
     [288.75011483],
     [283.91254408],
     [279.23296132],
     [275.86071183],
     [269.911732],
     [265.31848244],
     [262.05875128],
     [259.21798062],
     [257.34199092],
     [257.02133303],
     [256.99976333],
     [256.98409738],
     [258.2374757],
     [268.90461691],
     [279.28915072],
     [305.15136668],
     [333.47232841],
     [352.47723217],
     [361.9390745],
     [377.30473656],
     [398.28585516],
     [412.5666344],
     [428.66534364],
     [444.63235663],
     [468.18345511],
     [486.24411399],
     [506.35556254],
     [524.26132276],
     [533.80482109],
     [540.37747843],
     [550.05343931],
     [552.55524167],
     [561.46864851],
     [566.38093317],
     [569.59433901],
     [570.91196481],
     [573.2952246],
     [577.20604854],
     [578.04291333],
     [583.88153434],
     [587.79636828],
     [592.04055579],
     [594.52807253],
     [595.99851815],
     [597.48571926],
     [597.98033049],
     [597.99714272],
     [597.9938411],
     [598.91484763],
     [600.7544227],
     [604.59560614],
     [608.88890393],
     [614.47617728],
     [623.13378998],
     [629.73247075],
     [636.29029803],
     [638.],
     [-395.],
     [-394.99999947],
     [-394.99998952],
     [-394.99983319],
     [-394.9977429],
     [-394.97315775],
     [-394.69483656],
     [-393.74995009],
     [-390.95874572],
     [-388.04684952],
     [-382.74893208],
     [-375.70850096],
     [-371.33661008],
     [-366.61792721],
     [-362.96473628],
     [-359.62168831],
     [-355.77454873],
     [-349.9040923],
     [-346.28954094],
     [-341.14980196],
     [-337.53363703],
     [-335.55539057],
     [-332.41954788],
     [-330.32935013],
     [-327.58669035],
     [-324.68509283],
     [-315.05624206],
     [-309.42442577],
     [-305.08839378],
     [-305.00780104],
     [-305.00067352],
     [-304.99972941],
     [-304.99671065],
     [-304.97364561],
     [-306.07345211],
     [-307.00786668],
     [-307.00189219],
     [-307.00023677],
     [-307.00002824],
     [-307.00006338],
     [-307.000683],
     [-307.00642657],
     [-307.04654557],
     [-310.01425664],
     [-310.81608257],
     [-313.72554217],
     [-316.60784281],
     [-319.08305062],
     [-320.36819598],
     [-323.30151541],
     [-327.5005827],
     [-331.083583],
     [-340.80485882],
     [-349.35730693],
     [-360.21140146],
     [-365.77774712],
     [-370.10836593],
     [-373.07874015],
     [-375.82390925],
     [-361.7835644],
     [-353.81951294],
     [-337.59527805],
     [-331.75736556],
     [-322.22042235],
     [-317.18833599],
     [-314.13500974],
     [-311.70080019],
     [-309.92806472],
     [-309.99638821],
     [-310.]])

    shape = wordManager.respondToDemonstration(0, user_input)

    showShape(shape, i)
