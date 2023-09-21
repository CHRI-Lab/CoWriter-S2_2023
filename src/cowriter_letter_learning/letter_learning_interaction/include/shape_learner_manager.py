#!/usr/bin/env python3

"""
Manages a collection of shape_learners, with long-term memory about the 
history of previous collections seen. An example is managing
shape_learners which represent letters, and the collections represent
words. 
"""

import logging
import sys
import os.path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../include'))
from shape_learner import ShapeLearner
# for mutable namedtuple (dict might also work)
#from recordtype import recordtype

shape_logger = logging.getLogger("shape_logger")

bound_expanding_amount = 0.
use_prev_params_when_shape_reappears = True

class Shape:
    """
    Shape class to replace recordtype below, which was not functioning
    properly when running rostest for test_tablet_input_interpreter.
    """
    def __init__(self, path=None, shape_id=None, shape_type=None,
                 shape_type_code=None, params_to_vary=None,
                 param_values=None):
        self.path = path
        self.shape_id = shape_id
        self.shape_type = shape_type
        self.shape_type_code = shape_type_code
        self.params_to_vary = params_to_vary
        self.param_values = param_values

# Leave commented out in case we need it, commented out import of recordtype as well
#Shape = recordtype('Shape', [('path', None), ('shape_id', None), ('shape_type', None), ('shape_type_code', None),
 #                            ('params_to_vary', None), ('param_values', None)])


def configure_logging(path):
    """
    Configures logging for the application.

    Parameters:
        path (str): The path to the log file or directory where the log file will be created.

    Returns:
        None

    Raises:
        None

    Description:
        This function configures the logging settings for the application. It takes a path parameter that specifies the
        location where the log file will be created. If the path is a directory, a default log file named "shapes.log" 
        will be created inside that directory. If the path is empty or None, logging will be disabled.

        The function creates a logging handler using the specified path and sets the log level to DEBUG. It also defines
        a formatter for the log messages, which includes the timestamp, logger name, log level, and log message.

        If the path is empty or None, a NullHandler is used, which effectively disables logging.

        Finally, the logging handler is added to the 'shape_logger' logger object, which is assumed to be defined 
        globally, and the log level of the 'shape_logger' is set to DEBUG.

    Example:
        configure_logging("/var/log/app_logs")  # Configures logging with log file located at "/var/log/app_logs/shapes.log"
        configure_logging("")  # Disables logging
    """


    if path:
        if os.path.isdir(path):
            path = os.path.join(path, "shapes.log")
        handler = logging.FileHandler(path)
        handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
    else:
        handler = logging.NullHandler()

    shape_logger.addHandler(handler)
    shape_logger.setLevel(logging.DEBUG)


# ##--------------------------------------------- WORD LEARNING FUNCTIONS
class ShapeLearnerManager:
    """
    A manager class for shape learners.

    Parameters:
        generate_settings_function (function): A function that generates settings for a given shape type.
        shapes_logging_path (str): The path to the log file for logging shape-related information. Default is "shapes.log".

    Attributes:
        generate_settings (function): The function that generates settings for a given shape type.
        shapes_learnt (list): A list of all the shapes that have been learned.
        shape_learners_all (list): A list of all shape learners.
        shape_learners_current_collection (list): A list of shape learners for the current collection.
        settings_shape_learners_all (list): A list of settings for all shape learners.
        settings_shape_learners_current_collection (list): A list of settings for shape learners in the current collection.
        shape_learners_seen_before_current_collection (list): A list indicating if a shape has been seen before in the current collection.
        current_collection (str): The current collection of shapes being worked on.
        collections_learnt (list): A list of collections that have been learnt.
        next_shape_learner_to_be_started (int): Index of the next shape learner to be started.

    Methods:
        __init__(generate_settings_function, shapes_logging_path):
            Initializes the ShapeLearnerManager object.

        initialise_shape_learners():
            Initializes the shape learners for the current collection.

        start_next_shape_learner():
            Starts the learning process for the next shape learner in the current collection.

        feedback_manager(shape_index_message_for, best_shape_index, no_new_shape):
            Manages the feedback provided to a shape learner.

        respond_to_demonstration(shape_index_message_for, shape):
            Responds to a demonstration by updating the shape learner's model.

        index_of_shape_incurrent_collection(shape_type):
            Returns the index of a shape in the current collection.

        index_of_shape_in_all_shapes_learnt(shape_type):
            Returns the index of a shape in all shapes learnt.

        shape_at_index_incurrent_collection(shape_type_index):
            Returns the shape type at the given index in the current collection.

        shape_at_index_in_all_shapes_learnt(shape_type_index):
            Returns the shape type at the given index in all shapes learnt.

        shapes_of_current_collection():
            Returns a list of Shape objects for the shapes in the current collection.

        new_collection(collection):
            Initializes a new collection of shapes.

        reset_parameter_bounds(shape_type_index):
            Resets the parameter bounds for a shape learner.

        generate_simulated_feedback(shape_type_index, new_shape, new_param_value):
            Generates simulated feedback for a shape learner.

        save_all(shape_index_message_for):
            Saves all information related to the shape learner.

        save_demo(shape_index_message_for):
            Saves demonstration-related information for the shape learner.

        save_params(shape_index_message_for):
            Saves the current parameters of the shape learner.
    """
    def __init__(self, generate_settings_function, shapes_logging_path="shapes.log"):
        """
        Initializes the ShapeLearnerManager object.

        Parameters:
            generate_settings_function (function): A function that generates settings for a given shape type.
            shapes_logging_path (str): The path to the log file for logging shape-related information. Default is "shapes.log".
        """
        configure_logging(shapes_logging_path)
        shape_logger.info("**************** NEW SESSION ***************")

        self.generate_settings = generate_settings_function
        self.shapes_learnt = []
        self.shape_learners_all = []
        self.shape_learners_current_collection = []
        self.settings_shape_learners_all = []
        self.settings_shape_learners_current_collection = []
        self.shape_learners_seen_before_current_collection = []
        self.current_collection = ""
        self.collections_learnt = []
        self.next_shape_learner_to_be_started = 0

    def initialise_shape_learners(self):
        """
        Initializes the shape learners based on the current collection of shapes.
        """
        self.shape_learners_current_collection = []
        self.settings_shape_learners_current_collection = []
        self.shape_learners_seen_before_current_collection = []
        for i in range(len(self.current_collection)):
            shape_type = self.current_collection[i]

            # check if shape has been learnt before
            try:
                shape_type_index = self.shapes_learnt.index(shape_type)
                new_shape = False
            except ValueError:
                new_shape = True
            self.shape_learners_seen_before_current_collection.append(
                not new_shape)
            if new_shape:
                settings = self.generate_settings(shape_type)

                shape_learner = ShapeLearner(settings)
                self.shapes_learnt.append(shape_type)
                self.shape_learners_all.append(shape_learner)
                self.settings_shape_learners_all.append(settings)
                self.shape_learners_current_collection.append(
                    self.shape_learners_all[-1])
                self.settings_shape_learners_current_collection.append(
                    self.settings_shape_learners_all[-1])

            else:
                # use the bounds determined last time
                # type: ignore
                previous_bounds = self.shape_learners_all[shape_type_index].get_parameter_bounds(
                )
                new_initial_bounds = previous_bounds
                # USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
                new_initial_bounds[0, 0] -= bound_expanding_amount
                # USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
                new_initial_bounds[0, 1] += bound_expanding_amount
                self.shape_learners_all[shape_type_index].set_parameter_bounds(
                    new_initial_bounds)  # type: ignore
                self.shape_learners_current_collection.append(
                    self.shape_learners_all[shape_type_index])  # type: ignore
                self.settings_shape_learners_current_collection.append(
                    self.settings_shape_learners_all[shape_type_index])  # type: ignore

    def start_next_shape_learner(self):
        # start learning
        """
        Starts the learning process for the next shape in the current collection.

        Returns:
            shape (Shape): The shape object representing the next shape to be learned.
        
        Raises:
            RuntimeError: If there are no more shape learners to be started.
        """
        if (self.next_shape_learner_to_be_started < len(self.current_collection)):
            shape_type = self.current_collection[self.next_shape_learner_to_be_started]
            shape_type_code = self.next_shape_learner_to_be_started
            shape_index = self.index_of_shape_incurrent_collection(shape_type)

            if use_prev_params_when_shape_reappears \
               and self.shape_learners_seen_before_current_collection[self.next_shape_learner_to_be_started]:  # shape has been seen before
                [path, param_values] = self.shape_learners_current_collection[shape_index].get_learned_shape()
                shape_logger.info("%s: continuing learning. Current params: %s. Path: %s" % (
                    shape_type, param_values.flatten().tolist(), path.flatten().tolist()))
            else:
                [path, param_values] = self.shape_learners_current_collection[shape_index].start_learning()
                shape_logger.info("%s: starting learning. Initial params: %s. Path: %s" % (
                    shape_type, param_values.flatten().tolist(), path.flatten().tolist()))

            params_to_vary = self.settings_shape_learners_current_collection[
                shape_index].params_to_vary
            self.next_shape_learner_to_be_started += 1
            shape = Shape(path=path, shape_id=0, shape_type=shape_type,
                          shape_type_code=shape_type_code, params_to_vary=params_to_vary, param_values=param_values)
            return shape
        else:
            raise RuntimeError(
                'Don\'t know what shape learner you want me to start...')

    def feedback_manager(self, shape_index_message_for, best_shape_index, no_new_shape):
        """
        Manages the feedback for a shape learner and generates a new shape if required.

        Args:
            shape_index_message_for (int): The index of the shape for which the feedback is intended.
            best_shape_index (int): The index of the best shape based on the feedback.
            no_new_shape (bool): Flag indicating whether to generate a new shape or only respond to feedback.

        Returns:
            num_iters_converged (int) or -1: The number of iterations converged if a new shape is generated, or -1 if no shape is generated.
            shape (Shape) or -1: The newly generated shape if `no_new_shape` is False, or -1 if no shape is generated.
        """
        shape_message_for = self.shape_at_index_incurrent_collection(
            shape_index_message_for)
        if (shape_message_for == -1):
            shape_logger.warning(
                'Ignoring message because not for valid shape type')
            return -1
        else:

            if no_new_shape:  # just respond to feedback, don't make new shape
                self.shape_learners_current_collection[shape_index_message_for].respond_to_feedback(
                    best_shape_index)
                return 1
            else:
                [num_iters_converged, new_path, new_param_values] = self.shape_learners_current_collection[
                    shape_index_message_for].generate_new_shape_given_feedback(best_shape_index)
            params_to_vary = self.settings_shape_learners_current_collection[
                shape_index_message_for].params_to_vary
            shape = Shape(path=new_path, shape_id=[], shape_type=shape_message_for,
                          shape_type_code=shape_index_message_for, params_to_vary=params_to_vary, param_values=new_param_values)
            return num_iters_converged, shape

    def respond_to_demonstration(self, shape_index_message_for, shape):
        """
        Responds to a demonstration provided for a shape learner and generates a new model.

        Args:
            shape_index_message_for (int): The index of the shape for which the demonstration is intended.
            shape (Shape): The shape object
        Returns:
        shape (Shape) or -1: The newly generated shape if the demonstration is valid, or -1 if the demonstration is invalid.
        """
        shape_message_for = self.shape_at_index_in_all_shapes_learnt(
            shape_index_message_for)
        if (type(shape_message_for) == int):
            shape_logger.warning(
                'Ignoring demonstration because not for valid shape type')
            return -1
        else:
            new_path, new_param_values, params_demo = self.shape_learners_current_collection[
                shape_index_message_for].respond_to_demonstration(shape)

            shape_logger.info("%s: new demonstration.         Params: %s. Path: %s" % (
                shape_message_for, params_demo.flatten().tolist(), shape.flatten().tolist()))

            params_to_vary = self.settings_shape_learners_current_collection[
                shape_index_message_for].params_to_vary
            shape = Shape(path=new_path,
                          shape_id=[],
                          shape_type=shape_message_for,
                          shape_type_code=shape_index_message_for,
                          params_to_vary=params_to_vary,
                          param_values=new_param_values)
            shape_logger.info("%s: new generated model.       Params: %s. Path: %s" % (
                shape_message_for, new_param_values.flatten().tolist(), new_path.flatten().tolist()))
            return shape

    def index_of_shape_incurrent_collection(self, shape_type):
        try:
            shape_type_index = self.current_collection.index(shape_type)
        except ValueError:  # unknown shape
            shape_type_index = -1
        return shape_type_index

    def index_of_shape_in_all_shapes_learnt(self, shape_type):
        try:
            shape_type_index = self.shapes_learnt.index(shape_type)
        except ValueError:  # unknown shape
            shape_type_index = -1
        return shape_type_index

    def shape_at_index_incurrent_collection(self, shape_type_index):
        try:
            shape_type = self.current_collection[shape_type_index]
        except ValueError:  # unknown shape
            shape_type = -1
        return shape_type

    def shape_at_index_in_all_shapes_learnt(self, shape_type_index):
        try:
            shape_type = self.shapes_learnt[shape_type_index]
        except ValueError:  # unknown shape
            shape_type = -1
        return shape_type

    def shapes_of_current_collection(self):
        """
        Get the learned shapes for the shapes in the current collection.

        Returns:
            list: A list of Shape objects representing the learned shapes in the current collection.
        """

        shapes = []

        for idx, shape_learner in enumerate(self.shape_learners_current_collection):

            path, param_values = shape_learner.get_learned_shape()
            params_to_vary = shape_learner.params_to_vary
            shape_name = self.shape_at_index_incurrent_collection(idx)
            code = self.index_of_shape_in_all_shapes_learnt(shape_name)

            shape = Shape(path=path,
                          shape_id=[],
                          shape_type=shape_name,
                          shape_type_code=code,
                          params_to_vary=params_to_vary,
                          param_values=param_values)

            shapes.append(shape)

        return shapes

    def new_collection(self, collection):
        """
        Set a new collection of shapes for the shape learning process.

        Args:
            collection (str): The new collection of shapes to work on.

        Returns:
            bool: True if the collection has been seen before, False otherwise.
        """
        self.current_collection = ""
        # check, for each letter, that we have the corresponding dataset
        for l in collection:
            try:
                self.generate_settings(l)
            except RuntimeError:
                # no dataset for this letter!
                shape_logger.error(
                    "No dataset available for letter <%s>. Skipping this letter." % l)
                continue

            self.current_collection += l

        self.next_shape_learner_to_be_started = 0

        shape_logger.info("Starting to work on word <%s>" % collection)

        try:
            self.collections_learnt.index(self.current_collection)
            collection_seen_before = True
        except ValueError:
            collection_seen_before = False
            self.collections_learnt.append(self.current_collection)

        self.initialise_shape_learners()

        return collection_seen_before

    def reset_parameter_bounds(self, shape_type_index):
        """
        Reset the parameter bounds for a specific shape type.

        Args:
            shape_type_index (int): The index of the shape type in the current collection.

        Returns:
            None
        """
        current_bounds = self.shape_learners_current_collection[shape_type_index].get_parameter_bounds(
        )

        # change bounds back to the initial ones
        new_bounds = self.shape_learners_current_collection[shape_type_index].initial_bounds
        self.shape_learners_current_collection[shape_type_index].set_parameter_bounds(
            new_bounds)
        shape_logger.debug('Changing bounds on shape ' + str(self.shape_at_index_incurrent_collection(shape_type_index)) + ' from ' + str(
            current_bounds) + ' to ' + str(new_bounds))

    def generate_simulated_feedback(self, shape_type_index, new_shape, new_param_value):
        return self.shape_learners_current_collection[shape_type_index].generate_simulated_feedback(new_shape, new_param_value)

    def save_all(self, shape_index_message_for):
        shape_message_for = self.shape_at_index_in_all_shapes_learnt(
            shape_index_message_for)
        if (shape_message_for == -1):
            shape_logger.warning(
                'Ignoring demonstration because not for valid shape type')
            return -1
        else:
            self.shape_learners_current_collection[shape_index_message_for].save_all(
            )

    def save_demo(self, shape_index_message_for):
        shape_message_for = self.shape_at_index_in_all_shapes_learnt(
            shape_index_message_for)
        if (shape_message_for < 0):
            shape_logger.warning(
                'Ignoring demonstration because not for valid shape type')
            return -1
        else:
            self.shape_learners_current_collection[shape_index_message_for].save_demo(
            )

    def save_params(self, shape_index_message_for):
        shape_message_for = self.shape_at_index_in_all_shapes_learnt(
            shape_index_message_for)
        if (shape_message_for < 0):
            shape_logger.warning(
                'Ignoring demonstration because not for valid shape type')
            return -1
        else:
            self.shape_learners_current_collection[shape_index_message_for].save_params(
            )
