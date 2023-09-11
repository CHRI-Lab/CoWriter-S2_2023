#!/usr/bin/env python3

"""
Manages a collection of shape_learners, with long-term memory about the 
history of previous collections seen. An example is managing shape_learners
which represent letters, and the collections represent words. 
"""

import logging
import os.path

from src.packages.shape_learning.src.shape_learning.shape_learner import ShapeLearner
# for mutable namedtuple (dict might also work)
from recordtype import recordtype

shape_logger = logging.getLogger("shape_logger")

bound_expanding_amount = 0.
use_prev_params_when_shape_reappears = True

Shape = recordtype('Shape', [('path', None), ('shape_id', None), ('shape_type', None), ('shape_type_code', None),
                             ('params_to_vary', None), ('param_values', None)])


def configure_logging(path):

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
    def __init__(self, generate_settings_function, shapes_logging_path="shapes.log"):

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
        shape_message_for = self.shape_at_index_in_all_shapes_learnt(
            shape_index_message_for)
        if (shape_message_for < 0):
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

        self.current_collection = ""
        # check, for each letter, that we have the corresponding dataset
        for l in collection:
            try:
                self.generate_settings(l)
            except RuntimeError:
                # no dataset for this letter!
                shape_logger.error(
                    "No dataset available for letter <%s>. Skipping this letter." % l)
                raise RuntimeError('No dataset available for letter')
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
