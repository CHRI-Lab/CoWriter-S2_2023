"""
Class to use feedback to learn optimal parameters for a shape modeled by
an associated ShapeModeler.

Depends on shape_modeler and recordtype.
"""


# for mutable namedtuple (dict might also work)
from recordtype import recordtype
import bisect
from copy import deepcopy

import numpy
from src.packages.shape_learning.src.shape_learning.shape_modeler import ShapeModeler


# shape learning parameters

# Allowed attempts to draw a new shape which is significantly different to the
# previous one but still within the range (just a precaution; sampling should
# be theoretically possible)
MAX_NUM_ATTEMPTS = 10000

# Tolerance on convergence test
TOL = 1e-2

NUM_PRINCIPLE_COMPONENTS = 5


settings_struct = recordtype('settings_struct',
                             ['shape_learning',  # String representing the shape which the object is learning
                              # Path to the dataset file that will be used to initialize the matrix for PCA
                              'init_dataset_file',
                              # List of path -- or single path-- to dataset that will be updated with demo shapes
                              'update_dataset_files',
                              'param_file',  # Path to the dataset file 'params.dat' inside which we save the learned parameters
                              'params_to_vary',
                              # Natural number between 1 and number of parameters in the associated ShapeModeler, representing the parameter to learn
                              # instead of pairwise comparison with most recent two shapes
                              'do_groupwise_comparison',
                              'initial_bounds',
                              # Initial acceptable parameter range (if a value is NaN, the initial_bounds_std_dev_multiples setting will be used to set that value)
                              'initial_bounds_std_dev_multiples',
                              # Initial acceptable parameter range in terms of the standard deviation of the parameter
                              'initial_param_value',
                              # Initial parameter value (NaN if to be drawn uniformly from initial_bounds)
                              'min_param_diff'])  # How different two shapes' parameters need to be to be published for comparison
# @todo: make groupwise comparison/pairwise comparison different implementations of shapeLearner class


class ShapeLearner:
    def __init__(self, settings):

        self.params_to_vary = settings.params_to_vary
        # self.num_principle_components = max(self.params_to_vary)
        self.num_principle_components = NUM_PRINCIPLE_COMPONENTS
        # assign a ShapeModeler to use
        print(settings.init_dataset_file)
        self.shape_modeler = ShapeModeler(init_filename=settings.init_dataset_file,
                                          update_filenames=settings.update_dataset_files,
                                          param_filename=settings.param_file,
                                          num_principle_components=self.num_principle_components)

        self.bounds = settings.initial_bounds
        for i in range(len(self.params_to_vary)):
            parameterVariances = self.shape_modeler.get_parameter_variances()
            if (numpy.isnan(settings.initial_bounds[i, 0]) or numpy.isnan(
                    settings.initial_bounds[i, 1])):  # want to set initial bounds as std. dev. multiple
                bounds_from_std_dev_multiples = numpy.array(settings.initial_bounds_std_dev_multiples[i, :]) * \
                    parameterVariances[settings.params_to_vary[i] - 1]

                if (numpy.isnan(settings.initial_bounds[i, 0])):
                    self.bounds[i, 0] = bounds_from_std_dev_multiples[0]
                if (numpy.isnan(settings.initial_bounds[i, 1])):
                    self.bounds[i, 1] = bounds_from_std_dev_multiples[1]

        self.do_groupwise_comparison = settings.do_groupwise_comparison
        self.shape_learning = settings.shape_learning
        self.min_param_diff = settings.min_param_diff

        self.initial_param_value = settings.initial_param_value
        self.params = numpy.zeros((self.num_principle_components, 1))

        for i in range(self.num_principle_components):
            self.params[i][0] = -self.initial_param_value[i]

        self.initial_bounds = deepcopy(self.bounds)
        self.converged = False
        self.num_iters = 0
        self.num_iters_converged = 0

    # ----------------------------------------------------- START LEARNING
    def start_learning(self):
        # make initial shape
        if (numpy.isnan(self.initial_param_value[0])):
            [shape, paramValues] = self.shape_modeler.make_random_shape_from_uniform(self.params, self.params_to_vary,
                                                                                     self.bounds)
            self.params = paramValues
        else:
            shape = self.shape_modeler.make_shape(self.params)

        self.best_param_values = self.params[
            self.params_to_vary[0] - 1]  # USE ONLY FIRST PARAM IN LIST FOR SELF-LEARNING ALGORITHM

        if (self.do_groupwise_comparison):
            self.params_sorted = [self.bounds[0, 0],
                                  self.bounds[0, 1]]  # USE ONLY FIRST PARAM IN LIST FOR SELF-LEARNING ALGORITHM
            bisect.insort(self.params_sorted, self.best_param_values)
            self.shape_to_params_mapping = [self.params]
        else:
            self.new_param_value = self.best_param_values
            self.params = [self.new_param_value]

        return shape, self.params

    # ---------------------------------------- START LEARNING - TRIANGULAR
    def start_learning_at(self, starting_bounds, starting_param_values):
        self.bounds = starting_bounds

        # make initial shape
        [shape, param_values] = self.shape_modeler.make_random_shape_from_traingular(self.params, self.params_to_vary,
                                                                                     self.bounds, starting_param_values)
        self.params = param_values
        self.best_param_values = param_values[
            self.params_to_vary[0] - 1]  # USE ONLY FIRST PARAM IN LIST FOR SELF-LEARNING ALGORITHM
        if (self.do_groupwise_comparison):
            self.params_sorted = [self.bounds[0, 0],
                                  self.bounds[0, 1]]  # USE ONLY FIRST PARAM IN LIST FOR SELF-LEARNING ALGORITHM
            bisect.insort(self.params_sorted, self.best_param_values)
            self.shape_to_params_mapping = [self.params]
        else:
            self.new_param_value = self.best_param_values

        return shape, self.best_param_values

    # ----------------------------------------------- MAKE DIFFERENT SHAPE

    def make_shape_different_to(self, param_value):
        # make new shape to compare with
        [new_shape, new_params] = self.shape_modeler.make_random_shape_from_traingular(self.params, self.params_to_vary,
                                                                                       self.bounds, [
                                                                                           param_value])  # USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
        # USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
        new_param_values = new_params[self.params_to_vary[0] - 1, 0]
        # ensure it is significantly different
        num_attempts = 1
        while (abs(new_param_values - param_value) < self.min_param_diff and num_attempts < MAX_NUM_ATTEMPTS):
            [new_shape, new_params] = self.shape_modeler.make_random_shape_from_traingular(self.params, self.params_to_vary,
                                                                                           self.bounds, [
                                                                                               param_value])  # USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
            new_param_values = new_params[
                self.params_to_vary[0] - 1, 0]  # USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
            num_attempts += 1

        if (num_attempts >= MAX_NUM_ATTEMPTS):  # couldn't find a 'different' shape in range
            # this should be prevented by the convergence test below
            print('Oh no!')

        # store it as an attempt
        if (self.do_groupwise_comparison):
            bisect.insort(self.params_sorted, new_param_values)
            self.shape_to_params_mapping.append(new_params)

        return new_shape, new_param_values

    # ------------------------------------------------- MAKE SIMILAR SHAPE
    def make_shape_similar_to(self, param_value):
        # make new shape, but don't enforce that it is sufficiently different
        [new_shape, new_param_values] = self.shape_modeler.make_random_shape_from_traingular(self.params, self.params_to_vary,
                                                                                             self.bounds, [
                                                                                                 param_value])  # USE FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
        # USE FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
        new_param_value = new_param_values[self.params_to_vary[0] - 1, 0]

        # store it as an attempt
        if (self.do_groupwise_comparison):
            bisect.insort(self.params_sorted, new_param_value)
            self.shape_to_params_mapping.append(new_param_values)

        return new_shape, new_param_value

    # ---------------------------------------- GENERATE SIMULATED FEEDBACK
    def generate_simulated_feedback(self, shape, new_param_value):
        # code in place of feedback from user: go towards goal parameter value
        # -1.5*parameterVariances[self.paramToVary-1]
        goal_param_value = numpy.float64(0)
        goal_params_value = numpy.zeros((self.num_principle_components, 1))
        goal_params_value[self.params_to_vary - 1, 0] = goal_param_value
        if (self.do_groupwise_comparison):
            errors = numpy.ndarray.tolist(
                abs(self.shape_to_params_mapping - goal_params_value))
            best_shape_idx = errors.index(min(errors))
        else:
            errors = [abs(self.best_param_values - goal_param_value),
                      abs(new_param_value - goal_param_value)]
            best_shape_idx = errors.index(min(errors))
        return best_shape_idx

    # ------------------------------------------------ RESPOND TO FEEDBACK
    def respond_to_feedback(self, best_shape):
        # update bestParamValue based on feedback received
        if (self.do_groupwise_comparison):
            params_best = self.shape_to_params_mapping[best_shape]

            self.best_param_values = params_best[
                self.params_to_vary[0] - 1, 0]  # USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
            best_param_value_index = bisect.bisect(self.params_sorted,
                                                   self.best_param_values) - 1  # indexing seems to start at 1 with bisect
            new_bounds = [self.params_sorted[best_param_value_index - 1],
                          self.params_sorted[best_param_value_index + 1]]

            # restrict bounds if they were caused by other shapes, because it must be sufficiently different to said shape(s)
            if ((best_param_value_index - 1) > 0):  # not the default min
                new_bounds[0] += self.min_param_diff
            if ((best_param_value_index + 1) < (len(self.params_sorted) - 1)):  # not the default max
                new_bounds[1] -= self.min_param_diff

            # protect from bounds switching expected order
            if (not (new_bounds[0] > new_bounds[1])):
                # USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
                self.bounds[0, :] = new_bounds

            diff_params = params_best - self.params
            diff = numpy.linalg.norm(diff_params)
            self.params += diff_params / 2
        else:  # do pairwise comparison with most recent shape and previous
            # restrict limits
            if (best_shape == 'new'):  # new shape is better
                worst_param_value = self.best_param_values
                best_param_value = self.new_param_value
                self.params[self.params_to_vary - 1, 0] = best_param_value
            else:  # new shape is worse
                worst_param_value = self.new_param_value
                best_param_value = self.best_param_values
                self.params[self.params_to_vary - 1, 0] = best_param_value

            # shape with lower value is worse
            if (worst_param_value == min(self.best_param_values, self.new_param_value)):
                # increase min bound to worst so we don't try any lower
                self.bounds[0] = worst_param_value
            else:  # shape with higher value is worse
                # decrease max bound to worst so we don't try any higher
                self.bounds[1] = worst_param_value

            # ------------------------------------------------------------ ITERATE

    def generate_new_shape_given_feedback(self, best_shape):
        # ------------------------------------------- respond to feedback
        # update bounds and bestParamValue
        self.respond_to_feedback(best_shape)

        # ----------------------------------------- check for convergence
        # continue if there are more shapes to try which are different enough
        if ((abs(self.bounds[0, 1] - self.best_param_values) - self.min_param_diff < TOL) and (abs(
            self.best_param_values - self.bounds[
                0, 0]) - self.min_param_diff) < TOL):  # USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
            self.converged = True
        else:
            self.converged = False

        # -------------------------------------------- continue iterating
        self.num_iters += 1

        # try again if shape is not good enough
        if (not self.converged):
            self.num_iters_converged = 0
            [new_shape, new_param_value] = self.make_shape_different_to(
                self.best_param_values)
            self.new_param_value = new_param_value
            return self.num_iters_converged, new_shape, new_param_value

        else:
            self.num_iters_converged += 1
            [new_shape, new_param_value] = self.make_shape_similar_to(
                self.best_param_values)
            self.new_param_value = new_param_value
            return self.num_iters_converged, new_shape, new_param_value

    def get_learned_params(self):
        return self.params

    def get_learned_shape(self):
        return self.shape_modeler.make_shape(self.params), self.params

    def get_parameter_bounds(self):
        return self.bounds

    def set_parameter_bounds(self, bounds):
        self.bounds = bounds

    def respond_to_demonstration(self, shape):
        """
        Algo:

        1) takes the shape of the demonstration

        2) takes the curent learned shape

        3) re-performs PCA taking in account the domonstrated shape,
           then obtains a new space with new eigen vectors

        4) projects demonstrated and learned shapes into this new space 
           and gets their new parameters  

        5) updates the learned parameters as the algebric middle 
           between demonstrated parameters and curent learned parameters. 
        """
        demo_shape = ShapeModeler.normalise_shape_height(numpy.array(shape))

        # take the shape corresponding to the curent learned parameters in the curent space
        learned_shape = self.shape_modeler.make_shape(self.params)

        # add the demo shape to the matrix for PCA and re-compute reference-shape params
        self.shape_modeler.extend_data_mat(demo_shape)
        ref_params = self.shape_modeler.ref_params

        # re-compute parameters of the learned shape and the demo shape in the new PCA-space
        params_demo, _ = self.shape_modeler.decompose_shape(demo_shape)
        self.params, _ = self.shape_modeler.decompose_shape(learned_shape)

        # learning :
        diff_params = params_demo - self.params
        self.params += diff_params/2  # go towards the demonstrated shape

        # self.params[self.params_to_vary[0]-1] = params_demo[self.params_to_vary[0]-1] #ONLY USE FIRST PARAM
        # store it as an attempt (this isn't super appropriate but whatever)
        if (self.do_groupwise_comparison):
            new_param_value = self.params[
                self.params_to_vary[0] - 1, 0]  # USE ONLY FIRST PARAM FOR SELF-LEARNING ALGORITHM ATM
            # print('Demo params: '+str(self.params))
            bisect.insort(self.params_sorted, new_param_value)
            self.shape_to_params_mapping.append(self.params)
            # self.respondToFeedback(len(self.params_sorted)-3) # give feedback of most recent shape so bounds modify
        return self.shape_modeler.make_shape(self.params), self.params, params_demo

    def save_all(self):
        self.shape_modeler.save_all()

    def save_demo(self):
        self.shape_modeler.save_demo()

    def save_params(self):
        param_value = []
        for i in range(self.num_principle_components):
            param_value.append(-self.params[i][0])
        self.shape_modeler.save_params(param_value, self.shape_learning)
