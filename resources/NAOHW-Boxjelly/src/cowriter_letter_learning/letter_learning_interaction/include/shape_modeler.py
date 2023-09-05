""" Class to decompose a dataset of shapes into its principle
components, and to make and show new shapes which are represented by
the mean shape plus some amount of said principle components.
"""

import random
from copy import deepcopy

import numpy
import os.path

from matplotlib import pyplot as plt

from sklearn.cluster import MeanShift


class ShapeModeler:
    def __init__(self,
                 shape_name=None,
                 samples=None,
                 init_filename=None,
                 update_filenames=None,
                 param_filename=None,
                 num_principle_components=10):
        """ Initialize a shape modeler

        If given an initial dataset (params samples or init_filename),
        loads the training dataset for a given shape, and run a PCA
        decomposition on it.

        The update dataset (a file name or a list of file names) are
        used to serialize the shapes from demonstrations. If it's a
        list, the first file will contains the initial shapes + the
        demo shapes, and the followings will contain just the demo
        shapes.

        :param samples: a list of sample shapes, each presented as a
                        list of coordinates [x1,x2,...,y1,y2...].
                        Each shape must have the same number of
                        coordinates.
        :param filename: path to a shape dataset. See makeDataMatrix
                         for the expected format.
        :paran num_principle_components: number of desired principle
                                         components
        """

        self.shape_name = shape_name
        self.num_principle_components = num_principle_components

        if samples is None and init_filename is None:
            
            return

        if samples:
            self.data_mat = numpy.array(samples)

        elif init_filename:
            # if 'uji_pen_chars2' in init_filename:
            #     self.make_data_matrix(init_filename)
            # elif 'uji_pen_subset' in init_filename:
            #     self.make_data_matrix_2(init_filename)
            self.make_data_matrix_2(init_filename)

        if update_filenames:
            self.update_filenames = update_filenames

        if param_filename:
            self.param_filename = param_filename

        if not isinstance(update_filenames, list):
            self.update_filenames = [update_filenames]

        self.perform_PCA()

        (self.ref_params, error) = self.decompose_shape(
            numpy.reshape(self.data_mat[0], (-1, 1)))
        # self.createNewSet()

    def make_data_matrix(self, filename):
        """Read data from text file and store resulting data matrix

        For n samples of m points, the text file should be formatted
        as:

            n
            m
            x11 x12 ... x1m y11 y12 ... y1m
            x21 x22 ... x2m y21 y22 ... y2m
            ...
            xn1 xn2 ... xnm yn1 yn2 ... ynm

        """

        # scan the dataset :
        lines = []
        try:
            with open(filename, 'r') as f:
                lines.append(f.readline())
                lines.append(f.readline())
                nb_samples = int(lines[0].strip())
                for i in range(nb_samples):
                    lines.append(f.readline())
        except IOError:
            raise RuntimeError("no reading permission for file"+filename)
        self.num_shapes_in_dataset = int(lines[0].strip())
        self.num_points_in_shapes = int(lines[1].strip())
        if not (self.num_shapes_in_dataset and self.num_points_in_shapes):
            raise RuntimeError("Unable to read sizes needed from text file")

        self.num_shapes_in_demo = 0
        self.data_mat = numpy.empty(
            (self.num_shapes_in_dataset, self.num_points_in_shapes * 2))
        self.demo_data_mat = numpy.empty(
            (self.num_shapes_in_demo, self.num_points_in_shapes * 2))

        ref_line = lines[2].strip()
        ref_values = ref_line.split(' ')
        if not (len(ref_values) == self.num_points_in_shapes * 2):
            raise RuntimeError(
                "Unable to read appropriate number of points from text file for reference shape ")
        self.ref_shape = [float(i) for i in ref_values]
        self.data_mat[0] = self.ref_shape
        # -1 because we have already add the first shape (reference)
        for i in range(self.num_shapes_in_dataset-1):
            line = lines[i+3].strip()
            values = line.split(' ')
            if not (len(values) == self.num_points_in_shapes * 2):
                raise RuntimeError(
                    "Unable to read appropriate number of points from text file for shape " + str(i + 1))

            self.data_mat[i+1] = [float(j) for j in values]

    def make_data_matrix_2(self, filename):
        """Read data from text file and store resulting data matrix
        For n samples of m points, the text file should be formatted
        as:
            n
            m
            x11 x12 ... x1m y11 y12 ... y1m
            x21 x22 ... x2m y21 y22 ... y2m
            ...
            xn1 xn2 ... xnm yn1 yn2 ... ynm
        """

        # scan the dataset :
        lines = []
        try:
            with open(filename, 'r') as f:
                lines.append(f.readline())
                lines.append(f.readline())
                nb_samples = int(lines[1].strip())
                for i in range(nb_samples+5):
                    lines.append(f.readline())
        except IOError:
            raise RuntimeError("no reading permission for file"+filename)

        self.num_shapes_in_dataset = int(lines[1].strip())
        self.num_points_in_shapes = int(lines[3].strip())
        if not (self.num_shapes_in_dataset and self.num_points_in_shapes):
            raise RuntimeError("Unable to read sizes needed from text file")

        self.num_shapes_in_demo = 0
        self.data_mat = numpy.empty(
            (self.num_shapes_in_dataset, self.num_points_in_shapes * 2))
        self.demo_data_mat = numpy.empty(
            (self.num_shapes_in_demo, self.num_points_in_shapes * 2))

        ref_line = lines[5].strip()
        ref_values = ref_line.split(' ')
        if not (len(ref_values) == self.num_points_in_shapes * 2):
            raise RuntimeError(
                "Unable to read appropriate number of points from text file for reference shape ")
        self.ref_shape = [float(i) for i in ref_values]
        self.data_mat[0] = self.ref_shape

        # -1 because we have already add the first shape (reference)
        for i in range(self.num_shapes_in_dataset-1):
            line = lines[i+7].strip()
            values = line.split(' ')
            if not (len(values) == self.num_points_in_shapes * 2):
                raise RuntimeError(
                    "Unable to read appropriate number of points from text file for shape " + str(i + 1))
            self.data_mat[i+1] = [float(i) for i in values]

    def perform_PCA(self):
        """
        Calculate the top 'num_principle_components' principle
        components of the dataset, the observed variance of each
        component, and the mean.
        """
        cover_mat = numpy.cov(self.data_mat.T)
        eig_vals, eig_vecs = numpy.linalg.eig(cover_mat)
        self.principle_components = numpy.real(
            eig_vecs[:, 0:self.num_principle_components])
        self.parameter_variances = numpy.real(
            eig_vals[0:self.num_principle_components])
        self.mean_shape = self.data_mat.mean(0).reshape(
            (self.num_points_in_shapes * 2, 1))

    def get_euclidian_center(self):
        """
        Get the euclidian mean point by point
        return a vector of 70 mean-points
        """
        return numpy.mean(self.data_mat, 0), numpy.var(self.data_mat, 0)

    def get_euclidian_dist(self, shape):
        """
        Get the euclidian distance between a shape and the mean
        shape of the dataset, point by point, divided by variance
        for normalization
        """
        # data_mean, data_var = self.get_euclidian_center()
        dist = self.mean_shape - shape
        # sigma =numpy.sqrt(self.parameter_variances)
        return numpy.sum(dist*dist)

    def get_dist_to_ref(self, shape):
        params_1, _ = self.decompose_shape(
            self.data_mat[0].reshape((self.num_points_in_shapes * 2, 1)))
        params_2, _ = self.decompose_shape(shape)
        dist = numpy.array(params_1)-numpy.array(params_2)
        var = numpy.abs(numpy.array(self.get_parameter_variances()))
        ndist = dist*dist/var
        return numpy.sum(ndist[:1])

    def get_parameter_variances(self):
        """
        Return the variances associated which each of the top principle
        components
        """
        return self.parameter_variances

    def make_shape(self, params):
        """
        Generate a shape with the given parameter vector
        """
        if (not params.shape == (self.num_principle_components, 1)):
            raise RuntimeError(
                "Vector of parameters must have dimensions of (num_principle_components,1)")
        shape = self.mean_shape + numpy.dot(self.principle_components, params)
        return shape

    def make_shape_varying_param(self, params_to_vary, param_values):
        """
        Generate a shape modifying the given parameter
        """
        xb = numpy.zeros((self.num_principle_components, 1))
        for i in range(len(params_to_vary)):
            xb[params_to_vary[i] - 1, 0] = param_values[i]
        shape = self.make_shape(xb)
        return shape, xb

    def make_random_shape_from_uniform(self, params, params_to_vary, bounds):
        """
        Draw 'paramsToVary' values from uniform distribution with
        limits given by 'bounds' and make shape
        """
        xb = deepcopy(params)
        for i in range(len(params_to_vary)):
            sample = random.uniform(bounds[i, 0], bounds[i, 1])
            xb[params_to_vary[i] - 1, 0] = sample
        shape = self.make_shape(xb)
        return shape, xb

    def make_random_shape_from_traingular(self, params, params_to_vary, bounds, modes):
        """
        Draw 'paramsToVary' values from triangular distribution with
        limits given by 'bounds' and modes given by 'modes' and make
        shape       
        """
        b = deepcopy(params)
        for i in range(len(params_to_vary)):
            sample = random.triangular(bounds[i, 0], modes[i], bounds[i, 1])
            b[params_to_vary[i] - 1, 0] = sample
        return self.make_shape(b), b

    def decompose_shape(self, shape):
        """
        Convert shape into its 'num_principle_components' parameter
        values (projects onto the num_principle_components-dimensional
        space)
        """
        if (not shape.shape == (self.num_points_in_shapes * 2, 1)):
            print(shape.shape)
            print(self.num_points_in_shapes)
            raise RuntimeError(
                "Shape to decompose must be the same size as shapes used to make the dataset")
        params = numpy.dot(self.principle_components.T,
                           shape - self.mean_shape)

        approx_shape = self.mean_shape + \
            numpy.dot(self.principle_components, params)
        diff = abs(shape - approx_shape) ** 2
        error = sum(diff) / (self.num_points_in_shapes * 2)
        return params, error

    def normalise_mean_shape_height(self):
        """
        Normalizes the height of the mean shape attribute of the class.

        This method applies the ShapeModeler's normalise_shape_height
        function to the mean_shape attribute of the class instance.
        """
        self.mean_shape = ShapeModeler.normalise_shape_height(self.mean_shape)

    def show_mean_shape(self, block=True):
        """
        Displays the mean shape after normalizing it.

        The method normalizes the mean shape attribute then displays it
        using the ShapeModeler's show_shape function.

        Args:
            block (bool): If True, the display will block the execution
                          of the rest of the program until the plot
                          window is closed. Defaults to True.
    """
        ShapeModeler.show_shape(
            ShapeModeler.normalise_shape(self.mean_shape), block)

    def extend_data_mat(self, shape):
        """
        Add the demonstrated shape to the data matrix to perform PCA.
        We want to update principle components by taking into account
        the demo shapes.
        """
        self.num_shapes_in_dataset += 1
        self.num_shapes_in_demo += 1
        self.data_mat = numpy.append(self.data_mat, shape.T, axis=0)
        self.demo_data_mat = numpy.append(self.demo_data_mat, shape.T, axis=0)
        self.perform_PCA()
        (self.ref_params, error) = self.decompose_shape(
            numpy.reshape(self.data_mat[0], (-1, 1)))

    def save_all(self):
        """ 
        Save the inital shape + the demo shapes into a new dataset.
        """
        if self.update_filenames:
            filename = self.update_filenames[0]
            if filename:
                print('saving in' + filename)
                if not os.path.exists(filename):
                    raise RuntimeError("path to dataset" +
                                       filename + "not found")
                try:
                    with open(filename, 'wb') as f:
                        f.write(str.encode('nb_sample:\n'))
                        f.write(str.encode('%i\n' %
                                self.num_shapes_in_dataset))
                        f.write(str.encode('nb_pts:\n'))
                        f.write(str.encode('%i\n' % self.num_points_in_shapes))
                        f.write(str.encode('ref:\n'))
                        f.write(str.encode(
                            ' '.join(map(str, self.data_mat[0])) + '\n'))
                        f.write(str.encode('...\n'))
                        for i in range(len(self.data_mat)-1):
                            f.write(str.encode(
                                ' '.join(map(str, self.data_mat[i+1]))))
                            f.write(str.encode('\n'))
                except IOError:
                    raise RuntimeError(
                        "no writing permission for file" + filename)
            else:
                raise Exception("Unknown filename")

    def save_demo(self):
        """
        Save the demo shape into a new data set.
        """
        if len(self.update_filenames) > 1:
            for filename in self.update_filenames[1:]:
                if filename:
                    if not os.path.exists(filename):
                        raise RuntimeError(
                            "path to dataset"+filename+"not found")
                    try:
                        with open(filename, 'wb') as f:
                            f.write(str.encode('nb_sample:\n'))
                            f.write(str.encode('%i\n' %
                                    self.num_shapes_in_dataset))
                            f.write(str.encode('nb_pts:\n'))
                            f.write(str.encode('%i\n' %
                                    self.num_points_in_shapes))
                            f.write(str.encode('ref:\n'))
                            f.write(str.encode(
                                ' '.join(map(str, self.data_mat[0])) + '\n'))
                            f.write(str.encode('...\n'))
                            for i in range(len(self.data_mat)-1):
                                f.write(
                                    str.encode(' '.join(map(str, self.demo_data_mat[i+1]))))
                                f.write(str.encode('\n'))
                    except IOError:
                        raise RuntimeError(
                            "no writing permission for file"+filename)
                else:
                    raise Exception("Unknown filename")

    def save_params(self, params, letter):
        """
        Save parameters in new dataset
        """
        if self.param_filename:
            filename = self.param_filename
            print('saving params in'+filename)
            if not os.path.exists(filename):
                raise RuntimeError("path to dataset"+filename+"not found")
            lines = []
            try:
                with open(filename, 'rb') as f:
                    for i in range(52):
                        lines.append(f.readline())
            except IOError:
                raise RuntimeError("no reading permission for file"+filename)
            try:
                with open(filename, 'wb') as f:
                    for i in range(52):
                        f.write(lines[i])
                        test = lines[i].replace(
                            '[', '').replace(']\n', '') == letter
                        if test:
                            lines[i+1] = str(params).replace('[',
                                                             '').replace(']', '')+'\n'
            except IOError:
                raise RuntimeError("no writing permission for file"+filename)

    def param_matrix(self):
        """
        Generates a parameter matrix for all shapes in the dataset.

        This method creates a matrix where each row corresponds to a
        shape  in the dataset, and each column corresponds to a
        principal component. Each value in the matrix represents the
        parameter value of a specific principal component for a
        specific shape.

        Returns:
            param_mat (numpy.ndarray): The matrix of shape parameters
                                       for each shape in the dataset.
        """
        param_mat = numpy.zeros(
            (self.num_shapes_in_dataset, self.num_principle_components))
        for i in range(self.num_shapes_in_dataset):
            shape = self.data_mat[i, :]
            params, _ = self.decompose_shape(
                shape.reshape((self.num_points_in_shapes * 2, 1)))
            param_mat[i, :] = params[:, 0]
        return param_mat

    def get_var(self):
        """
        Computes the variance of the data matrix along the zeroth axis.

        This method calculates the variance of the data_mat attribute
        of the class, where data_mat is assumed to be a 2D numpy array.

        Returns:
            numpy.ndarray: The variance of the data_mat attribute along
                           the zeroth axis.
        """
        return numpy.var(self.data_mat, 0)

    def get_clusters(self):
        """
        Get the different clusters of the letter
        """
        X = self.data_mat

        ms = MeanShift(bandwidth=1.9).fit(X)
        cluster_centers = ms.cluster_centers_
        n_clusters = len(cluster_centers)
        labels = ms.labels_
        var = []
        for i in range(n_clusters):
            var.append(numpy.var(X[labels == i]))
        var = numpy.array(var)

        return cluster_centers, n_clusters, var

    def get_min_dist(self, shape):
        """
        Give the distance between the demo and the closest cluster of
        the letter
        """

        clusters, _, var = self.get_clusters()
        scores = []
        for i in range(len(clusters)):
            dist = (clusters[i, :].reshape(
                (self.num_points_in_shapes * 2, 1))-shape)
            scores.append(numpy.sum(dist*dist))
        return numpy.min(numpy.array(scores))

    def get_centers(self):

        clusters, _, _ = self.get_clusters()
        centers = [clusters[i, :].reshape(
            (self.num_points_in_shapes * 2, 1)) for i in range(len(clusters))]
        return centers

    @staticmethod
    def show_shape(shape, block=False):
        """
        Show shape with random colour
        """
        num_points_in_shape = int(len(shape) / 2)
        x_shape = shape[0:num_points_in_shape]
        y_shape = shape[num_points_in_shape:]

        plt.plot(x_shape, -y_shape, c=numpy.random.rand(3, 1))
        plt.axis([-1, 1, -1, 1])
        if block:
            plt.show(block=block)  # block=False <-> plt.draw
        else:
            plt.draw()

    @staticmethod
    def normalise_shape(shape):
        """
        Normalise shape so that max dimension is 1 
        """
        num_points_in_shape = int(len(shape) / 2)
        x_shape = shape[0:num_points_in_shape]
        y_shape = shape[num_points_in_shape:]

        # shift so centre of shape is at (0,0)
        x_range = max(x_shape) - min(x_shape)
        y_range = max(y_shape) - min(y_shape)
        x_shape = x_shape - (max(x_shape) - x_range / 2)
        y_shape = y_shape - (max(y_shape) - y_range / 2)

        # normalise shape
        scale = max(x_range, y_range)
        if scale < 1e-10:
            print('Warning: shape is probably a bunch of points on top of each other...')

        x_shape = x_shape / scale
        y_shape = y_shape / scale

        new_shape = numpy.zeros(shape.shape)
        new_shape[0:num_points_in_shape] = x_shape
        new_shape[num_points_in_shape:] = y_shape
        return new_shape

    @staticmethod
    def get_shape_centre(shape):
        """
        Calculate the centre of the shape
        """
        num_points_in_shape = int(len(shape) / 2)
        x_shape = shape[0:num_points_in_shape]
        y_shape = shape[num_points_in_shape:]

        x_range = max(x_shape) - min(x_shape)
        y_range = max(y_shape) - min(y_shape)
        x_centre = (max(x_shape) - x_range / 2)
        y_centre = (max(y_shape) - y_range / 2)
        return [x_centre, -y_centre]

    @staticmethod
    def normalise_shape_height(shape):
        """
        Normalise shape so that height is 1 
        """
        num_points_in_shape = int(len(shape) / 2)
        x_shape = shape[0:num_points_in_shape]
        y_shape = shape[num_points_in_shape:]

        # shift so centre of shape is at (0,0)
        x_range = max(x_shape) - min(x_shape)
        y_range = max(y_shape) - min(y_shape)
        x_centre = (max(x_shape) - x_range / 2)
        y_centre = (max(y_shape) - y_range / 2)
        x_shape = x_shape - x_centre
        y_shape = y_shape - y_centre

        # normalise shape
        scale = y_range
        if scale < 1e-10:
            print('Warning: shape is probably a bunch of points on top of each other...')

        x_shape = x_shape / scale
        y_shape = y_shape / scale

        new_shape = numpy.zeros(shape.shape)
        new_shape[0:num_points_in_shape] = x_shape
        new_shape[num_points_in_shape:] = y_shape
        return new_shape

    @staticmethod
    def normalise_shape_width(shape):
        """
        Normalise shape so that width is 1 
        """
        num_points_in_shape = int(len(shape) / 2)
        x_shape = shape[0:num_points_in_shape]
        y_shape = shape[num_points_in_shape:]

        # shift so centre of shape is at (0,0)
        x_range = max(x_shape) - min(x_shape)
        y_range = max(y_shape) - min(y_shape)
        x_centre = (max(x_shape) - x_range / 2)
        y_centre = (max(y_shape) - y_range / 2)
        x_shape = x_shape - x_centre
        y_shape = y_shape - y_centre

        # normalise shape
        scale = x_range
        if scale < 1e-10:
            print('Warning: shape is probably a bunch of points on top of each other...')

        x_shape = x_shape / scale
        y_shape = y_shape / scale

        new_shape = numpy.zeros(shape.shape)
        new_shape[0:num_points_in_shape] = x_shape
        new_shape[num_points_in_shape:] = y_shape
        return new_shape

    @staticmethod
    def normalise_and_show_shape(shape, block=False):
        """
        Normalise shape so that max dimension is 1 and then show
        """
        shape = ShapeModeler.normalise_shape(shape)
        ShapeModeler.show_shape(shape, block)

    @staticmethod
    def show_shape_score(shape, scores, block=False):
        """
        Show shape with random colour
        """
        num_points_in_shape = len(shape) // 2
        x_shape = numpy.reshape(numpy.array(
            shape[0:num_points_in_shape]), num_points_in_shape)
        y_shape = numpy.reshape(numpy.array(
            shape[num_points_in_shape:]), num_points_in_shape)

        # plt.plot(x_shape, -y_shape, c=numpy.random.rand(3, 1))
        # plt.axis([-1, 1, -1, 1])
        if block:
            plt.show(block=block)  # block=False <-> plt.draw
        else:
            plt.errorbar(x_shape, -y_shape, yerr=scores/numpy.max(scores))
            plt.draw()
