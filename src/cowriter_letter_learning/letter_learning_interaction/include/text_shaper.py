#!usr/bin/env python3
"""
This Python file contains three classes: ShapedWord, TextShaper, and
ScreenManager. The purpose of these classes is to generate a visual
representation of words by combining the individual letter paths,
scaling, and positioning them accordingly.

ShapedWord:

Represents a shaped word, with its original word, paths of individual
letters, and an optional origin. Provides methods to check if a point
is on the word, get letters' bounding boxes, and compute a bounding
box for a given path.

TextShaper:

Responsible for shaping a word into a global shape by assembling the
paths of its individual letters. Provides two static methods,
shape_word and reference_boundingboxes, to assemble the paths of the
letters into a global shape and calculate reference bounding boxes for
the letters, respectively.


ScreenManager:

Manages the on-screen placement and interaction of words and reference
bounding boxes. Provides methods to clear current words and reference
bounding boxes, place a shaped word and reference bounding boxes on the
screen, find the closest letter to a point, find a letter on the screen
that corresponds to a given path, split a path into individual letters
based on reference bounding boxes, and check if two bounding boxes
intersect. Computes the global reference bounding box that encloses all
the reference bounding boxes.

Together, these classes enable users to create, manipulate, and
visualize text in a 2D space, taking into account the positioning and
scaling of individual letters. The primary use case for this code is in
applications where text is drawn or manipulated, and individual letter
paths need to be managed, such as handwriting recognition.
"""


from typing import Dict, List, Optional, Tuple, Union
from collections import OrderedDict
from scipy import interpolate
import numpy as np
import logging

from .shape_learner_manager import ShapeLearnerManager
from .shape_modeler import ShapeModeler

logger = logging.getLogger("text_shaper")
logger.setLevel(logging.DEBUG)


SIZESCALE_HEIGHT = 0.016  # Desired height of 'a' (metres)
SIZESCALE_WIDTH = 0.016  # Desired width of 'a' (metres)

TEMPLATE_SCALING = 1.0  # scale factor for the reference templates

# (width, height_above_baseline, height_below_baseline) with reference a = (1, 1, 0)
LETTER_BOUNDINGBOXES = {
    "a": (1.00, 1.0, 0.0),
    "b": (1.38, 2.38, 0.0),
    "c": (1.00, 1.0, 0.0),
    "d": (1.25, 1.89, 0.0),
    "e": (1.16, 1.0, 0.0),
    "f": (1.10, 2.38, 1.33),
    "g": (1.21, 1.0, 1.33),
    "h": (1.22, 2.38, 0.0),
    "i": (0.72, 1.0, 0.0),
    "j": (0.86, 1.0, 1.33),
    "k": (1.20, 2.38, 0.0),
    "l": (1.12, 2.38, 0.0),
    "m": (2.08, 1.0, 0.0),
    "n": (1.55, 1.0, 0.0),
    "o": (1.06, 1.0, 0.0),
    "p": (1.10, 1.0, 1.33),
    "q": (1.44, 1.0, 1.33),
    "r": (1.00, 1.0, 0.0),
    "s": (1.30, 1.0, 0.0),
    "t": (0.93, 1.89, 0.0),
    "u": (1.33, 1.0, 0.0),
    "v": (1.58, 1.0, 0.0),
    "w": (1.88, 1.0, 0.0),
    "x": (1.90, 1.0, 0.0),
    "y": (1.34, 1.0, 1.33),
    "z": (1.38, 1.0, 1.33),
}


class ShapedWord:
    """
    Container for the paths of the letters of a given word.
    It also exposes the bounding boxes of each letters and of the whole
    word.
    """

    def __init__(
        self,
        word: str,
        paths: List[List[Tuple[float, float]]],
        origin: Optional[List[float]] = None,
    ) -> None:
        """
        Initializes ShapedWord class.

        Args:
        - word: str, the word to be represented by the class.
        - paths: list of lists, where each sublist contains tuples of
                 floats representing the (x, y) coordinates of each
                 point in the path of a letter
        - origin: list, optional, a list of x and y coordinates
                  representing the origin of the word.
        """
        self.word: str = word
        self._paths: List[List[Tuple[float, float]]] = paths

        self.bounding_boxes: List[Tuple[int, int, int, int]] = self._compute_bbs()
        self.global_bounding_box: Tuple[
            float, float, float, float
        ] = self._compute_global_bb()

        self.origin = origin if origin is not None else [0.0, 0.0]

    def get_letters_paths(
        self, absolute: Optional[bool] = None
    ) -> List[List[Tuple[float, float]]]:
        """
        Returns the paths of the letters.

        Args:
        - absolute: bool, optional, if True returns the paths in
                    absolute coordinates.

        Returns:
        - list, the paths of the letters.
        """
        if absolute is None:
            absolute = True
        if absolute:
            return [
                [(x + self.origin[0], y + self.origin[1]) for x, y in path]
                for path in self._paths
            ]
        else:
            return self._paths

    def get_letters_bounding_boxes(
        self, absolute: bool = True
    ) -> Union[
        List[Tuple[float, float, float, float]], List[Tuple[int, int, int, int]]
    ]:
        """
        Returns the bounding boxes of the letters.

        Args:
        - absolute: bool, optional, if True returns the bounding boxes
                    in absolute coordinates.

        Returns:
        - list of tuples of floats, the bounding boxes of the letters.
        """

        if absolute:
            return [
                (
                    x1 + self.origin[0],
                    y1 + self.origin[1],
                    x2 + self.origin[0],
                    y2 + self.origin[1],
                )
                for x1, y1, x2, y2 in self.bounding_boxes
            ]
        else:
            return self.bounding_boxes

    def get_global_bb(self, absolute: bool = True) -> Tuple[float, float, float, float]:
        """
        Returns the global bounding box.

        Args:
        - absolute: bool, optional, if True returns the global bounding
                    box in absolute coordinates.

        Returns:
        - tuple, the global bounding box.
        """

        if absolute:
            x1, y1, x2, y2 = self.global_bounding_box
            return (
                x1 + self.origin[0],
                y1 + self.origin[1],
                x2 + self.origin[0],
                y2 + self.origin[1],
            )
        else:
            return self.global_bounding_box

    @staticmethod
    def compute_boundingbox(path: list) -> Tuple[float, float, float, float]:
        """
        Computes the bounding box of a given path.

        Args:
        - path: list, a list of coordinates representing the path.

        Returns:
        - tuple, the bounding box of the path.
        """

        x_min: int = 2000
        y_min: int = 2000
        x_max: int = 0
        y_max: int = 0

        for x, y in path:
            if x < x_min:
                x_min = x
            if y < y_min:
                y_min = y

            if x > x_max:
                x_max = x
            if y > y_max:
                y_max = y

        return x_min, y_min, x_max, y_max

    def _compute_bbs(self) -> List[Tuple[int, int, int, int]]:
        """
        Computes the bounding boxes for each letter in the word.

        :return: A list of tuples representing the bounding box for each letter.
                 Each tuple contains four integers representing the left, bottom,
                 right, and top coordinates of the bounding box, respectively.
        """

        bbs = []

        for path in self._paths:
            bbs.append(ShapedWord.compute_boundingbox(path))

        return bbs

    def _compute_global_bb(self) -> Tuple[float, float, float, float]:
        """
        Computes the global bounding box of the word by iterating
        through the bounding boxes of each letter and determining
        the minimum and maximum values for each dimension.

        :returns: A tuple of the form (x_min, y_min, x_max, y_max),
                  representing the global bounding box.
        """

        gx_min = 2000
        gy_min = 2000
        gx_max = 0
        gy_max = 0

        for x_min, y_min, x_max, y_max in self.bounding_boxes:
            if x_min < gx_min:
                gx_min = x_min
            if y_min < gy_min:
                gy_min = y_min

            if x_max > gx_max:
                gx_max = x_max
            if y_max > gy_max:
                gy_max = y_max

        return gx_min, gy_min, gx_max, gy_max

    def downsample(self, downsampling_factor: int) -> None:
        """
        Downsamples the paths of the letters of the word by a factor of
        downsampling_factor, replacing the existing paths in object.

        :param downsampling_factor: An integer representing the
        downsampling factor.

        :returns: None
        """

        # Create an empty list to hold the downsampled paths
        downsampled_paths = []

        # Iterate over each path in the existing paths list
        for path in self._paths:
            # Extract the x and y coordinates from each point in the path
            x_shape = [p[0] for p in path]
            y_shape = [p[1] for p in path]

            # Create a new set of x and y coordinates with the desired number of points
            t_current = np.linspace(0, 1, len(x_shape))
            t_desired = np.linspace(0, 1, int(len(x_shape) / downsampling_factor))

            # Curry scipy linear interpolation and use it to transform x_ and y_shape
            curried_interpolation = interpolate.interp1d(
                t_current, x_shape[:], kind="linear"
            )
            x_shape = curried_interpolation(t_desired)
            curried_interpolation = interpolate.interp1d(
                t_current, y_shape[:], kind="linear"
            )
            y_shape = curried_interpolation(t_desired)

            # Combine the x and y coordinates into a list of tuples
            # representing the downsampled path
            path = list(zip(x_shape, y_shape))

            # Add the downsampled path to the list of downsampled paths
            downsampled_paths.append(path)

        # Replace the existing paths with the downsampled paths
        self._paths = downsampled_paths

        # Recompute the bounding boxes of the letters
        # and the global bounding box of the word
        self.bounding_boxes = self._compute_bbs()
        self.global_bounding_box = self._compute_global_bb()

    def _isinbb(self, x: float, y: float, bb: tuple) -> bool:
        """
        Determines whether a point (x,y) is inside a bounding box.

        :param x: A float representing the x-coordinate of the point.
        :param y: A float representing the y-coordinate of the point.
        :param bb: A tuple of the form (x_min, y_min, x_max, y_max),
                   representing the bounding box.
        :returns: A boolean indicating whether the point is inside the
                  bounding box.
        """
        x1, y1, x2, y2 = bb
        return x1 <= x <= x2 and y1 <= y <= y2
        x1, y1, x2, y2 = bb
        return x1 <= x <= x2 and y1 <= y <= y2

    def ispointonword(
        self, x: float, y: float
    ) -> Tuple[bool, Optional[str], Optional[Tuple[float, float, float, float]]]:
        """
        Check if a given point lies within the bounding box of any of
        the letters in the word.

        Args:
            x (float): x-coordinate of the point to check
            y (float): y-coordinate of the point to check

        Returns:
            A tuple (is_on_letter, letter, letter_bb) where:
                - is_on_letter (bool): True if the point is within the
                                       bounding box of any letter,
                                       False otherwise
                - letter (str or None): The letter that the point lies
                                        on (if is_on_letter is True),
                                        None otherwise
                - letter_bb (tuple of float or None):
                    The bounding box of the letter that the point lies
                    on (if is_on_letter is True), None otherwise
        """

        for i, bb in enumerate(self.bounding_boxes):
            if self._isinbb(x - self.origin[0], y - self.origin[1], bb):
                return True, self.word[i], self.get_letters_bounding_boxes()[i]

        return False, None, None


class TextShaper:
    """
    A class that is responsible for shaping a word into a global shape
    by assembling the paths of its individual letters.
    """

    @staticmethod
    def shape_word(
        word: ShapeLearnerManager, downsampling_factor: Optional[int] = None
    ) -> ShapedWord:
        """
        Assembles the paths of the letters of the given word into a
        global shape.

        :param word: A ShapeLearnerManager instance for the current
                     word
        :param downsampling_factor: If provided, the final shape of each
                                    letter is (independently) resampled
                                    to (nb_pts / downsampling_factor)
                                    points

        :returns: A ShapedWord that contains the path of individual
                  letters
        """
        paths: List[List[Tuple[float, float]]] = []

        offset_x: float = 0
        offset_y: float = 0
        for shape in word.shapes_of_current_collection():
            path: List[Tuple[float, float]] = []

            w, ah, bh = LETTER_BOUNDINGBOXES[shape.shape_type]
            scale_factor = ah + bh  # height ratio between this letter and a 'a'
            # No need for a width scaling since the shape are only
            # *height*-normalized (cf below)

            glyph = ShapeModeler.normalise_shape_height(shape.path)  # type: ignore
            num_points_in_shape: int = len(glyph) // 2

            x_shape = glyph[0:num_points_in_shape].flatten().tolist()
            y_shape = glyph[num_points_in_shape:].flatten().tolist()

            # Adjusting the position of the letter based on the previous one
            if offset_x != 0 or offset_y != 0:  # not the first letter
                offset_x -= x_shape[0] * SIZESCALE_WIDTH * scale_factor
                offset_y += y_shape[0] * SIZESCALE_HEIGHT * scale_factor

            for i in range(num_points_in_shape):
                x = x_shape[i] * SIZESCALE_WIDTH * scale_factor
                y = -y_shape[i] * SIZESCALE_HEIGHT * scale_factor

                x += offset_x
                y += offset_y

                path.append((x, y))

            paths.append(path)

            # Adding a 'dot' to the letter if it's an 'i' or 'j'
            if shape.shape_type in ["i", "j"]:
                # HACK: waiting for proper multi-stroke learning
                logger.info("Adding a 'dot' to the letter")
                dot_path = [
                    (
                        offset_x + 0.0 * SIZESCALE_WIDTH * scale_factor,
                        offset_y + 0.8 * SIZESCALE_WIDTH * scale_factor,
                    ),
                    (
                        offset_x + 0.05 * SIZESCALE_WIDTH * scale_factor,
                        offset_y + 0.85 * SIZESCALE_HEIGHT * scale_factor,
                    ),
                    (
                        offset_x + 0.0 * SIZESCALE_WIDTH * scale_factor,
                        offset_y + 0.85 * SIZESCALE_HEIGHT * scale_factor,
                    ),
                    (
                        offset_x + 0.0 * SIZESCALE_WIDTH * scale_factor,
                        offset_y + 0.8 * SIZESCALE_HEIGHT * scale_factor,
                    ),
                ]
                paths.append(dot_path)

            # Connect the letter to the ending point of the previous one
            offset_x = path[-1][0]
            offset_y = path[-1][1]

        return ShapedWord(word.current_collection, paths)

    @staticmethod
    def reference_boundingboxes(word: str) -> List[Tuple[float, float, float, float]]:
        """
        Calculates the reference bounding boxes for the letters of the
        given word.

        :param word: A string representing the word

        :returns: A list of tuples, each tuple representing the
        bounding box of a letter in the format (x1, y1, x2, y2)
        """
        bbs: List[Tuple[float, float, float, float]] = []

        current_x: float = 0

        for letter in word:
            w, ah, bh = LETTER_BOUNDINGBOXES[letter]

            w *= SIZESCALE_WIDTH * TEMPLATE_SCALING
            ah *= SIZESCALE_HEIGHT * TEMPLATE_SCALING
            bh *= SIZESCALE_HEIGHT * TEMPLATE_SCALING

            bb: Tuple[float, float, float, float] = (current_x, -bh, current_x + w, ah)

            bbs.append(bb)
            current_x += w

        return bbs


class ScreenManager:
    """
    A class that manages the on-screen placement and interaction of
    words and reference bounding boxes.
    """

    def __init__(self, width: float, height: float):
        """
        :param width: width, in meters, of the writing zone
        :param height: height, in meters, of the writing zone
        """
        self.width = width
        self.height = height

        self.words: List["ShapedWord"] = []

        self.ref_word: str = ""
        self.ref_boundingboxes: List[Tuple[float, float, float, float]] = []

    def clear(self) -> None:
        """
        Clears the current word list and reference bounding boxes.
        """
        self.words = []
        self.ref_word = ""
        self.ref_boundingboxes = []

    def place_word(self, shaped_word: "ShapedWord") -> "ShapedWord":
        """
        Places the shaped_word on the screen.

        Note that this method *modifies* its parameter!

        :param shaped_word: A ShapedWord instance to place on the screen

        :returns: The placed ShapedWord instance
        """
        shaped_word.origin = [self.width * 0.25, self.height * 0.7]
        self.words.append(shaped_word)
        return shaped_word

    def place_reference_boundingboxes(
        self, word: str
    ) -> List[Tuple[float, float, float, float]]:
        """
        Places the reference bounding boxes for the given word on the
        screen.

        :param word: A string representing the word

        :returns: A list of placed reference bounding boxes as tuples
                  (x1, y1, x2, y2)
        """
        bbs = TextShaper.reference_boundingboxes(word)

        origin = [self.width * 0.25, self.height * 0.25]

        self.ref_word = word
        self.ref_boundingboxes = [
            (x1 + origin[0], y1 + origin[1], x2 + origin[0], y2 + origin[1])
            for x1, y1, x2, y2 in bbs
        ]

        return self.ref_boundingboxes

    def closest_letter(
        self, x: float, y: float, strict: bool = False
    ) -> Tuple[Optional[str], Optional[Tuple[float, float, float, float]]]:
        """
        Returns the letter (+ bounding box) on the screen the closest
        to (x, y) in screen coordinates, or None if no word has been
        drawn.

        If strict=True, returns a letter only if (x, y) is *on* (the
        bounding box of) a letter

        :param x: The x-coordinate of the point
        :param y: The y-coordinate of the point
        :param strict: Whether to return a letter only if (x, y) is on
                       the bounding box of the letter

        :returns: A tuple containing the closest letter and its
                  bounding box, or (None, None) if not found
        """
        if not self.words:
            logger.debug("Closest letter: no word drawn yet!")
            return "", None

        for word in self.words:
            on_letter, letter, bb = word.ispointonword(x, y)
            if on_letter:
                logger.debug("Closest letter: on top of '%s' bounding box", letter)
                return letter, bb

        if strict:
            return "", None

        # not on top of a bounding-box: compute distances to each letter, and
        # return the closest one

        distances = {}

        for word in self.words:
            for i, bb in enumerate(word.get_letters_bounding_boxes()):
                x1, y1, x2, y2 = bb
                bbx = float(x2 + x1) / 2
                bby = float(y2 + y1) / 2
                distance = (x - bbx) * (x - bbx) + (y - bby) * (y - bby)
                # store the letter with its distance
                distances[distance] = (word.word[i], bb)

        shortest_distance = sorted(distances.keys())[0]
        letter, bb = distances[shortest_distance]

        logger.debug("Closest letter: '%s'", letter)
        return letter, bb

    def find_letter(
        self, path: List[Tuple[float, float]]
    ) -> Tuple[Optional[str], Optional[Tuple[float, float, float, float]]]:
        """
        Finds the letter on the screen that corresponds to the given
        path.

        :param path: A list of tuples representing the x, y coordinates
                     of the path

        :returns: A tuple containing the found letter and its bounding
                  box, or (None, None) if not found
        """

        x, y = ShapeModeler.get_shape_centre(path)
        return self.closest_letter(x, y)

    def split_path_from_template(
        self, path: List[Tuple[float, float]]
    ) -> Dict[str, List[Tuple[float, float]]]:
        """
        Returns a dict of ('letter':path)s by splitting a given path
        (typically, a full word) on the boundaries of the current
        screen reference bounding boxes.

        Returns an empty dict if the path does not intersect with all
        the letters' bounding boxes.

        :param path: A list of tuples representing the x, y coordinates
                     of the path

        :returns: A dictionary containing the split paths for each
                  letter as ('letter':path)
        """
        path_bb = ShapedWord.compute_boundingbox(path)

        # first, check that the path does intersect with *each* of the
        # reference bbs.
        for bb in self.ref_boundingboxes:
            if not ScreenManager.intersect(bb, path_bb):
                return {}

        current_bb: int = 0

        glyphs = OrderedDict()
        if self.ref_boundingboxes:
            glyph: List[Tuple[float, float]] = []
            for i, point in enumerate(path):
                x, y = point

                glyph.append((x, y))

                if x > self.ref_boundingboxes[current_bb][2]:  # x > bb.x_max
                    # last bounding box? put everythin remaining bits in the last glyph
                    if current_bb == len(self.ref_boundingboxes) - 1:
                        glyph.extend(path[i + 1 :])  # noqa: E203
                        break
                    else:
                        glyphs[self.ref_word[current_bb]] = glyph[:]

                    current_bb += 1
                    glyph = []

            glyphs[self.ref_word[current_bb]] = glyph[:]

        return glyphs

    @staticmethod
    def intersect(
        bb1: Tuple[float, float, float, float], bb2: Tuple[float, float, float, float]
    ) -> bool:
        """
        Returns True if two bounding boxes intersect.

        :param bb1: A tuple representing the first bounding box (x1,
                    y1, x2, y2)
        :param bb2: A tuple representing the second bounding box (x1,
                    y1, x2, y2)

        :returns: A boolean indicating whether the bounding boxes
                  intersect
        """
        x11, y11, x12, y12 = bb1
        x21, y21, x22, y22 = bb2

        return False if x11 > x22 or x21 > x12 or y11 > y22 or y21 > y12 else True

    def _compute_global_ref_bb(self) -> Tuple[float, float, float, float]:
        """
        Computes the global reference bounding box that encloses all
        the reference bounding boxes.

        :returns: A tuple representing the global reference bounding
        box (x_min, y_min, x_max, y_max)
        """
        gx_min: float = 2000
        gy_min: float = 2000
        gx_max: float = 0
        gy_max: float = 0

        for x_min, y_min, x_max, y_max in self.ref_boundingboxes:
            if x_min < gx_min:
                gx_min = x_min
            if y_min < gy_min:
                gy_min = y_min

            if x_max > gx_max:
                gx_max = x_max
            if y_max > gy_max:
                gy_max = y_max

        return gx_min, gy_min, gx_max, gy_max
