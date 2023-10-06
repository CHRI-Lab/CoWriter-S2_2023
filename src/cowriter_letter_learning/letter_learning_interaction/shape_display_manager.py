#!/usr/bin/env python3

"""
Shape Display Manager
=====================

This module contains the ShapeDisplayManager class, responsible for
managing the positioning of shapes in a grid-like structure. It assigns
new shapes to their appropriate location and can convert the location
of an event into the shape which it was done around.

The module imports numpy for numerical operations and the typing module
for type annotations. The module contains three lists of preferred
shape positions for three different shape types. The dimensions of the
shapes are defined as constants.

Classes
-------
ShapeDisplayManager:
    Manages the positioning of shapes. Assigns new shapes to their
    appropriate location, and can convert the location of an event into
    the shape which it was done around.

    Attributes
    ----------
    shapes_drawn : np.ndarray
        A 3D array representing the drawn shapes and their IDs.

    Methods
    -------
    clear_all_shapes():
        Clears all drawn shapes from the manager.
    display_new_shape(shape_type_code: int) -> List[float]:
        Displays a new shape with the given shape_type_code in the next
        available position.
    is_possible_to_display_new_shape(shape_type_code: int) -> bool:
        Checks if it's possible to display a new shape with the given
        shape_type_code.
    index_of_location(location: List[float]) -> Tuple[int, int]:
        Finds the row and column indices of the location in the grid.
    shape_at_location(location: List[float]) -> Tuple[int, int]:
        Finds the shape type code and shape ID at the given location.
    closest_shapes_to_location(location: List[float])
        -> Tuple[List[int], List[int]]:
            Map a location to the closest shape(s) drawn at that location.
    display_shape_at_location(shape_type_code: int, location: List[float])
        -> bool:
            Displays a shape with the given shape_type_code at the specified location.
"""
import numpy as np
from typing import List, Tuple

shape_width: float = 0.04
shape_height: float = 0.0465
shape_size: np.ndarray = np.array([shape_width, shape_height])

# List of preferred shape cells
position_list_shape0: List[List[int]] = [
    [1, 1],
    [0, 0],
    [2, 0],
    [1, 0],
    [0, 1],
    [2, 1],
    [0, 2],
    [2, 2],
    [1, 2],
    [0, 3],
    [2, 3],
    [1, 3],
    [1, 4],
    [0, 4],
    [2, 4],
]
position_list_shape1: List[List[int]] = [
    [1, 2],
    [0, 2],
    [2, 2],
    [1, 1],
    [1, 3],
    [0, 1],
    [0, 3],
    [2, 1],
    [2, 3],
    [1, 0],
    [0, 0],
    [2, 0],
    [1, 4],
    [0, 4],
    [2, 4],
]
position_list_shape2: List[List[int]] = [
    [1, 3],
    [0, 4],
    [2, 4],
    [1, 4],
    [1, 2],
    [0, 3],
    [2, 3],
    [0, 2],
    [2, 2],
    [0, 1],
    [2, 1],
    [1, 1],
    [1, 0],
    [0, 0],
    [2, 0],
]
position_list: List[List[List[int]]] = [
    position_list_shape0,
    position_list_shape1,
    position_list_shape2,
]


class ShapeDisplayManager:
    """
    Manages the positioning of shapes. Assigns new shapes to their
    appropriate location, and can convert the location of an event into
    the shape which it was done around.

    Attributes
    ----------
    shapes_drawn : np.ndarray
        A 3D array representing the drawn shapes and their IDs.
    """

    def __init__(self):  # TODO allow for generic shape and dimensions of display grid
        # 3rd dim: shape_type_code, ID
        self.shapes_drawn = np.ones((3, 5, 2)) * np.nan

    def clear_all_shapes(self) -> None:
        """
        Clears all drawn shapes from the manager.
        """
        self.shapes_drawn = np.ones((3, 5, 2)) * np.nan  # 3rd dim: shape_type_code, ID

    def display_new_shape(self, shape_type_code: int) -> List[float]:
        """
        Displays a new shape with the given shape_type_code in the next
        available position.

        Parameters
        ----------
        shape_type_code : int
            The type code of the shape to display.

        Returns
        -------
        List[float]
            The position of the new shape as [x, y].
        """
        if shape_type_code > len(position_list) - 1:
            print("I don't know how to position that shape")
            return [-1, -1]
        else:
            row: int = -1
            column: int = -1
            found_space: bool = False
            position_list_index: int = 0

            while (not found_space) and (
                position_list_index < len(position_list[shape_type_code])
            ):
                # Check next position in position list for this shape
                [row_test, column_test] = position_list[shape_type_code][
                    position_list_index
                ]

                if np.isnan(self.shapes_drawn[row_test, column_test, 0]):
                    # Space is available
                    row = row_test
                    column = column_test
                    found_space = True
                else:
                    # Space is not available - keep looking
                    position_list_index += 1

            if found_space:
                # generate shape id and add shape to self.shapes_drawn
                shape_id = np.equal(self.shapes_drawn[:, :, 0], shape_type_code).sum()
                self.shapes_drawn[row, column, 0] = shape_type_code
                self.shapes_drawn[row, column, 1] = shape_id
            else:
                print("I cannot draw here.")

            # Calculate and return the position of the new shape
            num_rows = self.shapes_drawn.shape[0]
            position = [
                (column + 0.5) * shape_width,
                ((num_rows - 1) - row + 0.5) * shape_height,
            ]

            return position

    def is_possible_to_display_new_shape(self, shape_type_code: int) -> bool:
        """
        Checks if it's possible to display a new shape with the given
        shape_type_code.

        Parameters
        ----------
        shape_type_code : int
            The type code of the shape to check.

        Returns
        -------
        bool
            True if it's possible to display the new shape, False
            otherwise.
        """
        if shape_type_code > (len(position_list) - 1):
            print("I don't know how to position that shape")
            found_space: bool = False
        else:
            found_space: bool = False
            position_list_index: int = 0

            # Loop through each position in the position_list
            # for the given shape_type_code
            while (not found_space) and (
                position_list_index < len(position_list[shape_type_code])
            ):
                # Check next position in position list for this shape
                [row_test, column_test] = position_list[shape_type_code][
                    position_list_index
                ]
                # Check if the position is available
                if np.isnan(self.shapes_drawn[row_test, column_test, 0]):
                    # Space is available
                    found_space = True
                else:
                    # Space is not available - keep looking
                    position_list_index += 1

        return found_space

    def index_of_location(self, location: List[float]) -> Tuple[int, int]:
        """
        Finds the row and column indices of the location in the grid.

        Parameters
        ----------
        location : List[float]
            The location to find the indices for as [x, y].

        Returns
        -------
        Tuple[int, int]
            The row and column indices of the location.
        """
        location = np.array(location)  # type: ignore
        location_cell = (location - shape_size / 2) / shape_size

        num_rows: int = self.shapes_drawn.shape[0]

        row: int = (num_rows - 1) - int(round(location_cell[1]))
        column: int = int(round(location_cell[0]))

        return row, column

    def shape_at_location(self, location: List[float]) -> Tuple[int, int]:
        """
        Finds the shape type code and shape ID at the given location.

        Parameters
        ----------
        location : List[float]
            The location to find the shape at as [x, y].

        Returns
        -------
        Tuple[int, int]
            The shape type code and shape ID at the location.
        """
        # Map a location to the shape drawn at that location.
        # Shape_type_code will be -1 if invalid location.
        # Shape_id will be -1 if no shape present at location.
        [row, column] = self.index_of_location(location)

        num_rows = self.shapes_drawn.shape[0]
        num_columns = self.shapes_drawn.shape[1]

        if row > (num_rows - 1) or row < 0:
            # If the row is out of range, print an error message
            # and set shape_type_code and shape_id to -1 and 0
            print("Invalid row")
            shape_type_code = -1
            shape_id = 0
        elif column > (num_columns - 1) or column < 0:
            # Same as previous comment, but for column
            print("Invalid columnumn")
            shape_type_code = -1
            shape_id = 0
        else:
            # Get shape code and shape id at the given location
            shape_type_code = self.shapes_drawn[row, column, 0]
            shape_id = self.shapes_drawn[row, column, 1]

            if np.isnan(shape_id):  # nothing is there
                shape_id = -1
                shape_type_code = 0
            else:
                shape_type_code = int(shape_type_code)
                shape_id = int(shape_id)

        return shape_type_code, shape_id

    def closest_shapes_to_location(
        self, location: List[float]
    ) -> Tuple[List[int], List[int]]:
        """
        Map a location to the closest shape(s) drawn at that location.
        If multiple shapes are adjacent to the location, all will be
        returned.
        shape_type_code will be -1 if invalid location.
        shape_id will be -1 if no shapes have been drawn.

        Parameters
        ----------
        location : List[float]
            The location to find the closest shape(s) to as [x, y].

        Returns
        -------
        Tuple[List[int], List[int]]
            A tuple containing two lists: the first list contains the
            shape type codes of the closest shapes, and the second list
            contains the shape IDs of the closest shapes.
        """
        [row, column] = self.index_of_location(location)

        num_rows = self.shapes_drawn.shape[0]
        num_columns = self.shapes_drawn.shape[1]

        if row > (num_rows - 1) or row < 0:
            print("Invalid row")
            shape_type_code = [-1]
            shape_id = [0]
        elif column > (num_columns - 1) or column < 0:
            print("Invalid columnumn")
            shape_type_code = [-1]
            shape_id = [0]
        elif np.all(np.isnan(self.shapes_drawn[:, :, 0])):
            print("No shapes drawn yet")
            shape_id = [-1]
            shape_type_code = [0]
        else:
            # Calculate distance of all drawn shapes from the given location
            shape_indexes = np.argwhere(np.isfinite(self.shapes_drawn[:, :, 0]))
            num_shapes_drawn = shape_indexes.shape[0]
            dists_to_location = np.zeros(num_shapes_drawn)
            for i in range(num_shapes_drawn):
                dists_to_location[i] = np.sqrt(
                    (row - shape_indexes[i, 0]) ** 2
                    + (column - shape_indexes[i, 1]) ** 2
                )

            closest_indexes = np.argsort(dists_to_location)

            i = 0
            shape_type_code = []
            shape_id = []
            # Return all of the shapes which are closest if there
            # are multiple at the same distance
            while (
                i < len(closest_indexes)
                and dists_to_location[closest_indexes[i]]
                == dists_to_location[closest_indexes[0]]
            ):
                next_closest_row = shape_indexes[closest_indexes[i], 0]
                next_closest_column = shape_indexes[closest_indexes[i], 1]
                shape_type_code.append(
                    self.shapes_drawn[next_closest_row, next_closest_column, 0]
                )
                shape_id.append(
                    self.shapes_drawn[next_closest_row, next_closest_column, 1]
                )
                i += 1

        return shape_type_code, shape_id

    def display_shape_at_location(
        self, shape_type_code: int, location: List[float]
    ) -> bool:
        """
        Displays a shape with the given shape_type_code at the
        specified location.

        Parameters
        ----------
        shape_type_code : int
            The type code of the shape to display.
        location : List[float]
            The location to display the shape at as [x, y].

        Returns
        -------
        bool
            True if the shape was successfully displayed, False
            otherwise.
        """
        [row, column] = self.index_of_location(location)
        num_rows: int = self.shapes_drawn.shape[0]
        num_columns: int = self.shapes_drawn.shape[1]

        if row > (num_rows - 1) or row < 0:
            print("Invalid row")
            success: bool = False
        elif column > (num_columns - 1) or column < 0:
            print("Invalid columnumn")
            success: bool = False
        else:
            # Set the shape type code and shape id at the given location
            # and set success to True
            shape_id = np.equal(self.shapes_drawn[:, :, 0], shape_type_code).sum()
            self.shapes_drawn[row, column, 0] = shape_type_code
            self.shapes_drawn[row, column, 1] = shape_id
            success = True

        return success
