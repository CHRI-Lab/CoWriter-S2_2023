#!/usr/bin/env python3
"""
This module provides a ROS node for managing the display of shapes, as
well as several handler functions for the services provided by this
node.
"""

import os
import rospy
import sys

from letter_learning_interaction.srv import *

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../include'))
from shape_display_manager import ShapeDisplayManager

# Being used as a global variable, should refactor to a class
# Not doing now to minimise knock on changes
# Not defined in __main__ so can test node with rostest
shape_display_manager = ShapeDisplayManager()

def handle_clear_all_shapes(request: ClearAllShapesRequest) -> ClearAllShapesResponse:
    """
    Handler function for the `clear_all_shapes` service.

    Clears all shapes from the ShapeDisplayManager.

    Parameters:
    - request (ClearAllShapesRequest): The request object for the
                                       service.

    Returns:
    - response (ClearAllShapesResponse): The response object for the
                                         service.
    """
    shape_display_manager.clear_all_shapes()
    rospy.loginfo('Shapes cleared')
    response = ClearAllShapesResponse()
    response.success.data = True

    return response


def handle_display_new_shape(request: DisplayNewShapeRequest) -> DisplayNewShapeResponse:
    """
    Handler function for the `display_new_shape` service.

    Displays a new shape of the given shape_type_code and returns the
    location of the new shape.

    Parameters:
    - request (DisplayNewShapeRequest): The request object for the
                                        service.

    Returns:
    - response (DisplayNewShapeResponse): The response object for the
                                          service.
    """
    location = shape_display_manager.display_new_shape(request.shape_type_code)
    response = DisplayNewShapeResponse()
    response.location.x = location[0]
    response.location.y = location[1]
    rospy.loginfo(f'Shape added at {location}')

    return response


def handle_index_of_location(request: IndexOfLocationRequest) -> IndexOfLocationResponse:
    """
    Handler function for the `index_of_location` service.

    Returns the row and column indices corresponding to the given
    location.

    Parameters:
    - request (IndexOfLocationRequest): The request object for the
                                        service.

    Returns:
    - response (IndexOfLocationResponse): The response object for the
                                          service.
    """
    location = [request.location.x, request.location.y]
    row, column = shape_display_manager.index_of_location(location)
    response = IndexOfLocationResponse()
    response.row = row
    response.column = column
    rospy.loginfo(f'Index returned: {response.row}, {response.column}')

    return response


def handle_shape_at_location(request: ShapeAtLocationRequest) -> ShapeAtLocationResponse:
    """
    Handler function for the `shape_at_location` service.

    Returns the shape type code and shape ID for the shape at the given
    location.

    Parameters:
    - request (ShapeAtLocationRequest): The request object for the
                                        service.

    Returns:
    - response (ShapeAtLocationResponse): The response object for the
                                          service.
    """
    location = [request.location.x, request.location.y]
    response = ShapeAtLocationResponse()
    shape_type_code, shape_id = shape_display_manager.shape_at_location(location)
    response.shape_type_code = shape_type_code
    response.shape_id = shape_id
    rospy.loginfo(
        f'Shape at location returned: {response.shape_type_code}_{response.shape_id}')

    return response


def handle_closest_shapes_to_location(request: ClosestShapesToLocationRequest) -> ClosestShapesToLocationResponse:
    """
    Handler function for the `closest_shapes_to_location` service.

    Returns the shape type codes and shape IDs for the closest shapes
    to the given location.

    Parameters:
    - request (ClosestShapesToLocationRequest): The request object for
                                                the service.

    Returns:
    - response (ClosestShapesToLocationResponse):
        The response object for the service.
    """
    location = [request.location.x, request.location.y]
    response = ClosestShapesToLocationResponse()
    [response.shape_type_code,
        response.shape_id] = shape_display_manager.closest_shapes_to_location(location)
    rospy.loginfo(
        f'Closest shape(s) to location returned: {response.shape_type_code}_{response.shape_id}')

    return response


def handle_possible_to_display(request) -> IsPossibleToDisplayNewShapeResponse:
    """
    Handle the service request for determining if a new shape of a
    given type can be displayed given the current state of the
    displayed shapes.

    Args:
        request: A object containing the shape type
            code for the new shape.

    Returns:
        An IsPossibleToDisplayNewShapeResponse object indicating
        whether it is possible to display the new shape.
    """
    response = IsPossibleToDisplayNewShapeResponse()
    response.is_possible.data = shape_display_manager.is_possible_to_display_new_shape(
        request.shape_type_code)
    rospy.loginfo(f'If possible returned {response.is_possible.data}')

    return response


def handle_display_shape_at_location(request: DisplayShapeAtLocationRequest) -> DisplayShapeAtLocationResponse:
    """
    Handle the service request for displaying a new shape of a given
    type at a specified location.

    Args:
        request: A DisplayShapeAtLocationRequest object containing the
                 shape type code for the new shape and the location at
                 which to display it.

    Returns:
        A DisplayShapeAtLocationResponse object indicating whether the
        shape was successfully displayed.
    """
    response = DisplayShapeAtLocationResponse()
    location = [request.location.x, request.location.y]

    response.success.data = shape_display_manager.display_shape_at_location(
        request.shape_type_code, location)
    rospy.loginfo(f'Shape added at : {location}')

    return response


def display_manager_server() -> None:
    """
    Set up the ROS node and services for the shape display manager
    server, and handle requests received by calling the appropriate
    handler functions.
    """
    rospy.init_node('display_manager_server')
    clear_service = rospy.Service(
        'clear_all_shapes', ClearAllShapes, handle_clear_all_shapes)
    rospy.loginfo("Ready to clear all shapes.")

    display_shape_service = rospy.Service(
        'display_new_shape', DisplayNewShape, handle_display_new_shape)
    rospy.loginfo("Ready to display new shapes.")

    index_of_location_service = rospy.Service(
        'index_of_location', IndexOfLocation, handle_index_of_location)
    rospy.loginfo("Ready to determine index of location.")

    shape_at_location_service = rospy.Service(
        'shape_at_location', ShapeAtLocation, handle_shape_at_location)
    rospy.loginfo("Ready to determine shape at location.")

    closest_shapes_to_location_service = rospy.Service(
        'closest_shapes_to_location', ClosestShapesToLocation, handle_closest_shapes_to_location)
    rospy.loginfo("Ready to determine closest shape(s) to location.")

    display_shape_at_location_service = rospy.Service(
        'display_shape_at_location', DisplayShapeAtLocation, handle_display_shape_at_location)
    rospy.loginfo("Ready to display new shapes at specific location.")

    possible_to_display_service = rospy.Service(
        'possible_to_display_shape', IsPossibleToDisplayNewShape, handle_possible_to_display)
    rospy.loginfo("Ready to determine is shape fits.")
    rospy.spin()

if __name__ == "__main__":
    display_manager_server()
    rospy.loginfo('shut down')
