#!/usr/bin/env python3
"""
This module provides a ROS node for managing the display of shapes, as
well as several handler functions for the services provided by this
node.
"""

from interface.srv import (
    ClearAllShapes,
    DisplayNewShape,
    IndexOfLocation,
    ShapeAtLocation,
    ClosestShapesToLocation,
    DisplayShapeAtLocation,
    IsPossibleToDisplayNewShape,
)

import rclpy
from rclpy.node import Node

from letter_learning_interaction.shape_display_manager import (
    ShapeDisplayManager,
)

# Being used as a global variable, should refactor to a class
# Not doing now to minimise knock on changes
# Not defined in __main__ so can test node with rostest
shape_display_manager = ShapeDisplayManager()


class DisplayManagerServer(Node):
    def __init__(self):
        super().__init__("display_manager_server")
        self.clear_all_shapes_service = self.create_service(
            ClearAllShapes, "clear_all_shapes", self.handle_clear_all_shapes
        )
        self.display_new_shape_service = self.create_service(
            DisplayNewShape, "display_new_shape", self.handle_display_new_shape
        )

        self.index_of_location_service = self.create_service(
            IndexOfLocation, "index_of_location", self.handle_index_of_location
        )

        self.shape_at_location_service = self.create_service(
            ShapeAtLocation, "shape_at_location", self.handle_shape_at_location
        )

        self.closest_shapes_to_location_service = self.create_service(
            ClosestShapesToLocation,
            "closest_shapes_to_location",
            self.handle_closest_shapes_to_location,
        )

        self.display_shape_at_location_service = self.create_service(
            DisplayShapeAtLocation,
            "display_shape_at_location",
            self.handle_display_shape_at_location,
        )

        self.possible_to_display_service = self.create_service(
            IsPossibleToDisplayNewShape,
            "possible_to_display_shape",
            self.handle_possible_to_display,
        )
        self.get_logger().info("Display manager server initialised")

    def handle_clear_all_shapes(self, request, response):
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
        self.get_logger().info("Shapes cleared")
        return response

    def handle_display_new_shape(self, request, response):
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
        response.location.x = location[0]
        response.location.y = location[1]
        self.get_logger.info(f"Shape added at {location}")
        return response

    def handle_index_of_location(self, request, response):
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
        response.row = row
        response.column = column
        self.get_logger.info(f"Index returned: {response.row}, {response.column}")
        return response

    def handle_shape_at_location(self, request, response):
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
        shape_type_code, shape_id = shape_display_manager.shape_at_location(location)
        response.shape_type_code = shape_type_code
        response.shape_id = shape_id
        self.get_logger.info(
            f"Shape at location returned: {response.shape_type_code}_{response.shape_id}"  # noqa: E501
        )
        return response

    def handle_closest_shapes_to_location(self, request, response):
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
        [
            response.shape_type_code,
            response.shape_id,
        ] = shape_display_manager.closest_shapes_to_location(location)
        self.get_logger.info(
            f"Closest shape(s) to location returned: {response.shape_type_code}_{response.shape_id}"  # noqa: E501
        )
        return response

    def handle_possible_to_display(self, request, response):
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
        response.is_possible.data = (
            shape_display_manager.is_possible_to_display_new_shape(
                request.shape_type_code
            )
        )
        self.get_logger.info(f"If possible returned {response.is_possible.data}")
        return response

    def handle_display_shape_at_location(self, request, response):
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
        location = [request.location.x, request.location.y]
        response.success.data = shape_display_manager.display_shape_at_location(
            request.shape_type_code, location
        )
        self.get_logger.info(f"Shape added at : {location}")
        return response


def main(args=None):
    rclpy.init(args=args)
    display_manager_server = DisplayManagerServer()
    display_manager_server.get_logger().info("Node up and running")
    rclpy.spin(display_manager_server)
    display_manager_server.get_logger().info("Node shutting down")
    display_manager_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
