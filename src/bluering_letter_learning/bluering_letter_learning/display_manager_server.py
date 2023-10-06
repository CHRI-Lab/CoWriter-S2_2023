#!/usr/bin/env python
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
from bluering_letter_learning.include.utils.shape_display_manager import (
    ShapeDisplayManager,
)


class DisplayManagerServer(Node):
    def __init__(self):
        super().__init__("display_manager_server")

        self.shapeDisplayManager = ShapeDisplayManager()

        self.create_service(
            ClearAllShapes, "clear_all_shapes", self.handle_clear_all_shapes
        )
        self.get_logger().info("Ready to clear all shapes.")

        self.create_service(
            DisplayNewShape, "display_new_shape", self.handle_display_new_shape
        )
        self.get_logger().info("Ready to display new shapes.")

        self.create_service(
            IndexOfLocation,
            "index_of_location",
            self.handle_index_of_location,
        )
        self.get_logger().info("Ready to determine index of location.")

        self.create_service(
            ShapeAtLocation, "shape_at_location", self.handle_shape_at_location
        )
        self.get_logger().info("Ready to determine shape at location.")

        self.create_service(
            ClosestShapesToLocation,
            "closest_shapes_to_location",
            self.handle_closest_shapes_to_location,
        )
        self.get_logger().info(
            "Ready to determine closest shape(s) to location."
        )

        self.create_service(
            DisplayShapeAtLocation,
            "display_shape_at_location",
            self.handle_display_shape_at_location,
        )
        self.get_logger().info(
            "Ready to display new shapes at specific location."
        )

        self.create_service(
            IsPossibleToDisplayNewShape,
            "possible_to_display_shape",
            self.handle_possible_to_display,
        )
        self.get_logger().info("Ready to determine is shape fits.")

    def handle_clear_all_shapes(self, request, response):
        self.shapeDisplayManager.clearAllShapes()
        self.get_logger().info("Shapes cleared")
        response.success.data = True  # probably not necessary
        return response

    def handle_display_new_shape(self, request, response):
        location = self.shapeDisplayManager.displayNewShape(
            request.shape_type_code
        )
        response.location.x = location[0]
        response.location.y = location[1]

        self.get_logger().info("Shape added at " + str(location))
        return response

    def handle_index_of_location(self, request, response):
        location = [request.location.x, request.location.y]
        [
            response.row,
            response.column,
        ] = self.shapeDisplayManager.indexOfLocation(location)
        self.get_logger().info(
            "Index returned: " + str(response.row) + ", " + str(response.column)
        )
        return response

    def handle_shape_at_location(self, request, response):
        location = [request.location.x, request.location.y]
        [
            response.shape_type_code,
            response.shape_id,
        ] = self.shapeDisplayManager.shapeAtLocation(location)
        self.get_logger().info(
            "Shape at location returned: "
            + str(response.shape_type_code)
            + "_"
            + str(response.shape_id)
        )
        return response

    def handle_closest_shapes_to_location(self, request, response):
        location = [request.location.x, request.location.y]
        [
            response.shape_type_code,
            response.shape_id,
        ] = self.shapeDisplayManager.closestShapesToLocation(location)
        self.get_logger().info(
            "Closest shape(s) to location returned: "
            + str(response.shape_type_code)
            + "_"
            + str(response.shape_id)
        )
        return response

    def handle_possible_to_display(self, request, response):
        response.is_possible.data = (
            self.shapeDisplayManager.isPossibleToDisplayNewShape(
                request.shape_type_code
            )
        )
        self.get_logger().info(
            "If possible returned " + str(response.is_possible.data)
        )
        return response

    def handle_display_shape_at_location(self, request, response):
        location = [request.location.x, request.location.y]
        response.success.data = self.shapeDisplayManager.displayShapeAtLocation(
            request.shape_type_code, location
        )
        self.get_logger().info("Shape added at :" + str(location))
        return response


def main(args=None):
    rclpy.init(args=args)
    dms = DisplayManagerServer()
    rclpy.spin(dms)
    dms.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
