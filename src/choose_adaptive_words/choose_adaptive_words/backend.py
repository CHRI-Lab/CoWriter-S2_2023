import rclpy
from rclpy.executors import MultiThreadedExecutor

from threading import Thread

from .manager_ui_bridge import ManagerUIBridge
from .child_ui_bridge import ChildUIBridge
from .app import create_app
from .temp_backend import UIBackend
from .strugg_letter import StruggLetterNode

TOPIC_WORDS_TO_WRITE = "words_to_write"


# TODO @Difan
def main():
    rclpy.init()

    manager_bridge = ManagerUIBridge()
    child_bridge = ChildUIBridge()
    ui_backend = UIBackend()
    strugg_backend = StruggLetterNode()

    executor = MultiThreadedExecutor()
    executor.add_node(manager_bridge)
    executor.add_node(child_bridge)
    executor.add_node(ui_backend)
    executor.add_node(strugg_backend)

    thread = Thread(target=executor.spin)
    thread.start()

    app = create_app()
    app.manager_bridge = manager_bridge
    app.strugg_backend = strugg_backend
    app.child_bridge = child_bridge

    try:
        app.run(host="0.0.0.0", port=3001, debug=True)
    finally:
        manager_bridge.get_logger().info("Shutting down ROS2 Node . . .")
        manager_bridge.destroy_node()
        child_bridge.get_logger().info("Shutting down ROS2 Node . . .")
        child_bridge.destroy_node()
        ui_backend.destroy_node()
        strugg_backend.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()
