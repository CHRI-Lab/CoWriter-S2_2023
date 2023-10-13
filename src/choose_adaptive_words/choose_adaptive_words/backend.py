import rclpy
from rclpy.executors import MultiThreadedExecutor

from threading import Thread

from .manager_ui_bridge import ManagerUIBridge
from .child_ui_bridge import ChildUIBridge
from .app import create_app

TOPIC_WORDS_TO_WRITE = "words_to_write"

# TODO @Difan
def main():
    rclpy.init()

    manager_bridge = ManagerUIBridge()
    #child_bridge = ChildUIBridge()

    executor = MultiThreadedExecutor()
    executor.add_node(manager_bridge)
    #executor.add_node(child_bridge)

    thread = Thread(target=executor.spin)
    thread.start()

    app = create_app()
    app.manager_bridge = manager_bridge
    #app.child_bridge = child_bridge

    try:
        app.run(host="0.0.0.0", port=3001, debug=True)
    finally:
        manager_bridge.get_logger().info("Shutting down ROS2 Node . . .")
        manager_bridge.destroy_node()
        #child_bridge.get_logger().info("Shutting down ROS2 Node . . .")
        #child_bridge.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()
