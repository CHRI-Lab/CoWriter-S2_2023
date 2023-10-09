from flask import Flask, request, jsonify, current_app
from flask_cors import CORS

import rclpy
from rclpy.executors import MultiThreadedExecutor

from threading import Thread

from .manager_ui_bridge import ManagerUIBridge
from .child_ui_bridge import ChildUIBridge

TOPIC_WORDS_TO_WRITE = "words_to_write"


app = Flask(__name__)
cors_config = {
    "origins": ["*"],
    "methods": ["GET", "POST", "PUT", "DELETE"],
    "allow_headers": ["Content-Type", "Authorization"],
}

# Overlook CORS error when sending msg back to browser
CORS(app, resources={r"*": cors_config})


@app.route("/words_to_write", methods=["GET", "POST"])
def words_to_write():
    if request.method == "POST":
        data = request.get_json()
        current_app.node.get_logger().info(str(data))
        return jsonify(data)
    else:
        return jsonify({"words": ["hello", "world"]})


def main():
    rclpy.init()

    manager_bridge = ManagerUIBridge()
    child_bridge = ChildUIBridge()

    executor = MultiThreadedExecutor()
    executor.add_node(manager_bridge)
    executor.add_node(child_bridge)

    thread = Thread(target=executor.spin)
    thread.start()

    app.manager_bridge = manager_bridge
    app.child_bridge = child_bridge

    try:
        app.run(host="0.0.0.0", port=3001, debug=True)
    finally:
        manager_bridge.get_logger().info("Shutting down ROS2 Node . . .")
        manager_bridge.destroy_node()
        child_bridge.get_logger().info("Shutting down ROS2 Node . . .")
        child_bridge.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()
