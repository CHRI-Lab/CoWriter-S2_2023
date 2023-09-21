## Migration from ros1 to ros2 (python)

### CMakeLists.txt and package.xml files

https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html
voir notes module AWS
en gros catkin -> ament

### Convert python nodes

No more rospy, need to convert it to rclpy

```python
# init nodes
import rclpy
from rclpy.node import Node

rclpy.init()
node = Node('node_name')



#Creating a Publisher:

#rospy:
pub = rospy.Publisher('topic_name', MessageType, queue_size=10)
#rclpy
pub = node.create_publisher(MessageType, 'topic_name', 10)

Creating a Subscriber:

rospy: rospy.Subscriber('topic_name', MessageType, callback)
rclpy: subscription = node.create_subscription(MessageType, 'topic_name', callback, 10)


old
def handle_service(request):
    return Response()

service = rospy.Service('service_name', ServiceType, handle_service)


new
def handle_service(request, response):
    return response

service = node.create_service(ServiceType, 'service_name', handle_service)


old
rospy.wait_for_service('service_name')
client = rospy.ServiceProxy('service_name', ServiceType)
response = client(request)

new
client = node.create_client(ServiceType, 'service_name')
while not client.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('service not available, waiting again...')
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
response = future.result()


# rate loops
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    # do something
    rate.sleep()

while rclpy.ok():
    # do something
    rclpy.spin_once(node, timeout_sec=0.1)


Logging:

rospy: rospy.loginfo("This is an info message")
rclpy: node.get_logger().info('This is an info message')

```

at each end of a node file, create a __main__ that contains the node declaration


distinction create_service and create_client (to the service)


https://roboticsbackend.com/ros2-global-parameters/#Launch_file
global params ros2



https://docs.ros.org/en/crystal/Tutorials/Custom-ROS2-Interfaces.html


https://docs.ros.org/en/foxy/How-To-Guides/Developing-a-ROS-2-Package.html


stucture d'un package ros2 -> note sur le resource/,,,
les nodes qui doivent contenir des fonctions main que l'on retrouve dans le fichier setup.py



top level launch file
https://daobook.github.io/ros2-docs/foxy/Tutorials/Launch-Files/Using-ROS2-Launch-For-Large-Projects.html
puis ros2 launch src/launch_all_nodes.py 