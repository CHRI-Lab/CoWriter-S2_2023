import socket
import json
import struct

# HOST = '172.25.96.1'    # IP address of the server
# PORT = 12345
#
# data = {'name': 'Nao', 'age': 6}
# json_data = json.dumps(data).encode('utf-8')
#
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP
# s.connect((HOST, PORT))
# s.sendall(json_data)
# s.close()

import sys

import rclpy
import threading
from rclpy.node import Node
from interface.msg import *
from interface.srv import *


class NaoWriterNaoqi(Node):

    def __init__(self):
        super().__init__('nao_writer_naoqi')
        self.cli = self.create_client(LearnShape, 'learn_shape')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.learn_shape_req = LearnShape.Request()

        self.subscription = self.create_subscription(
            Strokes,
            'strokesMessage',
            self.strokes_message_callback, 10)

        # add a socket object to the node
        self.__host = '127.0.0.1'
        self.__port = 12345
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP
        self.connect_socket()

    def connect_socket(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # TCP
        self.socket.connect((self.__host, self.__port))

    def send_request(self, strokes):
        self.get_logger().info('sending request.....')
        self.learn_shape_req.strokes = strokes
        self.future = self.cli.call_async(self.learn_shape_req)
        # rclpy.spin_until_future_complete(self, self.future)  # dont use this, will cause deadlock with rclpy.spin
        self.future.add_done_callback(self.future_callback)
        # return self.future.result()

    # def future_callback(self, future):
    #     if future.done():
    #         try:
    #             response = future.result()
    #             # self.get_logger().info('I get response: "%s"' % response.shape.path)
    #             try:
    #                 self.get_logger().info('sending action....')
    #                 self.socket.sendall(json.dumps({"DRAW": response.shape.path.tolist()}).encode('utf-8'))
    #                 data = self.socket.recv(1024)
    #                 self.get_logger().info(f'Received from server: {data.decode()}')
    #             except (ConnectionResetError, BrokenPipeError):
    #                 self.get_logger().error('Socket connection broken, reconnecting...')
    #                 self.connect_socket()
    #                 self.socket.sendall(json.dumps({"DRAW": response.shape.path.tolist()}).encode('utf-8'))
    #
    #         #     self.socket.sendall(json.dumps({"DRAW": response.shape.path.tolist()}).encode('utf-8'))
    #         #     # Receive data from the server
    #         #     data = self.socket.recv(1024)  # Receive up to 1024 bytes
    #         #     print('Received from server:', data.decode())
    #         except Exception as e:
    #             self.get_logger().error('Service call failed %r' % (e,))

    def future_callback(self, future):
        if future.done():
            try:
                response = future.result()

                # Preparing the message to be sent
                message = json.dumps({"DRAW": response.shape.path.tolist()}).encode('utf-8')
                message_length = struct.pack('>I', len(message))

                try:
                    self.get_logger().info('sending action....')

                    # Send the length of the message as a prefix
                    self.socket.sendall(message_length)

                    # Then send the message itself
                    self.socket.sendall(message)

                    data = self.socket.recv(1024)
                    self.get_logger().info(f'Received from server: {data.decode()}')

                except (ConnectionResetError, BrokenPipeError):
                    self.get_logger().error('Socket connection broken, reconnecting...')
                    self.connect_socket()

                    # Remember to send the length prefix when reconnecting
                    self.socket.sendall(message_length)
                    self.socket.sendall(message)

            except Exception as e:
                self.get_logger().error('Service call failed %r' % (e,))

    def strokes_message_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.shape_type)
        self.send_request(msg)


def main():
    rclpy.init()

    nao_writer_naoqi = NaoWriterNaoqi()

    rclpy.spin(nao_writer_naoqi)

    nao_writer_naoqi.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
