import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

from speech_recognition.include.audio_stream import AudioStream

import os

class SpeechRecognition(Node):
    def __init__(self):
        super().__init__("speech_recognition")

        channels = self.declare_parameter("channels", 1).value
        depth = self.declare_parameter("depth", 16).value
        sample_rate = self.declare_parameter("sample_rate", 16000).value

        min_silent_chunk_to_split = self.declare_parameter(
            "min_silent_chunk_to_split", 100
        ).value
        audio_buflen = self.declare_parameter("audio_buflen", 10240).value
        # silent_threshold = self.declare_parameter("silent_threshold", 700).value
        silent_threshold = int(os.getenv("SILENT_THRESHOLD"))

        log_audio_to_file = self.declare_parameter(
            "log_audio_to_file", False
        ).value
        audio_outfile = self.declare_parameter(
            "audio_outfile", "audio_log.wav"
        ).value

        transcript_topic = self.declare_parameter(
            "transcript_topic", "speech_rec"
        ).value
        listening_signal_topic = self.declare_parameter(
            "listening_signal_topic", "listening_signal"
        ).value
        audio_topic = self.declare_parameter("audio_topic", "audio").value

        self.get_logger().info(f"channels: {channels}")
        self.get_logger().info(f"depth: {depth}")
        self.get_logger().info(f"sample_rate: {sample_rate}")
        self.get_logger().info(
            f"min_silent_chunk_to_split: {min_silent_chunk_to_split}"
        )
        self.get_logger().info(f"audio_buflen: {audio_buflen}")
        self.get_logger().info(f"silent_threshold: {silent_threshold}")
        self.get_logger().info(f"log_audio_to_file: {log_audio_to_file}")
        self.get_logger().info(f"audio_outfile: {audio_outfile}")
        self.get_logger().info(f"transcript_topic: {transcript_topic}")
        self.get_logger().info(
            f"listening_signal_topic: {listening_signal_topic}"
        )
        self.get_logger().info(f"audio_topic: {audio_topic}")

        self.audio_stream = AudioStream(
            channels,
            depth,
            sample_rate,
            min_silent_chunk_to_split,
            audio_buflen,
            silent_threshold,
            log_audio_to_file,
            audio_outfile,
            self.transcript_callback,
            self.get_logger(),
        )

        self.transcript_publisher = self.create_publisher(
            String, transcript_topic, 10
        )
        self.create_subscription(
            String,
            listening_signal_topic,
            self.audio_stream.setListening,
            10,
        )
        self.create_subscription(
            AudioData, audio_topic, self.audio_stream.audio_callback, 10
        )

    def transcript_callback(self, text, word_to_write=False):
        self.transcript_publisher.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    speech_recognition = SpeechRecognition()
    rclpy.spin(speech_recognition)
    speech_recognition.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
