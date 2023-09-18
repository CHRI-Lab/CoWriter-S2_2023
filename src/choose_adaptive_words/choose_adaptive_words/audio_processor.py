#!/usr/bin/env python3

import os
import time
import sounddevice as sd
import whisper
from scipy.io.wavfile import write
from rclpy.node import Node


class SpeechTranscriber:
    def __init__(self, language):
        self.model = whisper.load_model("tiny.en")
        self.language = language

    def transcribe_audio(self, audio_file):
        """
        Perform preprocessing and transcribes an audio file using OpenAI
        Whisper, returning the transcribed text.
        """
        return self.model.transcribe(audio_file)


class AudioProcessor:
    """
    A class that controls the process of recording audio
    from the device's microphone, transcribing it into text
    using the Whisper ASR model, and publishing the transcriptions to a
    ROS topic.

    The recording is performed in chunks of a specified duration
    (default 7 seconds). After each chunk is recorded, it is processed
    and the resulting transcription is published to the '/speech_rec'
    ROS topic if it's non-empty and unchanged from the previous chunk's
    transcription.

    This class is designed to work in conjunction with a 'StateManager'
    which subscribes to the '/speech_rec' ROS topic and uses the
    incoming transcriptions to guide the robot's states.

    Attributes:
    - language: The language expected in the audio to be transcribed.
    - audio_channel: Channel to record audio.
    - sampling_freq: Sample rate for audio (Hz).
    - record_file: Path to the file where the recorded audio is saved.
    - speech_transcriber: An instance of the SpeechTranscriber class
                          for audio transcription.
    - transcription: The transcription of the latest chunk of audio.
    - prev_transcription: The transcription of the previous chunk of
                          audio.
    - is_recording: A boolean indicating whether the audio recording
                    is currently active.
    - stop_process: A boolean indicating whether to stop the
                    recording-transcribing process.
    - chunk_duration: The duration of each chunk of audio to be
                      recorded (in seconds).
    - transcription_publisher: A ROS Publisher object for publishing
                               the transcriptions to a ROS topic.
    """

    def __init__(self, language, ros_node: Node):
        super().__init__()
        self.ros_node_logger = ros_node.get_logger()
        self.language: str = language
        self.record_file = os.getcwd() + "/record.wav"
        self.speech_transcriber = SpeechTranscriber(self.language)
        self.transcription: str = ""
        self.prev_transcription: str = ""
        self.is_recording: bool = False
        self.stop_process: bool = False
        self.chunk_duration = 7  # seconds

        self.sampling_freq = 44100  # hz
        self.audio_channel = 2

    def run(self):
        """
        Run on a loop, recording audio and processing it.
        """
        self.ros_node_logger("Starting start_recording().")
        self.start_recording()
        # Record for chunk_duration seconds
        time.sleep(self.chunk_duration)
        self.ros_node_logger("Finished start_recording().")
        self.ros_node_logger("Starting stop_recording()")
        self.stop_recording()
        self.ros_node_logger("Finished stop_recording()")
        self.ros_node_logger("Starting process_audio()")
        self.process_audio()
        self.ros_node_logger("Finished process_audio()")

    def start_recording(self):
        """
        Start recording audio.
        """
        self.record_raw = sd.rec(
            int(self.chunk_duration * self.sampling_freq),
            samplerate=self.sampling_freq,
            channels=self.audio_channel,
            blocking=False,
        )
        self.is_recording = True

    def stop_recording(self):
        """
        Stop recording audio.
        """
        write(self.record_file, self.sampling_freq, self.record_raw)
        self.is_recording = False

    def process_audio(self):
        """
        Process recorded audio and publish the transcription.
        """
        transcription_chunk: str = self.speech_transcriber.transcribe_audio(
            self.record_file
        )["text"]
        self.ros_node_logger("Transcription is %s" % transcription_chunk)
        self.transcription += transcription_chunk

        # If transcription is non-empty & unchanged, publish and reset
        if self.transcription and self.transcription == self.prev_transcription:
            # Reset self.transcription
            self.transcription = ""

        self.prev_transcription = self.transcription

    def stop(self):
        """
        Stops the audio recording and processing thread.
        """
        self.stop_process = True
