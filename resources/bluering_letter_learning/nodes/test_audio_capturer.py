#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import wave
import numpy as np
from audio_common_msgs.msg import AudioData
import copy

from google.cloud import speech_v1p1beta1 as speech
import re
import sys


class GCS():
    def __init__(self, n_channel=1, sample_rate=16000):
        super().__init__()
        #? Set up the Speech-to-Text client
        self.client = speech.SpeechClient()

        #? Set up the transcription configuration
        self.config = speech.RecognitionConfig(
            # encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            language_code='en-US',
            enable_word_time_offsets=True,

        )
        self.config.sample_rate_hertz = sample_rate
        self.config.audio_channel_count = n_channel
        # self.streaming_config = speech.StreamingRecognitionConfig(
        #     config=self.config, interim_results=True
        # )
        self.streaming_config = speech.StreamingRecognitionConfig(
            config=self.config
        )

    def recognize(self, content: bytes) -> str:
        """
        Perform the transcription
        """
        #? fill in header: https://docs.fileformat.com/audio/wav/
        # header_hex = '52 49 46 46 24 00 FF 7F 57 41 56 45 66 6D 74 20 10 00 00 00 01 00 02 00 44 AC 00 00 10 B1 02 00 04 00 10 00 64 61 74 61 00 00 FF 7F'
        # header_bytes = bytes.fromhex(header_hex)
        # content = header_bytes + content
        
        # print(f'[ ][recognize] content : {content}')
        audio = speech.RecognitionAudio(content=content)
        # print(f'   [recognize] audio : {audio}')
        response = self.client.recognize(config=self.config, audio=audio)
        print(f'   [recognize] response : {response}')
        open('/home/ubuntu/catkin_ws/test.wav', 'wb').write(content)

        total_str = ''
        for result in response.results:
            for word in result.alternatives[0].words:
                total_str += word.word
        return total_str
    



_gcs_ = GCS(1, 16000)
_buf_ = b''


# Set the name of the ROS topic that contains the audio data
AUDIO_TOPIC = '/audio/audio'

# Set the name of the output wave file
OUTPUT_FILE = '/home/ubuntu/catkin_ws/v3.wav'

# Create a new wave file for writing
wf = wave.open(OUTPUT_FILE, 'wb')


SAMPLE_RATE = 16000
SAMPLE_WIDTH = 2
NCHANNELS = 1

# Set the wave file parameters based on the audio format
wf.setnchannels(NCHANNELS)  # Mono audio
wf.setsampwidth(SAMPLE_WIDTH)  # 16-bit audio
wf.setframerate(SAMPLE_RATE)  # Sample rate of 16 kHz


# Define a callback function to handle incoming audio data
def audio_callback(data):
    global _buf_

    # Convert the audio data to a NumPy array
    audio_data = np.frombuffer(data.data, dtype=np.int16)

    audio_bytes = audio_data.tobytes()
    # Write the audio data to the wave file
    wf.writeframes(audio_bytes)

    _buf_ += audio_bytes
    print(f'len buf: {len(_buf_)}')
    if len(_buf_) > 124160:
        content = copy.deepcopy(_buf_)
        _buf_ = b''
        # s2t
        print(f'processing buf content : {len(content)}')
        txt = _gcs_.recognize(content)
        print(f'txt : {txt}')



# Subscribe to the audio topic and start the ROS node
rospy.init_node('speech_to_text')
rospy.Subscriber(AUDIO_TOPIC, AudioData, audio_callback)
rospy.spin()

# Close the wave file
wf.close()
