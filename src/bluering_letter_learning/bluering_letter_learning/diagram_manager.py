#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import wave
import numpy as np
import copy
import time
import openai
from std_msgs.msg import String, Bool
from audio_common_msgs.msg import AudioData
from bluering_letter_learning.msg import ActionToDo as ActionToDoMsg
from bluering_letter_learning.speech_to_text.google_cloud import GCS
from recordtype import recordtype
import io
#import whisper
# model = whisper.load_model("base")

ActionToDo = recordtype('ActionToDo', [('action_type', None), ('data', None)])
COMMANDS = {
    'change_word': ['change word'], 
    'login': ['old user', 'old child'], 
    'register': ['new user', 'new child'],
}
MIN_SILENT_CHUNK_TO_SPLIT = rospy.get_param('min_silent_chunk_to_split', 100)

class StaticSilenceDetector(object):
    """
    Silence detection on chunks of sound based on static threshold
    """

    is_static = True

    def __init__(self, rate, threshold):
        self.rate = rate
        self.threshold = threshold

    def is_silent(self, snd_data) -> bool:
        """
        Returns 'True' if all the data is below the 'silent' threshold.
        """
        # rospy.loginfo(f'[ ][is_silent] np.abs(snd_data).max() : {np.abs(snd_data).max()}  |  .mean() : {np.abs(snd_data).mean()}  |  self.threshold : {self.threshold}')
        # return True if np.abs(snd_data).max() < self.threshold else False
        return True if np.abs(snd_data).mean() < self.threshold else False




class ROSAudioStream(object):
    def __init__(self, finish_transcribe_cb) -> None:
        #? Set the name of the ROS topic that contains the audio data
        self.audio_topic = rospy.get_param('~audio_topic', 'audio')

        #? Do we want to log audio to file
        self.log_audio_to_file = rospy.get_param('~log_audio_to_file', False)
        if self.log_audio_to_file is True:
            #? Set the name of the output wave file if we do want to log audio to file
            self.audio_outfile = rospy.get_param('~audio_outfile', '/tmp/test.wav')
            #? Create audio log base dir if not existed
            outdir = os.path.dirname(self.audio_outfile)
            if not os.path.isdir(outdir):
                os.makedirs(outdir)
            #? Create a new wave file for writing
            self.wf = wave.open(self.audio_outfile, 'wb')

        self.sample_rate = rospy.get_param('~sample_rate', 16000)
        self.n_channels = rospy.get_param('~n_channels', 1)
        depth = rospy.get_param('~depth', 16)

        if depth == 8:
            self.sample_width = 1
        elif depth == 16:
            self.sample_width = 2
        elif depth == 32:
            self.sample_width = 4
        else:
            raise ValueError('depth must be 8, 16 or 32')

        #? Set the wave file parameters based on the audio format
        if self.log_audio_to_file is True:
            self.wf.setnchannels(self.n_channels)  #? Mono audio
            self.wf.setsampwidth(self.sample_width)  #? 16-bit audio
            self.wf.setframerate(self.sample_rate)  #? Sample rate of 16 kHz

        #? listening or not, when the robot speaking, we should stop listening
        self.listening = False
        self.listening_word_to_write = False
        LISTENING_SIGNAL_TOPIC = rospy.get_param('~listening_signal_topic', 'listening_signal')
        rospy.Subscriber(LISTENING_SIGNAL_TOPIC, String, self.setListening)


        #? Speech to text class
        self.gcs = GCS(self.n_channels, self.sample_rate)
        self.buf = b''
        self.first_silent_chunks = [b'', b''] #? store the 2 silent chunks right before the first detected sound, will prepend this silent_chunk to the audio buffer so that google cloud doesn't miss the first word
        #? callback on finishing transcribe
        self.finish_transcribe_cb = finish_transcribe_cb

        #? minimum buffer len to store audio data before transcribe (use dynamic buffer by detecting silence instead of fixed buflen)
        self.audio_buflen = rospy.get_param('~audio_buflen', 10240)
        #? keep track of continous silent chunk
        self.n_cont_silent_chunk = 0
        silent_threshold = rospy.get_param('~silent_threshold', 700)
        self.silence_detect = StaticSilenceDetector(self.sample_rate, silent_threshold)

    def setListening(self, data: String):
        data = data.data.lower()
        rospy.loginfo(f'[ ][ROSAudioStream][setListening] data = {data}')
        # self.listening = data
        if data == 'false':
            self.listening = False
            self.listening_word_to_write = False
        elif data == 'convo':
            self.listening = True
            self.listening_word_to_write = False
        elif data == 'word':
            self.listening = True
            self.listening_word_to_write = True

    def start(self):
        rospy.Subscriber(self.audio_topic, AudioData, self.audio_callback)

    def __exit__(self):
        #? Close the wave file
        if self.log_audio_to_file is True:
            self.wf.close()

    def audio_callback(self, data: AudioData) -> None:
        """
        Callback function to handle incoming audio data
        """
        if self.listening is False:
            return

        #? Convert the audio data to a NumPy array
        audio_data = np.frombuffer(data.data, dtype=np.int16)

        #? Detect if audio data is all silence
        silent = self.silence_detect.is_silent(audio_data)
        if silent is True:
            # rospy.loginfo(f'[ ][ROSAudioStream][audio_callback] chunk is silent')
            self.n_cont_silent_chunk += 1
        else:
            self.n_cont_silent_chunk = 0 #? reset counter

        #? Add this audio chunk to audio buffer
        # if silent is False:
        #? add even silent chunk too, if not the audio will be too fast google api cannot transcribe
        #? but we don't want to add long silence before the child speaks
        #? but we don't want a sudden sound at first
        #? => add 2 silent chunks at the beginning of the audio, then the rest of the audio, keep whatever it captured, even silent chunks
        if len(self.buf) == 0: #? if this is the first chunk with sound
            if silent is False: #? if this is chunk with sound, append the first_silent_chunks first
                # rospy.loginfo(f'[ ][ROSAudioStream][audio_callback] first chunk with sound ! len(self.first_silent_chunks[0]) = {len(self.first_silent_chunks[0])} | len(self.first_silent_chunks[1]) = {len(self.first_silent_chunks[1])}')
                #? Write the audio data to the wave file
                if self.log_audio_to_file is True:
                    self.wf.writeframes(self.first_silent_chunks[0])
                    self.wf.writeframes(self.first_silent_chunks[1])
                self.buf += self.first_silent_chunks[0] + self.first_silent_chunks[1]
            else: #? else if this is silent chunk
                #? Convert numpy array to bytes array
                audio_bytes = audio_data.tobytes()
                #? Replace whatever in self.first_silent_chunks with this (update to latest silent chunks)
                self.first_silent_chunks[0] = self.first_silent_chunks[1]
                self.first_silent_chunks[1] = audio_bytes
                # rospy.loginfo(f'[ ][ROSAudioStream][audio_callback] silent chunk when len(buf) = 0 | len(self.first_silent_chunks[0]) = {len(self.first_silent_chunks[0])} | len(self.first_silent_chunks[1]) = {len(self.first_silent_chunks[1])}')


        if len(self.buf) > 0: #? buf always at least contains 2 silent chunks
            #? Convert numpy array to bytes array
            audio_bytes = audio_data.tobytes()

            #? Write the audio data to the wave file
            if self.log_audio_to_file is True:
                self.wf.writeframes(audio_bytes)

            self.buf += audio_bytes
            # rospy.loginfo(f'len buf: {len(self.buf)}')

        if silent is False:
        # if True:
            rospy.loginfo(f'[ ][ROSAudioStream][audio_callback] silent : {silent}  |  self.n_cont_silent_chunk : {self.n_cont_silent_chunk}  |  self.buf : {len(self.buf)}  |  self.audio_buflen : {self.audio_buflen}')

        # if len(self.buf) > self.audio_buflen: #? when audio buffer reaches this size, call gcs
        if (
            silent is True and #? only split to process at silent chunk
            self.n_cont_silent_chunk > MIN_SILENT_CHUNK_TO_SPLIT and #? long silence enough might indicate the speaker finishes speaking
            ((
                self.listening_word_to_write is False and 
                len(self.buf) > self.audio_buflen #? if robot is in convo mode, then wait for full sentence (audio_buflen big enough
            ) or (
                self.listening_word_to_write is True and 
                len(self.buf) > 1024 #? if robot is listening for word to write, small buf is ennogh
            ))
        ): #? . make sure buffer len is of minimum size to be processed call gcs. 
            content = copy.deepcopy(self.buf)
            self.buf = b'' #? reset store buf
            self.n_cont_silent_chunk = 0 #? reset silent chunk counter

            #? call google cloud speech to transcribe audio buffer
            rospy.loginfo(f'\n[ ][ROSAudioStream][audio_callback] processing buf content : {len(content)}')
            start_time = time.time()
            transcribed_txt = self.gcs.recognize(content)

            # buffer = io.BytesIO(content)
            # buffer.name = 'testy.wav'
            # transcribed_txt = openai.Audio.transcribe("whisper-1", buffer)
            rospy.loginfo(f'[+][ROSAudioStream][audio_callback] transcribed_txt : {transcribed_txt}')

            # rospy.loginfo(f'   [ROSAudioStream][audio_callback] time_taken : {time.time() - start_time:.2f} secs')
            rospy.loginfo(f'   [ROSAudioStream][audio_callback] self.listening_word_to_write : {self.listening_word_to_write}')
            # rospy.loginfo(f'   [ROSAudioStream][audio_callback] self.finish_transcribe_cb : {self.finish_transcribe_cb}')

            #? call to cb func to further process this transcription
            if self.finish_transcribe_cb is not None and len(transcribed_txt) > 0:
                self.listening = False #? when process, stop capturing audio
                self.finish_transcribe_cb(transcribed_txt, self.listening_word_to_write)



class DiagramManager(object):
    def __init__(self) -> None:
        rospy.loginfo(f'[ ][DiagramManager] init called')
        #? if True, subscribe data from audio_topic, else no
        ACTION_TODO_TOPIC = rospy.get_param('~action_todo_topic', 'action_todo')
        self.pub_toact = rospy.Publisher(ACTION_TODO_TOPIC, ActionToDoMsg, queue_size=10)
        WORDS_TOPIC = rospy.get_param('~words_to_write_topic', 'words_to_write')
        self.pub_word_to_write = rospy.Publisher(WORDS_TOPIC, String, queue_size=10)
        NEW_CHILD_TOPIC = rospy.get_param('~new_teacher_topic', 'new_child')
        self.pub_new_child = rospy.Publisher(NEW_CHILD_TOPIC, String, queue_size=10)
        self.childName = None
        self.first_prompt = True

        self.openai_model = rospy.get_param('~openai_model', 'text-davinci-002')
        self.openai_temp = rospy.get_param('~openai_temp', 0.5)
        self.openai_max_tokens = rospy.get_param('~openai_max_tokens', 1024)
        self.openai_timeout = rospy.get_param('~openai_timeout', 20) #? to avoid rpm
        # TODO: might wanna store api_key somewhere more secure than passing as param
        openai.api_key = rospy.get_param('~openai_apikey', '')
        if len(openai.api_key) == 0:
            raise Exception('Invalid API_KEY', 'openai_apikey unset')

        self.au = ROSAudioStream(self.process_transcribed_data)
        self.child_gender = None

    def spin(self):
        self.au.start()
        rospy.spin()

    def detect_command(self, text):
        """
        Detect command from a text string
        """
        rospy.loginfo(f'[ ][DiagramManager][detect_command] text : {text}')
        text = text.lower()

        command = ''
        data = ''
        error_msg = ''
        for cmd_type in COMMANDS.keys(): #? types of commands: change_word, login, register
            if len(command) > 0: 
                break
            for cmd in COMMANDS[cmd_type]: #? voice commands for each type
                if cmd in text:
                    command = cmd_type

                    if cmd_type == 'change_word':
                        data = text.split(cmd)[1].strip().lower()
                        if len(text) >= 15: #? cannot write too long word
                            data = ''
                            error_msg = f"The word is too long for me. Could I try another word?"
                        elif ' ' in text: #? cannot write multiple words yet
                            data = ''
                            error_msg = f"I'm better with one single word. Could you give me one single word only?"
                    
                    elif cmd_type == 'login' or cmd_type == 'register':
                        # data = text.split(cmd)[1].strip()
                        #? ask ChatGPT to extract name from the phrase:
                        prompt = f'what is he name in this phrase: "{text}". answer short with one word which is the name only.'
                        data = self.ask_chatgpt(prompt) #? return <name>. (eg: Joey.)
                        if len(data) > 0 and 'no name' not in data:
                            data = data.split('.')[0]
                            self.childName = data

                    break

        return command, data, error_msg

    
    def process_transcribed_data(self, text, expecting_word=False):
        """
        Callback function to process transcribed data
        """
        rospy.loginfo(f'[ ][DiagramManager][process_transcribed_data] text : {text}')
        rospy.loginfo(f'   [DiagramManager][process_transcribed_data] expecting_word : {expecting_word}')

        if len(text) == 0:
            return
        
        action_to_do = ActionToDoMsg() #? what to do
        reply = '' #? if say, what to say

        if expecting_word is True: #? if robot is only expecting a word
            if len(text) >= 15: #? cannot write too long word
                reply = f"The word is too long for me. Could I try another word?"
                action_to_do.action_type = 'long_word'
                action_to_do.data = reply
                self.pub_toact.publish(action_to_do)
                return
            elif ' ' in text: #? cannot write multiple words yet
                reply = f"I'm better with one single word. Could you give me one single word only?"
                action_to_do.action_type = 'long_word'
                action_to_do.data = reply
                self.pub_toact.publish(action_to_do)
                return
            else: #? write
                self.pub_word_to_write.publish(text)
                rospy.loginfo(f'   [DiagramManager][process_transcribed_data] self.pub_word_to_write.publish({text})')
                return

        else: #? else if robot is having convo with the child, then process the child's sentence
            # #? detection of the child speaking to the robot (google is so unstable it cannot consistently detect the first hey word)
            # if text[0:4] != 'hey ' and ' hey ' not in text:
            #     self.au.listening = False
            #     return

            command, data, error_noti = self.detect_command(text)
            print(f'   [DiagramManager][process_transcribed_data] command : {command}  |  error_noti : {error_noti}')

            if len(command) == 0: #? if child does not command, just have a chill convo
                prompt = f"Reply to this as if to a 10-year-old friend: {text}"
                if self.childName is not None:
                    prompt = f"Reply to this as if to a 10-year-old friend named {self.childName}: {text}"

                reply = self.ask_chatgpt(prompt)
                if len(reply) == 0:
                    reply = f"Something's wrong with my genius fellow buddy. I could not answer that right now. Why don't we keep practising?"

            elif len(command) > 0: #? tell the robot to do some action
                if len(error_noti) > 0:
                    reply = error_noti
                else:
                    if command == 'change_word':
                        self.pub_word_to_write.publish(data)
                        rospy.loginfo(f'   [DiagramManager][process_transcribed_data](commanded) self.pub_word_to_write.publish({data})')
                    elif command == 'login':
                        self.pub_new_child.publish(f'{data}|old')
                        rospy.loginfo(f'   [DiagramManager][process_transcribed_data](commanded) self.pub_new_child.publish({data}|old)')
                    elif command == 'register':
                        self.pub_new_child.publish(f'{data}|new')
                        rospy.loginfo(f'   [DiagramManager][process_transcribed_data](commanded) self.pub_new_child.publish({data}|new)')


        if len(reply) > 0: #? action: to speak
            rospy.loginfo(f'[+][DiagramManager/process_transcribed_data] toact[speak] : {reply}')

            action_to_do.action_type = 'speak'
            action_to_do.data = reply
            self.pub_toact.publish(action_to_do)


    def ask_chatgpt(self, prompt) -> str:
        """
        Provide smart answer to a sentence using ChatGPT
        """
        # TODO: Integrate ChatGPT here
        # if self.first_prompt is True:
        # if True:
        #     prompt = f"Reply to this as if to a 10-year-old friend: {text}"
        #     if self.childName is not None:
        #         prompt = f"Reply to this as if to a 10-year-old friend named {self.childName}: {text}"
        #     self.first_prompt = False
        # else:
        #     prompt = text
        answer = ''
        start_time = time.time()
        try:
            # response = openai.Completion.create(
            #     engine=self.openai_model,
            #     prompt=prompt,
            #     temperature=self.openai_temp,
            #     max_tokens=self.openai_max_tokens,
            #     n=1,
            #     stop=None,
            #     timeout=self.openai_timeout,
            # )
            # answer = response.choices[0].text.strip()
            response = openai.ChatCompletion.create(
                model=self.openai_model,
                temperature=self.openai_temp,
                max_tokens=self.openai_max_tokens,
                messages=[{
                    "role": "user",
                    "content": prompt,
                }]
            )
            # choices[0].message.content
            answer = response.choices[0].message.content.strip()
        except openai.error.OpenAIError as error:
            # answer = f"I'm down dude"
            rospy.logerr(f'[!][DiagramManager/ask_chatgpt] OpenAI API Error: {error}')

        rospy.loginfo(f'[+][DiagramManager/ask_chatgpt] answer : {answer}')
        rospy.loginfo(f'   [DiagramManager/ask_chatgpt] time_taken : {time.time() - start_time:.2f} secs')

        return answer



if __name__ == '__main__':
    #? Subscribe to the audio topic and start the ROS node
    rospy.init_node('diagram_manager')
    dm = DiagramManager()
    dm.spin()
