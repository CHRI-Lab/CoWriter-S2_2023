import os
import numpy as np
import wave
import copy

from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

from speech_recognition.include.google_cloud import GCS


class StaticSilenceDetector:
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
        return True if np.abs(snd_data).mean() < self.threshold else False


class AudioStream:
    def __init__(
        self,
        n_channels,
        depth,
        sample_rate,
        min_silent_chunk_to_split,
        audio_buflen,
        silent_threshold,
        log_audio_to_file,
        audio_outfile,
        finish_transcribe_cb,
        logger,
    ):
        self.logger = logger

        self.min_silent_chunk_to_split = min_silent_chunk_to_split

        # Do we want to log audio to file
        self.log_audio_to_file = log_audio_to_file
        if self.log_audio_to_file is True:
            # Set the name of the output wave file
            # if we do want to log audio to file
            self.audio_outfile = audio_outfile
            # Create audio log base dir if not existed
            outdir = os.path.dirname(self.audio_outfile)
            if not os.path.isdir(outdir):
                os.makedirs(outdir)
            # Create a new wave file for writing
            self.wf = wave.open(self.audio_outfile, "wb")

        self.sample_rate = sample_rate
        self.n_channels = n_channels
        self.depth = depth

        if self.depth == 8:
            self.sample_width = 1
        elif self.depth == 16:
            self.sample_width = 2
        elif self.depth == 32:
            self.sample_width = 4
        else:
            raise ValueError("depth must be 8, 16 or 32")

        # Set the wave file parameters based on the audio format
        if self.log_audio_to_file is True:
            self.wf.setnchannels(self.n_channels)  # Mono audio
            self.wf.setsampwidth(self.sample_width)  # 16-bit audio
            self.wf.setframerate(self.sample_rate)  # Sample rate of 16 kHz

        # listening or not, when the robot speaking, we should stop listening
        self.listening = False
        self.listening_word_to_write = False

        # Speech to text class
        self.gcs = GCS(self.n_channels, self.sample_rate)
        self.buf = b""

        # store the 2 silent chunks right before the first detected sound,
        # will prepend this silent_chunk to the audio buffer
        # so that google cloud doesn't miss the first word
        self.first_silent_chunks = [
            b"",
            b"",
        ]
        # callback on finishing transcribe
        self.finish_transcribe_cb = finish_transcribe_cb

        # minimum buffer len to store audio data before transcribe
        # (use dynamic buffer by detecting silence instead of fixed buflen)
        self.audio_buflen = audio_buflen
        # keep track of continous silent chunk
        self.n_cont_silent_chunk = 0
        self.silent_threshold = silent_threshold
        self.silence_detect = StaticSilenceDetector(
            self.sample_rate, self.silent_threshold
        )

    def __exit__(self):
        # Close the wave file
        if self.log_audio_to_file is True:
            self.wf.close()

    def setListening(self, data: String):
        data = data.data.lower()
        self.logger.info(f"[ ][ROSAudioStream][setListening] data = {data}")
        if data == "false":
            self.listening = False
            self.listening_word_to_write = False
        elif data == "convo":
            self.listening = True
            self.listening_word_to_write = False
        elif data == "word":
            self.listening = True
            self.listening_word_to_write = True

    def audio_callback(self, data: AudioData) -> None:
        """
        Callback function to handle incoming audio data
        """
        if self.listening is False:
            return

        # Convert the audio data to a NumPy array
        audio_data = np.frombuffer(data.data, dtype=np.int16)

        # Detect if audio data is all silence
        silent = self.silence_detect.is_silent(audio_data)
        if silent is True:
            self.n_cont_silent_chunk += 1
        else:
            self.n_cont_silent_chunk = 0  # reset counter

        # Add this audio chunk to audio buffer
        # if silent is False:
        # add even silent chunk too,
        # if not the audio will be too fast google api cannot transcribe
        # but we don't want to add long silence before the child speaks
        # but we don't want a sudden sound at first
        # => add 2 silent chunks at the beginning of the audio,
        # then the rest of the audio,
        # keep whatever it captured, even silent chunks
        if len(self.buf) == 0:  # if this is the first chunk with sound
            # if this is chunk with sound, append the first_silent_chunks first
            if silent is False:
                # Write the audio data to the wave file
                if self.log_audio_to_file is True:
                    self.wf.writeframes(self.first_silent_chunks[0])
                    self.wf.writeframes(self.first_silent_chunks[1])
                self.buf += (
                    self.first_silent_chunks[0] + self.first_silent_chunks[1]
                )
            else:  # else if this is silent chunk
                # Convert numpy array to bytes array
                audio_bytes = audio_data.tobytes()
                # Replace whatever in self.first_silent_chunks with this
                # (update to latest silent chunks)
                self.first_silent_chunks[0] = self.first_silent_chunks[1]
                self.first_silent_chunks[1] = audio_bytes

        if len(self.buf) > 0:  # buf always at least contains 2 silent chunks
            # Convert numpy array to bytes array
            audio_bytes = audio_data.tobytes()

            # Write the audio data to the wave file
            if self.log_audio_to_file is True:
                self.wf.writeframes(audio_bytes)

            self.buf += audio_bytes
            # rospy.loginfo(f'len buf: {len(self.buf)}')

        if silent is False:
            self.logger.info(
                f"[ ][ROSAudioStream][audio_callback] silent : {silent}  |"
                + f"  self.n_cont_silent_chunk : {self.n_cont_silent_chunk}  |"
                + f"  self.buf : {len(self.buf)}  |"
                + f"  self.audio_buflen : {self.audio_buflen}"
                + f"  audio_data mean: {np.abs(audio_data).mean()}"
            )

        if (
            silent is True
            # long silence enough might indicate the speaker finishes speaking
            and self.n_cont_silent_chunk > self.min_silent_chunk_to_split
            and (
                # if robot is in convo mode, then wait for full sentence
                # (audio_buflen big enough
                (
                    self.listening_word_to_write is False
                    and len(self.buf) > self.audio_buflen
                )
                # if robot is listening for word to write, small buf is ennogh
                or (
                    self.listening_word_to_write is True
                    and len(self.buf) > 1024
                )
            )
        ):  # make sure buffer len is of minimum size to be processed call gcs.
            content = copy.deepcopy(self.buf)
            self.buf = b""  # reset store buf
            self.n_cont_silent_chunk = 0  # reset silent chunk counter

            # call google cloud speech to transcribe audio buffer
            self.logger.info(
                "[ ][ROSAudioStream][audio_callback] "
                + f"processing buf content : {len(content)}"
            )
            transcribed_txt = self.gcs.recognize(content)

            self.logger.info(
                "[+][ROSAudioStream][audio_callback] "
                + f"transcribed_txt : {transcribed_txt}"
            )
            self.logger.info(
                "   [ROSAudioStream][audio_callback] "
                + "self.listening_word_to_write : "
                + f"{self.listening_word_to_write}"
            )

            # call to cb func to further process this transcription
            if (
                self.finish_transcribe_cb is not None
                and len(transcribed_txt) > 0
            ):
                self.listening = False  # when process, stop capturing audio
                self.finish_transcribe_cb(
                    transcribed_txt, self.listening_word_to_write
                )
