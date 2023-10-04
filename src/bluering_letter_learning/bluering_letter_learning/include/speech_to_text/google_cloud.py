from google.cloud import speech_v1p1beta1 as speech


class GCS():
    def __init__(self, n_channel=1, sample_rate=16000):
        super().__init__()
        #? Set up the Speech-to-Text client
        self.client = speech.SpeechClient()

        #? Set up the transcription configuration
        self.config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            language_code='en-US',
            enable_word_time_offsets=True,

        )
        self.config.sample_rate_hertz = sample_rate
        self.config.audio_channel_count = n_channel

    def recognize(self, content: bytes) -> str:
        """
        Perform the transcription
        """
        # print(f'[ ][recognize] content : {content}')
        audio = speech.RecognitionAudio(content=content)
        response = self.client.recognize(config=self.config, audio=audio)
        print(f'   [recognize] response : {response}')

        total_str = []
        for result in response.results:
            for word in result.alternatives[0].words:
                total_str.append(word.word)
        return ' '.join(total_str).strip()
