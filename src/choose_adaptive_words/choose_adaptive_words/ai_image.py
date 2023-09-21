import os
import openai
import urllib.request
import pkg_resources

api_key = "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"
openai.api_key = api_key


class AI_IMAGE:
    def __init__(self):
        self.user_prompt = "A cartoon"
        self.size = "256x256"
        self.file_name = "ai_image.png"

    def generate_image(self, text):
        choose_adaptive_words_path = pkg_resources.resource_filename(
            __name__, "design"
        )

        response = openai.Image.create(
            prompt=self.user_prompt + text, n=1, size=self.size
        )
        image_url = response["data"][0]["url"]
        urllib.request.urlretrieve(
            image_url, choose_adaptive_words_path + "/assets/ai_image.png"
        )
