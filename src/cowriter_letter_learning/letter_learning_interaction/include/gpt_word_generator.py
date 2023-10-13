import os
import openai
import json
from .phrase_manager import PhraseManagerGPT

# openai.api_key = os.getenv("OPENAI_API_KEY")

class GPT_Word_Generator:
    def __init__(self, phrase_manager):
        if phrase_manager is None:
            raise ValueError("phrase_manager cannot be None and should be initialised first")
        openai.api_key = os.getenv("OPENAI_API_KEY")
        self.interest = None
        self.phrase_manager = phrase_manager

    def generate_word(self): #this function is called when the "generate word" button in manager UI is clicked
        def ask_gpt(prompt):
            response = openai.Completion.create(
                        model="text-davinci-003",
                        prompt=prompt,
                        temperature=0.5
                    ).get("choices")[0].text.lstrip()
            response = response.replace('\n', ' ')
            response = response.replace('\\', '')
            response = response.replace('.', '')
            return response

        if self.interest is None:
            message_history = self.phrase_manager.messages
            message_history = json.dumps(message_history)
            prompt = "Based on this kid's input history, suggest one general topic that the kid might be interested in. Response should only contain one word of topic. Input history: " + message_history
            self.interest = ask_gpt(prompt)
        
        prompt = "Generate one short and kids-friendly word related to " + self.interest + ". Your response should only contain the word."
        new_word = ask_gpt(prompt)

        return new_word #this new_word will be passed to the manager UI and displayed in the word_to_write space for manager to review

