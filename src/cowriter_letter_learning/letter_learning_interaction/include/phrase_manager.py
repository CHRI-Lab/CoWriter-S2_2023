import openai
from .interaction_settings import InteractionSettings


class PhraseManager:
    """
    A wrapper class for managing phrases that the NAO robot uses during
    interaction. The class stores a set of phrases and their
    corresponding counters for different interaction scenarios, such as
    introduction, demo response, asking for feedback, word response,
    and more. The phrases are fetched from the InteractionSettings
    class based on the provided language.

    Args:
        language (str): The language used for the phrases.
    """

    def __init__(self, language: str):
        (
            self.intro_phrase,
            self.demo_response_phrases,
            self.asking_phrases_after_feedback,
            self.asking_phrases_after_word,
            self.word_response_phrases,
            self.word_again_response_phrases,
            self.test_phrase,
            self.thank_you_phrase,
        ) = InteractionSettings.get_phrases(language)

        self.demo_response_phrases_counter: int = 0
        self.asking_phrases_after_feedback_counter: int = 0
        self.asking_phrases_after_word_counter: int = 0
        self.word_response_phrases_counter: int = 0
        self.word_again_response_phrases_counter: int = 0


class PhraseManagerGPT(PhraseManager):
    """
    Class for managing phrases and conversation with a GPT-based model.

    This class handles the conversation flow with the model, ensuring
    that the interactions are appropriate and conform to the set
    language. It uses an API key to facilitate communication with the
    model and manages the message history of the conversation.

    Attributes:
        language (str): The language to be used in the conversation.
        messages (list): The history of conversation with the model.

    Methods:
        get_gpt_response(input_text: str) -> str: Given the input text,
            this function generates a conversational response from the
            GPT model.
    """

    def __init__(self, language):
        super().__init__(language)
        # NOTE: The openai api key below will need to be replaced for ChatGPT functionality
        #       to work past 06/2023.
        openai.api_key = "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"
        self.language = language
        self.messages = [
            {
                "role": "system",
                "content": "You are a handwriting student of a child.",
            }
        ]

    def get_gpt_response(self, input_text):
        """
        Given input text (generated from a conversation with a child),
        gets a conversational response from the GPT model.

        The input text is added to the message history, and a response
        is generated from the GPT model. The response is then added to
        the message history and returned.

        Args:
            input_text (str): The input text from the child's
                              conversation.

        Returns:
            response_message (str): The response message generated from
                                    the GPT model.
        """
        # Add input text to message history
        self.messages.append(
            {
                "role": "user",
                "content": "Don't break lines in your reponse and use short and simple sentences answering the fllowing question: "
                + input_text,
            }
        )
        # Get chatgpt response
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo", messages=self.messages
        )
        # Add response to message history
        response_message = response["choices"][0]["message"].content
        response_message = response_message.replace("\n", " ")
        response_message = response_message.replace("\\", "")
        self.messages.append({"role": "assistant", "content": response_message})
        return response_message
