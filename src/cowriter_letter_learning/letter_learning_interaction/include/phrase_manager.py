import openai
import os
import random
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
        openai.api_key = os.getenv("OPENAI_API_KEY")
        self.language = language
        self.messages = [
            {
                "role": "system",
                "content": "Your name is NAO, and you're learning handwriting from a child aged between 5-10. Make sure to keep the conversation kids-friendly.",  # noqa
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

        def moderate(input_text):
            moderation_response = openai.Moderation.create(
                input=input_text
            )

            threshold = 0.05
            category_scores = moderation_response["results"][0]["category_scores"]
            flagged = False

            for category, score in category_scores.items():
                if category not in ["self-harm/intent", "self-harm/instructions"] and score > threshold:
                    flagged = True
                    break

            return flagged

        # Get GPT response only if the moderation outcome is not flagged
        flagged = False
        flagged = moderate(input_text)
        self.messages.append({"role": "user", "content": input_text})

        if flagged:
            response_message = "Sorry, but I can't assist with that. Why don't you tell me about your favorite animal?"
            self.messages.append({"role": "assistant", "content": response_message})
            return response_message
        
        else:
            self.messages.append({"role": "assistant", "content": "Respond to the following chat. Your response should be short and concise: " + input_text})
            
            # Get GPT response
            response = openai.ChatCompletion.create(
                model="gpt-4",
                temperature=0.5,
                max_tokens=1024,
                n=1,
                messages=self.messages
            )
            
            # Add response to message history
            response_message = response["choices"][0]["message"].content
            response_message = response_message.replace('\n', ' ')
            response_message = response_message.replace('\\', '')
            self.messages.append({"role": "assistant", "content": response_message})

            return response_message

    def get_motion_path(self, response_message):
        """
        Given response_text (returned message from get_gpt_response),
        gets a motion path for NAO's motion

        Args:
            response_message (str): The output text from get_gpt_response

        Returns:
            path (str): motion path selected for the response_message
        """
        
        path = None
        motions = {
            "greeting" : [
                "animations/Stand/Gestures/Hey_1",
                "animations/Stand/Gestures/Hey_6"
            ],
            "affirmation" : [
                "animations/Stand/Gestures/Yes_1",
                "animations/Stand/Gestures/Yes_2",
                "animations/Stand/Gestures/Yes_3"
            ],
            "interrogative" : [
                "animations/Stand/Gestures/YouKnowWhat_1",
                "animations/Stand/Gestures/YouKnowWhat_5"
            ],
            "joy" : [
                "animations/Stand/Gestures/Enthusiastic_4", 
                "animations/Stand/Gestures/Enthusiastic_5"
            ],
            "hesitation" : [
                "animations/Stand/Gestures/IDontKnow_1",
                "animations/Stand/Gestures/IDontKnow_2"
            ],
            "refusal" : [
                "animations/Stand/Gestures/No_3",
                "animations/Stand/Gestures/No_8",
                "animations/Stand/Gestures/No_9"
            ]
        }
        # motions = {
        #     "greeting": [ #greeting paths are deprecated since 2.8
        #         "animations/Stand/Gestures/Hey_1",
        #         "animations/Stand/Gestures/Hey_6"
        #     ],
        #     "affirmation" : [
        #         "animations/Stand/Affirmation/NAO/Center_Neutral_AFF_01",
        #         "animations/Stand/Affirmation/NAO/Center_Neutral_AFF_10",
        #         "animations/Stand/Affirmation/NAO/Center_Slow_AFF_02",
        #         "animations/Stand/Affirmation/NAO/Center_Strong_AFF_01"
        #     ],
        #     "interrogative" : [
        #         "animations/Stand/Question/NAO/Center_Neutral_QUE_03",
        #         "animations/Stand/Question/NAO/Center_Neutral_QUE_08",
        #         "animations/Stand/Question/NAO/Center_Slow_QUE_02",
        #         "animations/Stand/Question/NAO/Center_Strong_QUE_03"
        #     ],
        #     "joy" : [
        #         "animations/Stand/Exclamation/NAO/Center_Neutral_EXC_03", 
        #         "animations/Stand/Exclamation/NAO/Center_Neutral_EXC_06", 
        #         "animations/Stand/Exclamation/NAO/Center_Slow_EXC_0",
        #         "animations/Stand/Exclamation/NAO/Center_Strong_EXC_04"
        #     ],
        #     "hesitation" : [
        #         "animations/Stand/Question/NAO/Center_Neutral_QUE_01",
        #         "animations/Stand/Question/NAO/Center_Neutral_QUE_05",
        #         "animations/Stand/Question/NAO/Center_Neutral_QUE_10",
        #         "animations/Stand/Question/NAO/Center_Strong_QUE_01"
        #     ],
        #     "refusal" : [
        #         "animations/Stand/Negation/NAO/Center_Neutral_NEG_01", 
        #         "animations/Stand/Negation/NAO/Center_Neutral_NEG_04", 
        #         "animations/Stand/Negation/NAO/Center_Slow_NEG_01", 
        #         "animations/Stand/Negation/NAO/Center_Strong_NEG_01"
        #     ]
        # }

        def ask_gpt(prompt):
            response = openai.Completion.create(
                            model="text-davinci-003",
                            prompt=prompt,
                            temperature=0.3 # reduced temperautre to minimise generating random response
                        ).get("choices")[0].text.lstrip()
            response = response.replace('\n', ' ')
            response = response.replace('\\', '')
            response = response.replace('.', '')
            response = response.lower()
            return response

        if response_message != "":
            prompt = "Among this list of contexts [greeting, affirmation, interrogative, joy, hesitation, refusal, none], select one that best matches the following message: " + response_message + ". Select none if no context match is found. Your response should only contain one context word selected from the list."
            selected_tag = ask_gpt(prompt)
            if selected_tag in motions:
                tag_paths = motions[selected_tag]
                path = random.choice(tag_paths)
                
        return path
