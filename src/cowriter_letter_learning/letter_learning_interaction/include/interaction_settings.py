#!/usr/bin/env python3

"""
Manages the settings used for the learning_words_nao.py node.

This file defines the classes and functions that manage settings for
the word learning interactions of a Nao robot. It contains:
    - An enumeration for different learning modes for shape learning.
    - A dictionary with the learning mode for each shape expected.
    - A class to manage settings for word learning interactions.
    - Several methods to get values such as trajectory timings, head
      angles, and phrases.
    - A method to generate settings for the given shape type.
"""

from enum import Enum
from typing import Dict, List, Optional, Tuple
import os.path

import numpy as np

from .shape_learner import settings_struct


class LearningModes(Enum):
    """
    An enumeration to represent different learning modes for shape
    learning.
    """

    starts_good = 0
    starts_bad = 1
    starts_random = 2


# Define the learning mode for each shape we expect to see
learning_modes: Dict[str, LearningModes] = {
    "c": LearningModes.starts_random,
    "e": LearningModes.starts_random,
    "m": LearningModes.starts_random,
    "n": LearningModes.starts_random,
    "o": LearningModes.starts_random,
    "s": LearningModes.starts_random,
    "u": LearningModes.starts_random,
    "w": LearningModes.starts_random,
}


class InteractionSettings:
    """
    A class to manage settings for word learning interactions.
    """

    # dataset_directory is class variable, default value None
    dataset_directory: Optional[str] = None

    @staticmethod
    def get_trajectory_timings(nao_writing: bool) -> Tuple[float, float, float]:
        """
        Get the trajectory timings based on whether NAO is writing or not.

        Args:
            nao_writing (bool): True if NAO is writing, False otherwise.

        Returns:
            tuple: A tuple containing the time allowed for the first
                   point in traj (t0), seconds between points in traj
                   (dt), and delay before executing the traj.
        """
        if nao_writing:
            t0: float = 3.0
            dt: float = 0.25
            delay_before_executing: float = 3.0
        else:
            t0: float = 0.01
            dt: float = 0.1
            delay_before_executing: float = 2.5

        return t0, dt, delay_before_executing

    @staticmethod
    def get_head_angles() -> (
        Tuple[
            Tuple[float, float],
            Tuple[float, float],
            Tuple[float, float],
            Tuple[float, float],
            Tuple[float, float],
            Tuple[float, float],
        ]
    ):
        """
        Get the head angles for NAO to look at different positions.

        Returns:
            tuple: A tuple containing the head angles for NAO to look
                   at tablet down, tablet right, tablet left, person
                   front, person right, and person left.
        """
        head_angles_look_at_tablet_down: Tuple[float, float] = (-0.01538, 0.512)
        head_angles_look_at_tablet_right: Tuple[float, float] = (
            -0.2,
            0.08125996589660645,
        )
        head_angles_look_at_tablet_left: Tuple[float, float] = (
            0.2,
            0.08125996589660645,
        )
        head_angles_look_at_person_front: Tuple[float, float] = (
            -0.0123,
            0.1825,
        )
        head_angles_look_at_person_right: Tuple[float, float] = (
            -0.9639739513397217,
            0.08125996589660645,
        )
        head_angles_look_at_person_left: Tuple[float, float] = (
            0.9639739513397217,
            0.08125996589660645,
        )

        return (
            head_angles_look_at_tablet_down,
            head_angles_look_at_tablet_right,
            head_angles_look_at_tablet_left,
            head_angles_look_at_person_front,
            head_angles_look_at_person_right,
            head_angles_look_at_person_left,
        )

    # ---------------------------------------------- WORD LEARNING PHRASES

    @staticmethod
    def get_phrases(
        language: str,
    ) -> Tuple[
        str, str, str, List[str], List[str], List[str], List[str], List[str]
    ]:
        """
        Get phrases for the specified language.

        Args:
            language (str): The language for which to get the phrases.

        Returns:
            Tuple: A tuple containing the different phrases.
        """
        if language.lower() == "english":
            intro_phrase: str = (
                "Hello. I'm Nao. What are you interested in learning today?"
            )
            test_phrase: str = "Ok, test time. I'll try my best."
            thank_you_phrase: str = "Thank you for your help."

            demo_response_phrases: List[str] = [
                "Okay, I'll try it like you",
                "So that's how you write %s",
                "That's a much better %s than mine",
                "I'll try to copy you",
                "Let me try now",
                "Thank you",
            ]
            asking_phrases_after_feedback: List[str] = [
                "Any better?",
                "How about now?",
                "Now what do you think?",
                "Is there a difference?",
                "Is this one okay?",
                "Will you show me how?",
                "Did I improve?",
            ]
            asking_phrases_after_word: List[str] = [
                "Okay, what do you think?",
                "This is a hard word",
                "Is this how you write it?",
                "Please help me",
            ]
            word_response_phrases: List[str] = [
                "%s, okay. ",
                "%s seems like a good word",
                "Hopefully I can do well with this word",
                "%s. Let's try",
                "Okay, %s now",
            ]
            word_again_response_phrases: List[str] = [
                "%s again, okay.",
                "I thought I already did %s",
                "You like to practice this word",
            ]

        elif language.lower() == "french":
            intro_phrase: str = "Allez, on écrit des mots"
            test_phrase: str = "Ok, c'est l'heure du test. J'ai un peu peur."
            thank_you_phrase: str = "Merci pour ton aide !"

            demo_response_phrases: List[str] = [
                "Ok",
                "D'accord, j'essaye comme ça",
                "Ah, c'est comme ça qu'on écrit %s",
                "Bon",
                "Ce %s est pas mal",
                "Bon, j'essaye comme toi",
                "D'accord",
                "Ok, à moi",
                "À mon tour",
                "Ok",
                "Merci, je vais essayer",
            ]
            asking_phrases_after_feedback: List[str] = [
                "C'est mieux ?",
                "Voilà",
                "Et comme ça ?",
                "Tu en penses quoi ?",
                "Alors ?",
                "Qu'est-ce que tu en penses ?",
                "Il y a une différence ?",
                "Ça va cette fois ?",
                "Je me suis amélioré ?",
                "Tu trouves que c'est mieux ?",
            ]
            asking_phrases_after_word: List[str] = [
                "Bon, qu'est ce que tu en penses ?",
                "Pas facile !",
                "Voilà",
                "C'est bien comme ça ?",
                "Bon",
                "Je crois que j'ai besoin d'aide.",
                "Et voilà !",
            ]
            word_response_phrases: List[str] = [
                "D'accord pour %s",
                "Ok, j'essaye %s",
                "Bon, je devrais y arriver",
                "D'accord",
                "%s ? ok",
            ]
            word_again_response_phrases: List[str] = [
                "Encore %s ? bon, d'accord.",
                "Je crois que j'ai déjà fait %s",
                "On dirait que tu aimes bien %s !",
                "Encore ?",
            ]

        else:
            raise RuntimeError(f"Requested language ({language}) not supported")

        return (
            intro_phrase,
            demo_response_phrases,
            asking_phrases_after_feedback,
            asking_phrases_after_word,  # type: ignore
            word_response_phrases,
            word_again_response_phrases,
            test_phrase,
            thank_you_phrase,
        )  # type: ignore

    @classmethod
    def set_dataset_directory(cls, dataset_directory: str) -> None:
        """
        Set the dataset directory.

        Arg:
            dataset_directory (str): The path to the dataset directory.
        """
        cls.dataset_directory = dataset_directory

    # ---------------------------------------------- WORD LEARNING SETTINGS
    @staticmethod
    def generate_settings(shape_type: str) -> settings_struct:  # type: ignore
        """
        Generate settings for the given shape type.

        Args:
            shape_type: The type of shape for which to generate settings.

        Returns:
            An instance of settings_struct with the generated settings.
        """
        # Check if dataset_directory has been set
        if InteractionSettings.dataset_directory is None:
            raise RuntimeError(
                "Dataset directory has not been set yet with set_dataset_directory()"  # noqa: E501
            )

        # Define initial values for the settings
        params_to_vary: List[int] = [3]
        initial_bounds_std_dev_multiples: np.ndarray = np.array([[-6, 6]])
        do_groupwise_comparison: bool = True
        initial_param_value: List[float] = [0.0]
        initial_bounds: np.ndarray = np.array([[np.nan, np.nan]])

        # Load dataset file for the given shape type
        dataset_file: str = (
            InteractionSettings.dataset_directory + "/" + shape_type + ".dat"
        )
        if not os.path.exists(dataset_file):
            raise RuntimeError("Dataset is not known for shape " + shape_type)

        # Load dataset parameter file
        dataset_param: str = (
            InteractionSettings.dataset_directory + "/params.dat"
        )
        if not os.path.exists(dataset_param):
            raise RuntimeError("parameters not found for this dataset ")
        else:
            with open(dataset_param, "r") as f:
                initial_param_value = [0.0] * 20
                for line in f.readline():
                    # Find the line for the given shape type
                    # and get its parameters
                    if line[1:-2] == shape_type:
                        initial_param_value = [
                            float(s) for s in f.readline().split(", ")
                        ]
                        break

        # Generate settings for the shape type
        settings: settings_struct = settings_struct(  # type: ignore
            shape_learning=shape_type,
            params_to_vary=params_to_vary,
            do_groupwise_comparison=do_groupwise_comparison,
            init_dataset_file=dataset_file,
            update_dataset_files=None,
            initial_bounds=initial_bounds,
            initial_bounds_std_dev_multiples=initial_bounds_std_dev_multiples,
            initial_param_value=initial_param_value,
            param_file=None,
            min_param_diff=0.4,
        )

        return settings
