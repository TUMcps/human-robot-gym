from typing import Any, Dict, List, Tuple, Union

import pickle
import json

import numpy as np

from human_robot_gym.utils.mjcf_utils import xml_path_completion


def load_human_animation_data(
    human_animation_names: List[str],
    verbose: bool = False,
) -> List[Tuple[Dict[str, Any], Dict[str, Any]]]:
    """Load the human animation data from pickled files and the accompanying info json files.

    Gives a list of tuples of the form (animation_data, animation_info).
    If an animation info file is missing, the animation will be played back without transformation
    (i.e. no scaling, no position offset, no orientation offset).

    Args:
        human_animation_names (List[str]): List of human animation names to load.
        verbose (bool): Whether to print out debug information. Defaults to False.

    Returns:
        List[Tuple[Dict[str, Any], Dict[str, Any]]]: List of tuples of the form (animation_data, animation_info).
    """
    animation_data = []

    for animation_name in human_animation_names:
        try:
            with open(
                xml_path_completion(f"human/animations/human-robot-animations/{animation_name}.pkl"),
                "rb",
            ) as pkl_file:
                animation = pickle.load(pkl_file)
        except FileNotFoundError as e:
            if verbose:
                print(f"Animation file not found: {animation_name}")
            raise e

        try:
            with open(
                xml_path_completion(f"human/animations/human-robot-animations/{animation_name}_info.json"),
                "r",
            ) as info_file:
                info = json.load(info_file)
        except FileNotFoundError:
            if verbose:
                print(f"Animation info file not found: {animation_name}_info")
            info = {
                "position_offset": [0.0, 0.0, 0.0],
                "orientation_quat": [0.0, 0.0, 0.0, 1.0],
                "scale": 1.0,
            }

        animation_data.append((animation, info))

    return animation_data


def sin_modulation(
    classic_animation_time: int,
    modulation_start_time: int,
    amplitude: float,
    speed: float,
) -> float:
    """Sinusoidal modulation of the animation speed. Used for playing animations back and forth.

    Continuous transition from linear animation time to sinusoidal animation time at the modulation start time.
    Transition is continuously differentiable at the modulation start time if `speed` is set to `1`.

    Args:
        classic_animation_time (int): Linear progress in the animation.
        modulation_start_time (int): Time in seconds when the modulation should start.
        amplitude (float): Amplitude of the sine.
        speed (float): Frequency modifier of the sine. If `speed = 1`, the transition is continuously
            differentiable at the modulation start time (maximum animation speed in the sine is equal to the
            animation speed in the linear phase before).
            Higher values play the animation faster and lower values slower.

    Returns:
        float: Animation time after modulation.
    """
    return amplitude * np.sin(
        (classic_animation_time - modulation_start_time) / (amplitude / speed)
    ) + modulation_start_time


def layered_sin_modulations(
    classic_animation_time: int,
    modulation_start_time: int,
    amplitudes: List[float],
    speeds: List[float],
) -> float:
    """Sinusoidal modulation of the animation speed. Used for playing animations back and forth.
    Layers multiple sinusoidal modulations on top of each other for more complex modulation patterns.

    Args:
        classic_animation_time (int): linear progress in the animation.
        modulation_start_time (int): time in seconds when the modulation should start.
        amplitudes (List[float]): amplitudes of the sines.
        speeds (List[float]): Frequency modifiers of the layered sines.
            Higher values play the animation faster and lower values slower.

    Returns:
        float: animation time after modulation.
    """
    return np.sum(
        [
            sin_modulation(
                classic_animation_time=classic_animation_time,
                modulation_start_time=modulation_start_time,
                amplitude=amplitude,
                speed=speed,
            )
            for amplitude, speed in zip(amplitudes, speeds)
        ]
    ) - modulation_start_time * (len(amplitudes) - 1)


def sample_animation_loop_properties(
    animation_info: Dict[str, Any],
) -> Union[Tuple[List[float], List[float]], Dict[str, Tuple[List[float], List[float]]]]:
    """Sample amplitude and frequency for the sinusoidal modulation of the animation time.

    Amplitude and frequency are multiplied by an exponential of a clipped normal distributed random variable.
    It is clipped to [-3, 3] to avoid extreme values.

    A scaling parameter sets the range of possible random factors. For example, with a value of 1.1,
    this gives a range of [1.1^-3, 1.1^3] ~ [0.75, 1.33]
    Scaling parameters taken from the animation info.

    The animation info may either contain lists of amplitudes and speed modifiers or dictionaries
    containing such lists for multiple stages in the animation.

    Args:
        animation_info (Dict[str, Any]): Animation info dictionary.
            contains information about sine amplitude and frequency mean and std values.

    Returns:
        Union[
            Tuple[List[float], List[float]],
            Tuple[Dict[str, List[float]], Dict[str, List[float]]]
        ]: Amplitude and speed modifiers
            for the sinusoidal modulation of the animation time.
            If the animation contains multiple loopable stages this function returns a tuple of dictionaries
    """
    def random_factor_generator(std_factor: float) -> float:
        return np.exp(np.clip(np.random.normal(), -3, 3) * np.log(std_factor))

    def sample_loop_properties_lists(amplitudes: List[float], speeds: List[float]) -> Tuple[List[float], List[float]]:
        return (
            [
                amp * random_factor_generator(std_factor=animation_info["loop_amplitude_std_factor"])
                for amp in amplitudes
            ],
            [
                speed * random_factor_generator(std_factor=animation_info["loop_speed_std_factor"])
                for speed in speeds
            ],
        )

    if isinstance(animation_info["loop_amplitudes"], list):
        return sample_loop_properties_lists(
            amplitudes=animation_info["loop_amplitudes"],
            speeds=animation_info["loop_speeds"],
        )
    else:
        return {
            stage: sample_loop_properties_lists(
                amplitudes=animation_info["loop_amplitudes"][stage],
                speeds=animation_info["loop_speeds"][stage],
            )
            for stage in animation_info["loop_amplitudes"]
        }
