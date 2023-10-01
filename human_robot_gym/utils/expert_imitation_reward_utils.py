import numpy as np


def gaussian_similarity_fn(delta: float, iota: float) -> float:
    r"""Similarity function for expert imitation reward training.

    Used in `ActionBasedExpertImitationRewardWrapper` and `StateBasedExpertImitationRewardWrapper` subclasses.
    Rescaled Gaussian function that returns a similarity value in $(0, 1]$ based on a non-negative distance metric
    $\delta$ between the agent and expert.

    DeepMimic (Peng et al., 2018) uses a similar model for the end-effector similarity reward.
    Link to paper: https://arxiv.org/abs/1804.02717

    Exponential form:
    $sim_G(\delta) = exp{-0.5 \cdot (\delta \cdot \frac{\nu}{\iota})^2}$

    where:
        \nu = sqrt{2 \cdot ln(2)}

    Simplifies to:
    sim_G(\delta) = 2^{-(\frac{\delta}{\iota})^2}

    Args:
        delta (float): distance metric between agent and expert
        iota (float): half width at half maximum;
            distance after which the reward should be at 0.5
    Returns:
        float: similarity based on distance
    """
    return 2 ** (-(delta / iota) ** 2)


def tanh_similarity_fn(delta: float, iota: float) -> float:
    r"""Similarity function for expert imitation reward training.

    Used in `ActionBasedExpertImitationRewardWrapper` and `StateBasedExpertImitationRewardWrapper` subclasses.
    Rescaled tanh function that returns a similarity value in $(0, 1]$ based on a non-negative distance metric
    $\delta$ between the agent and expert.

    Function: $sim_T(\delta) = -tanh(tan(0.5) \cdot \frac{\delta}{\iota}) + 1$

    Args:
        delta (float): distance metric between agent and expert
        iota (float): scaling parameter: distance after which the reward should be at 0.5
    Returns:
        float: similarity based on distance
    """
    return -np.tanh(np.tan(0.5) * delta / iota) + 1


def similarity_fn(name: str, delta: float, iota: float) -> float:
    r"""Calculate similarity from a non-negative distance metric for expert imitation reward training.

    Used in `ActionBasedExpertImitationRewardWrapper` and `StateBasedExpertImitationRewardWrapper` subclasses.
    Returns a similarity value in $(0, 1]$. The similarity function to use is specified by `name`.

    Args:
        name (str): similarity function name. Can be either `"gaussian"` or `"tanh"`.
        delta (float): distance metric between agent and expert
        iota (float): scaling parameter: distance after which the reward should be at 0.5

    Returns:
        function: similarity value based on distance

    Raises:
        ValueError: Unknown similarity function: {name}
    """
    if name == 'gaussian':
        return gaussian_similarity_fn(delta=delta, iota=iota)
    elif name == 'tanh':
        return tanh_similarity_fn(delta=delta, iota=iota)
    else:
        raise ValueError(f'Unknown similarity function: {name}')
