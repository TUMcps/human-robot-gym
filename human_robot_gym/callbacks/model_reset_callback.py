"""This file contains a callback for resetting the model parameters periodically during training.

This approach was introduced by the paper "The Primacy Bias in Deep Reinforcement Learning" by
Nikishin et al. (2022): https://proceedings.mlr.press/v162/nikishin22a/nikishin22a.pdf.
The authors found that resetting the model parameters while maintaining the replay buffer
improves the ability of the algorithm to learn from data that was encountered later in training.

Author:
    Felix Trost (FT)

Changelog:
    25.05.23 (FT): File created.
"""
from typing import Callable, Optional
from stable_baselines3.common.callbacks import BaseCallback


class ModelResetCallback(BaseCallback):
    """Callback for resetting the model parameters periodically during training.

    This approach was introduced by the paper "The Primacy Bias in Deep Reinforcement Learning" by
    Nikishin et al. (2022): https://proceedings.mlr.press/v162/nikishin22a/nikishin22a.pdf.
    The authors found that resetting the model parameters while maintaining the replay buffer
    improves the ability of the algorithm to learn from data that was encountered later in training.

    Args:
        n_steps_between_resets (int): Number of steps between parameter resets.
        reset_fn (() -> None | None): Function that resets the model parameters.
            Can be set to `None`, in this case all named parameters of the model that do not belong to
            the feature extractor are reset. Defaults to `None`.
        verbose (int): Verbosity level. Defaults to 0.
    """
    def __init__(
        self,
        n_steps_between_resets: int,
        reset_fn: Optional[Callable[[], None]] = None,
        verbose: int = 0,
    ):
        super().__init__(verbose)
        self._n_steps_between_resets = n_steps_between_resets
        self._steps_since_last_resets = 0
        self._reset_fn = reset_fn

    def _on_step(self) -> bool:
        """Reset the model parameters periodically."""
        if (self.num_timesteps - self._steps_since_last_call) >= self._n_steps_between_resets:
            self._steps_since_last_call = self.num_timesteps
            self._reset_model()

        return True

    def _reset_model(self):
        """Reset the model parameters."""
        if self.verbose > 0:
            print("Resetting model...")
        if self._reset_fn is None:
            # named_modules contains all named modules in the module tree in a single iterator
            for name, module in self.model.policy.named_modules():
                if hasattr(module, "reset_parameters") and "features_extractor" not in name:
                    module.reset_parameters()
        else:
            self._reset_fn()
