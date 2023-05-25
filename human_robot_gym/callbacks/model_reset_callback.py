from typing import Callable, Optional
from stable_baselines3.common.callbacks import BaseCallback


class ModelResetCallback(BaseCallback):
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
        if (self.num_timesteps - self._steps_since_last_call) >= self._n_steps_between_resets:
            self._steps_since_last_call = self.num_timesteps
            self._reset_model()

        return True

    def _reset_model(self):
        if self.verbose > 0:
            print("Resetting model...")
        if self._reset_fn is None:
            # named_modules contains all named modules in the module tree in a single iterator
            for name, module in self.model.policy.named_modules():
                if hasattr(module, "reset_parameters") and "features_extractor" not in name:
                    module.reset_parameters()
        else:
            self._reset_fn()
