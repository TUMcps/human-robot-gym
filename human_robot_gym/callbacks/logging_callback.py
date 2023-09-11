"""This file defines the logging functionality via tensorboard for wandb.

Further defines model saving and loading.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2.5.22 JT Formatted docstrings
"""

from stable_baselines3.common.utils import safe_mean

from stable_baselines3.common.callbacks import BaseCallback

from typing import Any, Dict, List, Tuple, Union


class LoggingCallback(BaseCallback):
    """Custom callback for plotting additional values in tensorboard.

    Args:
        verbose: Extra terminal outputs.
        save_freq: Save the model and replay buffer every x steps.
        model_path: predefined model file for loading / saving.
        start_episode: Define start episode (if model is loaded).
        additional_log_info_keys: Additionally log these keys from the info dict.
        log_interval: How frequently to log additional info to tensorboard.
            Can be set to a single integer or a (int, str) tuple
            If an integer: log each `log_interval` timesteps.
            If a tuple (int, str): tuple of frequency and unit, e.g. (100, "step").
                Valid units are "step" and "episode".
            If the unit is "step" and multiple environments are used, make sure the
                interval is a multiple of the number of environments, otherwise the
                data will be logged in irregular intervals.
    """

    def __init__(
        self,
        verbose: int = 0,
        save_freq: int = 100,
        model_path: str = "models",
        start_episode: int = 0,
        additional_log_info_keys: List[str] = ["goal_reached"],
        log_interval: Union[int, Tuple[int, str]] = (1000, "step"),
    ):  # noqa: D107
        super().__init__(
            verbose,
        )
        self.save_freq = save_freq
        self._n_stored_models = 0
        self.episode_counter = start_episode
        self.additional_log_info_keys = additional_log_info_keys
        self.model_path = model_path
        self._info_buffer = dict()

        for key in additional_log_info_keys:
            self._info_buffer[key] = []
        self._n_logged_infos = 0

        if isinstance(log_interval, int):
            log_interval = (log_interval, "step")

        if isinstance(log_interval[0], str) and isinstance(log_interval[1], int):
            log_interval = (log_interval[1], log_interval[0])

        self.log_interval = log_interval

        # eval results
        self.evaluations_results = []
        self.evaluations_timesteps = []
        self.evaluations_length = []
        # For computing success rate
        self._eval_info_buffer = dict()
        for key in additional_log_info_keys:
            self._eval_info_buffer[key] = []

    def _on_step(self) -> bool:
        for i in range(len(self.locals["dones"])):
            if self.locals["dones"][i]:
                self.episode_counter += 1
                for key in self.additional_log_info_keys:
                    if key in self.locals["infos"][i]:
                        self._info_buffer[key].append(self.locals["infos"][i][key])
                if self.log_interval[1] == "episode" and (self.episode_counter + 1) % self.log_interval[0] == 0:
                    self._log_info()
        if self.log_interval[1] == "step" and (
            n_logged_infos := self.num_timesteps // self.log_interval[0]
        ) > self._n_logged_infos:
            self._n_logged_infos = n_logged_infos
            self._log_info()

        # Store models every `self.save_freq` timesteps
        # With parallel envs, `self.num_timesteps` is incremented by `n_envs` at each step
        # Thus, We save the model at the first step that crosses the next threshold
        if (n_stored_models := self.num_timesteps // self.save_freq + 1) > self._n_stored_models:
            self._n_stored_models = n_stored_models
            save_timestep = self.save_freq * (self._n_stored_models - 1)

            if self.verbose > 0:
                print(f"Saving model at {save_timestep} timesteps")

            self.model.save(
                f"{self.model_path}/model_{save_timestep:_}"  # File format: model_100_000.zip
            )
            if hasattr(self.model, 'save_replay_buffer'):
                self.model.save_replay_buffer(f"{self.model_path}/replay_buffer")

        return True

    def _log_info(self):
        for key in self._info_buffer:
            self.logger.record(
                "rollout/{}".format(key), safe_mean(self._info_buffer[key])
            )
            self._info_buffer[key] = []
        self.model._dump_logs()

    def _log_success_callback(
        self, locals_: Dict[str, Any], globals_: Dict[str, Any]
    ) -> None:
        """Pass this callback to the  ``evaluate_policy`` function in order to log the success rate.

        This is used when applicable, for instance when using HER.

        Args:
            locals_:
            globals_:
        """
        info = locals_["info"]

        if locals_["done"]:
            for key in self._eval_info_buffer.keys():
                maybe_is_key = info.get(key)
                if maybe_is_key is not None:
                    self._eval_info_buffer[key].append(maybe_is_key)
