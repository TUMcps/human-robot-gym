"""This file defines the logging functionality via tensorboard.

Further defines model saving and loading.

Author:
    Felix Trost (FT)

Contributors:

Changelog:
    11.09.23 FT File created
"""
from stable_baselines3.common.utils import safe_mean

from stable_baselines3.common.callbacks import BaseCallback

from typing import List, Tuple, Union
import time


class LoggingCallback(BaseCallback):
    """Custom callback for plotting metrics in tensorboard.
    Additionally, this callback can be used to save the model and replay buffer periodically.

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
        """Save metrics every `self.log_interval` steps/episodes."""
        for i in range(len(self.locals["dones"])):
            if self.locals["dones"][i]:
                self.episode_counter += 1
                for key in self.additional_log_info_keys:
                    if key in self.locals["infos"][i]:
                        self._info_buffer[key].append(self.locals["infos"][i][key])
                if self.log_interval[1] == "episode" and (self.episode_counter + 1) % self.log_interval[0] == 0:
                    self.log_info()
        if self.log_interval[1] == "step" and (
            n_logged_infos := self.num_timesteps // self.log_interval[0]
        ) > self._n_logged_infos:
            self._n_logged_infos = n_logged_infos
            self.log_info()

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

    def log_info(self):
        """Record metrics to tensorboard."""
        for key in self._info_buffer:
            self.logger.record(
                "rollout/{}".format(key), safe_mean(self._info_buffer[key])
            )
            self._info_buffer[key] = []
        if hasattr(self.model, '_dump_logs'):
            self.model._dump_logs()
        elif hasattr(self.model, 'logger'):
            self.logging_on_policy()
        else:
            self.logger.dump(step=self.num_timesteps)

    def logging_on_policy(self):
        """Log default environment statistics for on-policy algorithms."""
        fps = int((self.model.num_timesteps -
                   self.model._num_timesteps_at_start) / (time.time() - self.model.start_time))
        if len(self.model.ep_info_buffer) > 0 and len(self.model.ep_info_buffer[0]) > 0:
            self.model.logger.record("rollout/ep_rew_mean",
                                     safe_mean([ep_info["r"] for ep_info in self.model.ep_info_buffer]))
            self.model.logger.record("rollout/ep_len_mean",
                                     safe_mean([ep_info["l"] for ep_info in self.model.ep_info_buffer]))
        self.model.logger.record("time/fps", fps)
        self.model.logger.record("time/time_elapsed", int(time.time() - self.model.start_time), exclude="tensorboard")
        self.model.logger.record("time/total_timesteps", self.model.num_timesteps, exclude="tensorboard")
        self.model.logger.dump(step=self.num_timesteps)
