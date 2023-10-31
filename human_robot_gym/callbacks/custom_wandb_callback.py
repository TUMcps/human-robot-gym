"""This file defines the logging functionality via tensorboard for wandb.

Further defines model saving and loading.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2.5.22 JT Formatted docstrings
"""
import numpy as np
import gym

from stable_baselines3.common.utils import safe_mean
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env import (
    VecEnv,
    sync_envs_normalization,
)

from wandb.integration.sb3 import WandbCallback

from typing import Any, Dict, List, Tuple, Union
import time


class CustomWandbCallback(WandbCallback):
    """Custom callback for plotting additional values in tensorboard.
    Additionally, this callback can be used to save the model and replay buffer periodically.
    Performs an evaluation at the end of training and uploads the results to wandb.

    Args:
        eval_env: The evaluation environment.
        verbose: Extra terminal outputs.
        model_save_path: Path to save the model regularly.
        model_save_freq: Save the model every x episodes.
        gradient_save_freq: Save the gradients every x episodes.
        save_freq: Save the model and replay buffer every x steps.
        model_file: predefined model file for loading / saving.
        start_episode: Define start episode (if model is loaded).
        additional_log_info_keys: Additionally log these keys from the info dict.
        n_eval_episodes: Number of evaluation episodes.
        deterministic: No noise on action.
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
        eval_env: Union[gym.Env, VecEnv],
        verbose: int = 0,
        model_save_path: str = None,
        model_save_freq: int = 0,
        gradient_save_freq: int = 0,
        save_freq: int = 100,
        model_file: str = "models",
        start_episode: int = 0,
        additional_log_info_keys: List[str] = ["goal_reached"],
        n_eval_episodes: int = 0,
        deterministic: bool = True,
        log_interval: Union[int, Tuple[int, str]] = (1000, "step"),
        # log_path: Optional[str] = None,
    ):  # noqa: D107
        super(CustomWandbCallback, self).__init__(
            verbose, model_save_path, model_save_freq, gradient_save_freq
        )
        self.save_freq = save_freq
        self._n_stored_models = 0
        self.episode_counter = start_episode
        self.additional_log_info_keys = additional_log_info_keys
        self.model_file = model_file
        self.n_eval_episodes = n_eval_episodes
        self.deterministic = deterministic
        self._info_buffer = dict()
        for key in additional_log_info_keys:
            self._info_buffer[key] = []
        self._n_logged_infos = 0

        if isinstance(log_interval, int):
            log_interval = (log_interval, "step")

        if isinstance(log_interval[0], str) and isinstance(log_interval[1], int):
            log_interval = (log_interval[1], log_interval[0])

        self.log_interval = log_interval
        # if log_path is not None:
        #     log_path = os.path.join(log_path, "evaluations")
        # self.log_path = log_path

        # eval results
        self.evaluations_results = []
        self.evaluations_timesteps = []
        self.evaluations_length = []
        # For computing success rate
        self._eval_info_buffer = dict()
        for key in additional_log_info_keys:
            self._eval_info_buffer[key] = []
        # self._eval_results = dict()
        # for key in additional_log_info_keys:
        #     self._eval_results[key] = []

        # We don't have seperate eval environments. You can add this if you need it.
        self.eval_env = eval_env
        if self.n_eval_episodes == 0:
            self.model_evaluated = True
        else:
            self.model_evaluated = False

    def _on_step(self) -> bool:
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
                f"{self.model_file}/model_{save_timestep:_}"  # File format: model_100_000.zip
            )
            if hasattr(self.model, 'save_replay_buffer'):
                self.model.save_replay_buffer(f"{self.model_file}/replay_buffer")

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

    def _on_training_end(self) -> bool:
        """Perform evaluation after the training ends."""
        if self.model_evaluated:
            return True
        # Sync training and eval env if there is VecNormalize
        if self.model.get_vec_normalize_env() is not None:
            try:
                sync_envs_normalization(self.training_env, self.eval_env)
            except AttributeError:
                raise AssertionError(
                    "Training and eval env are not wrapped the same way, "
                    "see https://stable-baselines3.readthedocs.io/en/master/guide/callbacks.html#evalcallback "
                    "and warning above."
                )
        # Reset success rate buffer
        for key in self._eval_info_buffer.keys():
            self._eval_info_buffer[key] = []

        episode_rewards, episode_lengths = evaluate_policy(
            self.model,
            self.eval_env,
            n_eval_episodes=self.n_eval_episodes,
            render=False,
            deterministic=self.deterministic,
            return_episode_rewards=True,
            callback=self._log_success_callback,
        )

        # if self.log_path is not None:
        #     self.evaluations_timesteps.append(self.num_timesteps)
        #     self.evaluations_results.append(episode_rewards)
        #     self.evaluations_length.append(episode_lengths)

        #     # Save success log if present
        #     for key in self._eval_info_buffer.keys():
        #         if len(self._eval_info_buffer[key]) > 0:
        #             self._eval_results[key].append(self._eval_info_buffer[key])

        #     np.savez(
        #         self.log_path,
        #         timesteps=self.evaluations_timesteps,
        #         results=self.evaluations_results,
        #         ep_lengths=self.evaluations_length,
        #         **self._eval_results,
        #     )

        mean_reward, std_reward = np.mean(episode_rewards), np.std(episode_rewards)
        mean_ep_length, std_ep_length = np.mean(episode_lengths), np.std(
            episode_lengths
        )
        self.last_mean_reward = mean_reward

        if self.verbose > 0:
            print(
                f"Eval num_timesteps={self.num_timesteps}, "
                f"episode_reward={mean_reward:.2f} +/- {std_reward:.2f}"
            )
            print(f"Episode length: {mean_ep_length:.2f} +/- {std_ep_length:.2f}")
        # Add to current Logger
        self.logger.record("eval/num_episodes", len(episode_lengths))
        self.logger.record("eval/mean_reward", float(mean_reward))
        self.logger.record("eval/mean_ep_length", mean_ep_length)

        for key in self._eval_info_buffer.keys():
            if len(self._eval_info_buffer[key]) > 0:
                mean_val = np.mean(self._eval_info_buffer[key])
                if self.verbose > 0:
                    print(f"{key} rate: {100 * mean_val:.2f}%")
                self.logger.record(f"eval/{key}", mean_val)

        # Dump log so the evaluation results are printed with the correct timestep
        self.logger.record(
            "time/total_timesteps", self.num_timesteps, exclude="tensorboard"
        )
        self.logger.dump(self.num_timesteps)
        self.model_evaluated = True
        return True
