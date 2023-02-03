"""This file contains classes to generate Ornstein-Uhlenbeck noise.

Author:
    Felix Trost (FT)
"""

import numpy as np

class OrnsteinUhlenbeckProcess:
    """
    This class can be used to generate Ornstein-Uhlenbeck noise signals.
    Discretized using Euler-Maruyama method.
    Starting value is the mean (gamma)

    Args:
        size (int): number of processes, 
            determines length of vectors generated by step method
        alpha (float): mean reversion parameter
        beta (float): random shock parameter
        gamma (float): drift parameter
    """
    def __init__(
        self,
        size: int=1,
        alpha: float=0.5,
        beta: float=1,
        gamma: float=0,
    ):
        self._size=size
        self._alpha = alpha
        self._beta = beta
        self._gamma = gamma
        self.y = np.full(self._size, self._gamma, dtype=np.float64)

    def step(self, dt: float)-> np.ndarray:
        """Discrete step to estimate values after a given time interval.

        Args:
            dt (float): time delta

        Returns:
            np.ndarray: new noise values of shape (size,)
        """
        self.y = self.y + (
            self._alpha * (self._gamma - self.y) * dt +
            self._beta * np.sqrt(dt) * np.random.normal(size=self._size)
        )

        return self.y

class ReparameterizedOrnsteinUhlenbeckProcess(OrnsteinUhlenbeckProcess):
    """
    This class re-parameterizes the Ornstein-Uhlenbeck process.
    It generates Ornstein-Uhlenbeck noise signals based on asymptotic mean and variance values.
    Starting value is the mean (mu)
    
    Args:
        alpha (float): rubber band parameter, controls signal convergence speed; alpha > 0
            For higher values of alpha, the distribution of samples approaches the
            normal distribution N(mu, sigma) faster
        mu (float): signal mean value for t -> inf
        sigma (float): signal variance for t -> inf
    """
    def __init__(
        self,
        size: int=1,
        alpha: float=0.5,
        mu: float=0,
        sigma: float=1,
    ):
        super().__init__(
            size=size,
            alpha=alpha,
            beta=sigma*np.sqrt(2*alpha),
            gamma=mu,
        )

