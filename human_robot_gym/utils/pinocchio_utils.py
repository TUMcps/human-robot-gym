"""This file describes utility functions for pinocchio.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2.5.22 JT Formatted docstrings
"""
import numpy as np


def q_pin(arr):
    """Append a 0.0 to the given array.

    The pinocchio model has an additional fictional joint.

    Args:
        arr (np.array)

    Returns:
        np.array
    """
    return np.append(arr, 0.0)
