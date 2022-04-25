import numpy as np


def q_pin(arr):
    """
    The pinocchio model has an additional fictional joint.
    This function adds a 0.0 to the given array.

    Args:
        arr (np.array)

    Returns:
        np.array
    """
    return np.append(arr, 0.0)
