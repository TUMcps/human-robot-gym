"""This file describes the cantor pairing function for hashing two integers into one."""


def cantor_pairing(a: int, b: int) -> int:
    """Cantor pairing function.

    hash = (a + b) * (a + b + 1) / 2 + b

    Args:
        a (int): first integer
        b (int): second integer

    Returns:
        int: the cantor pairing of a and b
    """
    return (a + b) * (a + b + 1) // 2 + b
