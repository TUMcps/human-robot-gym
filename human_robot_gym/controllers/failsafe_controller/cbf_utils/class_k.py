"""Class K functions for Control Barrier Functions.

This module implements Class K functions that are used in CBF constraints
to ensure safety while maintaining performance.
"""

import numpy as np
from typing import Dict, Callable, Union, Literal


class MonotonicFunction:
    """A class for monotonic functions that only cover the positive real line.
    
    Supports linear, quadratic, exponential, logarithmic, and tanh functions.
    """

    FunctionType = Literal['linear', 'quadratic', 'exponential', 'logarithmic', 'tanh']
    
    def __init__(self, function_type: FunctionType, scale: float) -> None:
        """Initialize monotonic function.
        
        Args:
            function_type: Type of monotonic function
            scale: Positive scaling factor
        """
        self.function_type = function_type
        self.scale = scale
        assert self.scale > 0, "Scale must be positive"
        
        # Define the function implementations
        self._functions: Dict[FunctionType, Callable[[float], float]] = {
            'linear': lambda x: self.scale * x,
            'quadratic': lambda x: self.scale * x**2,
            'exponential': lambda x: self.scale * np.exp(x),
            'logarithmic': lambda x: self.scale * np.log(x + 1),  # Using x + 1 to ensure f(0) = 0
            'tanh': lambda x: self.scale * np.tanh(x)
        }
        
        if function_type not in self._functions:
            raise ValueError(f"Unsupported function type: {function_type}")

    def __call__(self, x: float) -> float:
        """Evaluate the function at x."""
        return self._functions[self.function_type](x)


class ClassKFunction:
    """A class for Class K functions.
    
    Class K functions are functions that are monotonically increasing 
    and pass through the origin. Therefore, they can be modelled as a 
    composition of two monotonic functions for positive and negative domains.
    """

    def __init__(self, func_pos_domain: MonotonicFunction, func_neg_domain: MonotonicFunction) -> None:
        """Initialize Class K function.
        
        Args:
            func_pos_domain: Monotonic function for positive domain
            func_neg_domain: Monotonic function for negative domain
        """
        self.func_pos_domain = func_pos_domain
        self.func_neg_domain = func_neg_domain

    def __call__(self, x: float) -> float:
        """Evaluate the Class K function at x."""
        if x >= 0.0:
            return self.func_pos_domain(x)
        else:
            return -self.func_neg_domain(-x)