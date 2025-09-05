"""Factory for creating safety controllers.

This module provides a factory function to create the appropriate safety controller
based on the shield_type parameter. It encapsulates the controller selection logic
and provides a clean interface for the environment.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2025-09-05 JT Initial implementation
"""

from human_robot_gym.controllers.failsafe_controller.failsafe_controller.sara_shield_controller import SaraShieldController

try:
    from human_robot_gym.controllers.failsafe_controller.failsafe_controller.cbf_safety_controller import CBFSafetyController, CASADI_AVAILABLE
    CBF_AVAILABLE = CASADI_AVAILABLE
except ImportError:
    CBF_AVAILABLE = False


def create_safety_controller(shield_type, **kwargs):
    """Create a safety controller based on the shield type.
    
    Args:
        shield_type (str): Type of safety shield to use:
            - "SSM": SARA shield with Smooth Safe Motion
            - "PFL": SARA shield with Potential Field Limiter  
            - "CBF": Control Barrier Function controller
            - "OFF": SARA shield with safety disabled
        **kwargs: Additional arguments passed to the controller
        
    Returns:
        SafetyController: Instance of the appropriate safety controller
        
    Raises:
        ValueError: If shield_type is not recognized
    """
    if shield_type in ["SSM", "PFL", "OFF"]:
        # Use SARA shield controller for these types
        return SaraShieldController(shield_type=shield_type, **kwargs)
    elif shield_type == "CBF":
        # Use CBF safety controller
        if not CBF_AVAILABLE:
            raise ImportError(
                "CBF safety controller is not available. CasADi is required for CBF functionality. "
                "Please install it with: pip install casadi or pip install -e .[cbf]"
            )
        return CBFSafetyController(shield_type=shield_type, **kwargs)
    else:
        available_types = get_supported_shield_types()
        raise ValueError(f"Unknown shield type: {shield_type}. "
                        f"Supported types are: {', '.join(available_types)}")


def get_supported_shield_types():
    """Get list of supported shield types.
    
    Returns:
        list: List of supported shield type strings
    """
    base_types = ["SSM", "PFL", "OFF"]
    if CBF_AVAILABLE:
        base_types.append("CBF")
    return base_types


def is_shield_type_supported(shield_type):
    """Check if a shield type is supported.
    
    Args:
        shield_type (str): Shield type to check
        
    Returns:
        bool: True if supported, False otherwise
    """
    return shield_type in get_supported_shield_types()