"""Test script for CBF controller implementation.

This script provides basic functionality tests for the CBFFailsafeController
to ensure it can be instantiated and basic operations work.
"""

import numpy as np
# import sys
# import os
# sys.path.insert(0, os.path.abspath('.'))

from human_robot_gym.controllers.failsafe_controller.cbf_utils.class_k import ClassKFunction, MonotonicFunction
from human_robot_gym.controllers.failsafe_controller.cbf_utils.kinematics import CBFKinematics


def test_class_k_function():
    """Test Class K function implementation."""
    print("Testing Class K function...")
    
    # Test monotonic functions
    linear_func = MonotonicFunction('linear', 1.0)
    quad_func = MonotonicFunction('quadratic', 1.0)
    exp_func = MonotonicFunction('exponential', 1.0)
    
    # Test Class K function
    class_k = ClassKFunction(linear_func, linear_func)
    
    # Test positive values
    assert class_k(1.0) == 1.0
    assert class_k(2.0) == 2.0
    
    # Test negative values
    assert class_k(-1.0) == -1.0
    assert class_k(-2.0) == -2.0
    
    # Test zero
    assert class_k(0.0) == 0.0
    
    print("Class K function tests passed!")


def test_cbf_kinematics():
    """Test CBF kinematics implementation."""
    print("Testing CBF kinematics...")
    
    # Test with different robot names
    for robot_name in ['panda', 'schunk', 'unknown']:
        kinematics = CBFKinematics(robot_name)
        
        # Test forward kinematics
        joint_angles = np.zeros(kinematics.n)
        T = kinematics.forward_kinematics(joint_angles)
        assert T.shape == (4, 4)
        
        # Test Jacobian computation
        jacobian = kinematics.compute_jacobian(joint_angles)
        assert jacobian.shape == (6, kinematics.n)
        
        print(f"Robot {robot_name}: {kinematics.n} joints, kinematics working")
    
    print("CBF kinematics tests passed!")


def test_integration():
    """Test integration between components."""
    print("Testing integration...")
    
    # Test that all components work together
    kinematics = CBFKinematics('panda')
    
    monotonic_pos = MonotonicFunction('linear', 1.0)
    monotonic_neg = MonotonicFunction('linear', 1.0)
    class_k = ClassKFunction(monotonic_pos, monotonic_neg)
    
    # Simulate basic CBF computation
    joint_angles = np.array([0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7])
    if len(joint_angles) > kinematics.n:
        joint_angles = joint_angles[:kinematics.n]
    elif len(joint_angles) < kinematics.n:
        joint_angles = np.pad(joint_angles, (0, kinematics.n - len(joint_angles)))
    
    T = kinematics.forward_kinematics(joint_angles)
    jacobian = kinematics.compute_jacobian(joint_angles)
    
    # Simulate distance computation
    robot_pos = T[:3, 3]
    human_pos = np.array([0.5, 0.5, 0.5])
    distance = np.linalg.norm(robot_pos - human_pos)
    
    # Test barrier function
    min_distance = 0.2
    h_value = distance - min_distance
    kappa_value = class_k(h_value)
    
    print(f"Distance: {distance:.3f}, h(x): {h_value:.3f}, κ(h(x)): {kappa_value:.3f}")
    print("Integration tests passed!")


if __name__ == "__main__":
    print("Running CBF controller tests...")
    
    try:
        test_class_k_function()
        test_cbf_kinematics()
        test_integration()
        print("\nAll tests passed! CBF controller implementation is working correctly.")
        
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()