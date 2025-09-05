"""Kinematics utilities for CBF controllers.

This module provides kinematic computation utilities adapted from the 
CBF safe diffusion package for use in the human-robot-gym framework.
"""

import numpy as np
from typing import Optional


def skew(vector: np.ndarray) -> np.ndarray:
    """Return the skew-symmetric matrix of a 3D vector."""
    return np.array([
        [0, -vector[2], vector[1]],
        [vector[2], 0, -vector[0]],
        [-vector[1], vector[0], 0]
    ])


def unskew(matrix: np.ndarray) -> np.ndarray:
    """Return the vector from a skew-symmetric matrix."""
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]])


def adjoint(T: np.ndarray) -> np.ndarray:
    """Compute the adjoint matrix of a transformation matrix."""
    adj = np.zeros((6, 6))
    R = T[:3, :3]
    p = T[:3, 3]
    adj[:3, :3] = R
    adj[3:, 3:] = R
    adj[:3, 3:] = skew(p) @ R
    return adj


def inverse_transform(T: np.ndarray) -> np.ndarray:
    """Compute inverse of a homogeneous transformation matrix."""
    T_inv = np.eye(4)
    R = T[:3, :3]
    p = T[:3, 3]
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ p
    return T_inv


def exp_screw(screw: np.ndarray, theta: float) -> np.ndarray:
    """Compute matrix exponential of a screw motion."""
    exp_matrix = np.eye(4)
    omega = screw[3:]
    v = screw[:3]
    
    if np.linalg.norm(omega) > 1e-8:
        # Use Rodrigues' formula for rotation matrix
        omega_norm = np.linalg.norm(omega)
        omega_unit = omega / omega_norm
        angle = theta * omega_norm
        exp_matrix[:3, :3] = (np.eye(3) + 
                             np.sin(angle) * skew(omega_unit) + 
                             (1 - np.cos(angle)) * skew(omega_unit) @ skew(omega_unit))
        exp_matrix[:3, 3] = ((theta * np.eye(3) + 
                             (1 - np.cos(theta)) * skew(omega) + 
                             (theta - np.sin(theta)) * skew(omega) @ skew(omega)) @ v).reshape(-1)
    else:
        # Pure translation
        exp_matrix[:3, 3] = theta * v
        
    return exp_matrix


def unskew_screw(screw_matrix: np.ndarray) -> np.ndarray:
    """Extract screw coordinates from skew-symmetric matrix."""
    screw_axis = np.zeros(6)
    screw_axis[:3] = screw_matrix[:3, 3]
    screw_axis[3:] = unskew(screw_matrix[:3, :3])
    return screw_axis


class DHLink:
    """Denavit-Hartenberg link representation."""
    
    def __init__(self, alpha: float, a: float, d: float):
        """Initialize DH link parameters.
        
        Args:
            alpha: Twist angle about x-axis
            a: Link length along x-axis  
            d: Link offset along z-axis
        """
        self.alpha = alpha
        self.a = a
        self.d = d

    def __repr__(self) -> str:
        return f'alpha: {self.alpha}, a: {self.a}, d: {self.d}'
    
    def transformation(self) -> np.ndarray:
        """Calculate the transformation matrix for the link without joint angle."""
        alpha, a, d = self.alpha, self.a, self.d

        R_x_alpha = np.array([
            [1, 0, 0],
            [0, np.cos(alpha), -np.sin(alpha)],
            [0, np.sin(alpha), np.cos(alpha)]
        ])
        T_Rx = np.eye(4)
        T_Rx[:3, :3] = R_x_alpha

        t_x_a = np.array([a, 0, 0])
        T_tx = np.eye(4)
        T_tx[:3, 3] = t_x_a

        t_z_d = np.array([0, 0, d])
        T_tz = np.eye(4)
        T_tz[:3, 3] = t_z_d

        return T_Rx @ T_tx @ T_tz


class DHTable:
    """Denavit-Hartenberg parameter table."""
    
    def __init__(self, dh_list: list):
        """Initialize DH table from list of parameters.
        
        Args:
            dh_list: List of (alpha, a, d) tuples
        """
        self.links = []
        for dh_params in dh_list:
            link = DHLink(*dh_params)
            self.links.append(link)
        self.n = len(self.links)

    def __repr__(self) -> str:
        return '\n'.join([f'Link {i}: {link}' for i, link in enumerate(self.links)])
    
    def transformations(self) -> list:
        """Get transformation matrices for all links."""
        return [link.transformation() for link in self.links]
    
    def transformations_up_to_i(self) -> np.ndarray:
        """Get cumulative transformations up to each joint."""
        Ms = np.zeros((self.n, 4, 4))
        M = np.eye(4)
        for i, link in enumerate(self.links):
            M = M @ link.transformation()
            Ms[i] = M
        return Ms


class CBFKinematics:
    """Kinematic chain for CBF computations using DH parameters."""
    
    def __init__(self, robot_name: str):
        """Initialize kinematic chain for specified robot.
        
        Args:
            robot_name: Name of the robot ('panda', 'schunk', etc.)
        """
        self.robot_name = robot_name.lower()
        
        if self.robot_name in ['panda', 'franka']:
            dh_params = self._get_panda_dh_params()
        elif self.robot_name in ['schunk', 'lbr_iiwa']:
            # Placeholder for Schunk parameters - would need actual DH parameters
            dh_params = self._get_schunk_dh_params()
        else:
            # Default to simplified 6-DOF parameters
            dh_params = self._get_default_dh_params()
            
        self.dh_table = DHTable(dh_params)
        self.n = self.dh_table.n
        
        # Precompute transformations
        self.Ms = self.dh_table.transformations_up_to_i()
        self.Ms_inv = self._compute_inverse_transforms()
        
        # Compute screw axes
        self.screw_axes = self._compute_screw_axes()
    
    def _get_panda_dh_params(self) -> list:
        """Get DH parameters for Franka Panda robot."""
        return [
            (0.0, 0.0, 0.333),        # Joint 1
            (-np.pi/2, 0.0, 0.0),     # Joint 2  
            (np.pi/2, 0.0, 0.316),    # Joint 3
            (np.pi/2, 0.0825, 0.0),   # Joint 4
            (-np.pi/2, -0.0825, 0.384), # Joint 5
            (np.pi/2, 0.0, 0.0),      # Joint 6
            (np.pi/2, 0.088, 0.21)    # Joint 7 + gripper
        ]
    
    def _get_schunk_dh_params(self) -> list:
        """Get DH parameters for Schunk robot (placeholder)."""
        # TODO: Replace with actual Schunk DH parameters
        raise NotImplementedError("DH parameters for schunk not known.")
        return [
            (0.0, 0.0, 0.34),
            (-np.pi/2, 0.0, 0.0),
            (np.pi/2, 0.0, 0.4),
            (np.pi/2, 0.0, 0.0),
            (-np.pi/2, 0.0, 0.4),
            (np.pi/2, 0.0, 0.0),
            (0.0, 0.0, 0.126)
        ]
    
    def _get_default_dh_params(self) -> list:
        """Get default DH parameters for generic 6-DOF robot."""
        raise NotImplementedError("DH parameters for unknown robot not known.")
        return [
            (0.0, 0.0, 0.3),
            (-np.pi/2, 0.0, 0.0),
            (np.pi/2, 0.0, 0.3),
            (np.pi/2, 0.0, 0.0),
            (-np.pi/2, 0.0, 0.3),
            (np.pi/2, 0.0, 0.1)
        ]
    
    def _compute_inverse_transforms(self) -> np.ndarray:
        """Compute inverse of all transformation matrices."""
        Ms_inv = np.zeros((self.n, 4, 4))
        for i in range(self.n):
            Ms_inv[i] = inverse_transform(self.Ms[i])
        return Ms_inv
    
    def _compute_screw_axes(self) -> np.ndarray:
        """Compute screw axes for all joints."""
        screw_axes = np.zeros((self.n, 6))
        
        # Revolute joint around z-axis
        A_matrix = np.zeros((4, 4))
        A_matrix[:3, :3] = skew(np.array([0.0, 0.0, 1.0]))
        
        for i in range(self.n):
            S_matrix = self.Ms[i] @ A_matrix @ self.Ms_inv[i]
            screw_axes[i] = unskew_screw(S_matrix)
            
        return screw_axes
    
    def forward_kinematics(self, joint_angles: np.ndarray, end_link: Optional[int] = None) -> np.ndarray:
        """Compute forward kinematics using screw theory.
        
        Args:
            joint_angles: Joint angles
            end_link: End link index (None for end-effector)
            
        Returns:
            4x4 transformation matrix
        """
        if len(joint_angles) != self.n:
            raise ValueError(f"Expected {self.n} joint angles, got {len(joint_angles)}")
        
        end = self.n if end_link is None else min(end_link + 1, self.n)
        
        T_cumulative = np.eye(4)
        for i in range(end):
            screw_axis = self.screw_axes[i]
            T_cumulative = T_cumulative @ exp_screw(screw_axis, joint_angles[i])
            
        T_cumulative = T_cumulative @ self.Ms[end - 1]
        return T_cumulative
    
    def space_jacobian(self, joint_angles: np.ndarray, end_link: Optional[int] = None) -> np.ndarray:
        """Compute space Jacobian.
        
        Args:
            joint_angles: Joint angles
            end_link: End link index (None for end-effector)
            
        Returns:
            6x(end_link) Jacobian matrix
        """
        if len(joint_angles) != self.n:
            raise ValueError(f"Expected {self.n} joint angles, got {len(joint_angles)}")
        
        end = self.n if end_link is None else min(end_link + 1, self.n)
        Js = np.zeros((6, end))
        
        # First column is just the first screw axis
        Js[:, 0] = self.screw_axes[0, :]
        
        # Compute remaining columns
        T_cumulative = np.eye(4)
        for i in range(1, end):
            # Update cumulative transformation
            prev_screw = self.screw_axes[i-1, :]
            T_i = exp_screw(prev_screw, joint_angles[i-1])
            T_cumulative = T_cumulative @ T_i
            
            # Apply adjoint to current screw axis
            Ad_T = adjoint(T_cumulative)
            Js[:, i] = (Ad_T @ self.screw_axes[i, :]).flatten()
        
        return Js
    
    def compute_jacobian(self, joint_angles: np.ndarray) -> np.ndarray:
        """Compute Jacobian matrix (alias for space_jacobian).
        
        Args:
            joint_angles: Current joint angles
            
        Returns:
            6xN Jacobian matrix
        """
        return self.space_jacobian(joint_angles)