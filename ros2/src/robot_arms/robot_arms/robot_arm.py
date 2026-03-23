#!/usr/bin/env python3
"""
Robot Arm Base Class

This module provides an abstract base class for robot arm controllers.
All robot arm implementations should inherit from this class and implement
the required abstract methods to ensure standardized interfaces.
"""

from abc import ABC, abstractmethod
from typing import List, Optional

class RobotArm(ABC):
    """
    Abstract base class for robot arm controllers.
    
    This class defines the standard interface that all robot arm implementations
    must follow. Subclasses should implement all abstract methods to provide
    hardware-specific functionality while maintaining a consistent API.
    
    All joint positions are expected to be in radians.
    All end effector poses are TBD. # TODO: @Mohit define pose format
    """
    
    @abstractmethod
    def connect(self) -> None:
        """
        Establish connection to the robot arm hardware.
        
        Raises:
            RuntimeError: If connection fails
        """
        raise NotImplementedError("Subclasses must implement connect")
    
    @abstractmethod
    def disconnect(self) -> None:
        """
        Disconnect from the robot arm hardware.
        
        Raises:
            RuntimeError: If disconnection fails
        """
        raise NotImplementedError("Subclasses must implement disconnect")
    
    @abstractmethod
    def move_j(
        self,
        joint_positions: List[float],
        velocity: Optional[float] = None,
        blocking: bool = False
    ) -> None:
        """
        Move the robot arm to the specified joint positions.
        
        Args:
            joint_positions: List of joint angles in radians [q1, q2, ..., qn]
            velocity: Optional velocity parameter (units depend on implementation)
            blocking: If True, wait until movement is complete before returning
        
        Raises:
            ValueError: If joint_positions length doesn't match number of joints
            RuntimeError: If movement fails or is unsafe
        """
        raise NotImplementedError("Subclasses must implement move_j")
    
    @abstractmethod
    def move_l(
        self,
        pose: List[float],
        velocity: Optional[float] = None,
        blocking: bool = False
    ) -> None:
        """
        Move the robot arm end effector to the specified Cartesian pose.
        
        Args:
            pose: TBD: Desired end effector pose as [x, y, z, rx, ry, rz]
            velocity: Optional velocity parameter (units depend on implementation)
            blocking: If True, wait until movement is complete before returning
        
        Raises:
            ValueError: If pose length is not 6
            RuntimeError: If pose is unreachable or movement fails
        """
        raise NotImplementedError("Subclasses must implement move_l")
    
    @abstractmethod
    def get_joint_positions(self) -> List[float]:
        """
        Get the current joint positions of the robot arm.
        
        Returns:
            List of joint angles in radians [q1, q2, ..., qn]
        """
        raise NotImplementedError("Subclasses must implement get_joint_positions")
    
    @abstractmethod
    def get_end_effector_pose(self) -> List[float]:
        """
        Get the current end effector pose.
        
        Returns:
            End effector pose as [x, y, z, rx, ry, rz]
            - x, y, z: position in meters
            - rx, ry, rz: orientation as Euler angles in radians
        """
        raise NotImplementedError("Subclasses must implement get_end_effector_pose")
    
    @abstractmethod
    def move_to_home_position(
        self,
        velocity: Optional[float] = None,
        blocking: bool = True
    ) -> None:
        """
        Move the robot arm to its home/zero position.
        
        Args:
            velocity: Optional velocity parameter (units depend on implementation)
            blocking: If True, wait until movement is complete before returning
        
        Raises:
            RuntimeError: If movement fails
        """
        raise NotImplementedError("Subclasses must implement move_to_home_position")


    """Future methods for advanced control and kinematics"""


    def get_joint_velocities(self) -> List[float]:
        """
        Get the current joint velocities of the robot arm.
        
        Returns:
            List of joint velocities in rad/s [dq1, dq2, ..., dqn]
        """
        raise NotImplementedError("Subclasses must implement get_joint_velocities")

    def compute_forward_kinematics(
        self,
        joint_positions: List[float]
    ) -> List[float]:
        """
        Compute the forward kinematics for given joint positions.
        
        Args:
            joint_positions: List of joint angles in radians [q1, q2, ..., qn]
        
        Returns: TDB
        """
        raise NotImplementedError("Subclasses must implement compute_forward_kinematics")
    
    def compute_inverse_kinematics(
        self,
        pose: List[float],
        initial_joint_positions: Optional[List[float]] = None
    ) -> Optional[List[float]]:
        """
        Compute the inverse kinematics for a given end effector pose.
        
        Args:
            pose: Desired end effector pose as [x, y, z, rx, ry, rz]
                  - x, y, z: position in meters
                  - rx, ry, rz: orientation as Euler angles in radians
            initial_joint_positions: Optional initial guess for joint positions
                                     in radians. If None, uses current joint positions.
        
        Returns:
            List of joint angles in radians [q1, q2, ..., qn] if solution found,
            None if no solution exists
        
        Raises:
            ValueError: If pose length is not 6
        """
        raise NotImplementedError("Subclasses must implement compute_inverse_kinematics")
    

