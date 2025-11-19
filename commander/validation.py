"""
Input Validation Helper Module for PAROL6 Commands

Provides consistent validation for robot command parameters to prevent
runtime errors and ensure safe operation.

Author: PAROL6 Team
Date: 2025-01-12
"""

import numpy as np
from typing import Union, List, Tuple, Optional, Any
from constants import (
    MIN_DURATION_S,
    MAX_DURATION_S,
    MIN_SPEED_PERCENTAGE,
    MAX_SPEED_PERCENTAGE,
    PERCENTAGE_MIN,
    PERCENTAGE_MAX,
    POSE_ELEMENT_COUNT,
    JOINT_ANGLE_COUNT,
)


# ============================================================================
# Exception Classes
# ============================================================================

class ValidationError(Exception):
    """
    Raised when command validation fails.

    Contains detailed error information to help debug validation issues.
    """
    def __init__(self, message: str, field: Optional[str] = None, value: Any = None):
        self.field = field
        self.value = value
        super().__init__(message)

    def __str__(self):
        if self.field and self.value is not None:
            return f"ValidationError in '{self.field}': {self.args[0]} (got: {self.value})"
        elif self.field:
            return f"ValidationError in '{self.field}': {self.args[0]}"
        else:
            return f"ValidationError: {self.args[0]}"


# ============================================================================
# Validation Helper Functions
# ============================================================================

class Validator:
    """
    Static validation methods for robot command parameters.

    All methods raise ValidationError on failure and return validated/converted
    values on success.
    """

    # ------------------------------------------------------------------------
    # Pose and Angle Validation
    # ------------------------------------------------------------------------

    @staticmethod
    def validate_pose(pose: Any, name: str = "pose") -> List[float]:
        """
        Validate 6-element pose [x, y, z, rx, ry, rz].

        Args:
            pose: Input pose (list, tuple, or numpy array)
            name: Name of the parameter (for error messages)

        Returns:
            List of 6 float values

        Raises:
            ValidationError: If pose is invalid

        Example:
            >>> pose = Validator.validate_pose([300, 0, 400, 0, 90, 0], "target_pose")
        """
        if pose is None:
            raise ValidationError(f"{name} cannot be None", field=name, value=pose)

        if not isinstance(pose, (list, tuple, np.ndarray)):
            raise ValidationError(
                f"{name} must be a list, tuple, or array",
                field=name,
                value=type(pose).__name__
            )

        if len(pose) != POSE_ELEMENT_COUNT:
            raise ValidationError(
                f"{name} must have {POSE_ELEMENT_COUNT} elements [x,y,z,rx,ry,rz], got {len(pose)}",
                field=name,
                value=len(pose)
            )

        try:
            return [float(x) for x in pose]
        except (ValueError, TypeError) as e:
            raise ValidationError(
                f"{name} elements must be numeric",
                field=name,
                value=pose
            )

    @staticmethod
    def validate_joint_angles(angles: Any, name: str = "angles") -> List[float]:
        """
        Validate 6-element joint angles array.

        Args:
            angles: Input angles (list, tuple, or numpy array)
            name: Name of the parameter (for error messages)

        Returns:
            List of 6 float values

        Raises:
            ValidationError: If angles are invalid

        Example:
            >>> angles = Validator.validate_joint_angles([0, -45, 90, 0, 45, 0])
        """
        if angles is None:
            raise ValidationError(f"{name} cannot be None", field=name, value=angles)

        if not isinstance(angles, (list, tuple, np.ndarray)):
            raise ValidationError(
                f"{name} must be a list, tuple, or array",
                field=name,
                value=type(angles).__name__
            )

        if len(angles) != JOINT_ANGLE_COUNT:
            raise ValidationError(
                f"{name} must have {JOINT_ANGLE_COUNT} elements (J1-J6), got {len(angles)}",
                field=name,
                value=len(angles)
            )

        try:
            return [float(x) for x in angles]
        except (ValueError, TypeError) as e:
            raise ValidationError(
                f"{name} elements must be numeric",
                field=name,
                value=angles
            )

    @staticmethod
    def validate_joint_index(index: int, name: str = "joint_index") -> int:
        """
        Validate joint index (0-5).

        Args:
            index: Joint index (0-based)
            name: Name of the parameter (for error messages)

        Returns:
            Validated integer index

        Raises:
            ValidationError: If index is invalid

        Example:
            >>> joint_idx = Validator.validate_joint_index(2)  # Joint 3 (0-indexed)
        """
        try:
            index = int(index)
        except (ValueError, TypeError):
            raise ValidationError(
                f"{name} must be an integer",
                field=name,
                value=index
            )

        if not (0 <= index < JOINT_ANGLE_COUNT):
            raise ValidationError(
                f"{name} must be between 0 and {JOINT_ANGLE_COUNT-1}, got {index}",
                field=name,
                value=index
            )

        return index

    # ------------------------------------------------------------------------
    # Duration and Time Validation
    # ------------------------------------------------------------------------

    @staticmethod
    def validate_duration(duration: Any, name: str = "duration",
                         min_val: float = MIN_DURATION_S,
                         max_val: float = MAX_DURATION_S,
                         allow_none: bool = False) -> Optional[float]:
        """
        Validate duration is a positive number within range.

        Args:
            duration: Input duration in seconds
            name: Name of the parameter (for error messages)
            min_val: Minimum allowed duration (default: 0.001s)
            max_val: Maximum allowed duration (default: 3600s)
            allow_none: Whether None is acceptable

        Returns:
            Validated float duration or None (if allow_none=True)

        Raises:
            ValidationError: If duration is invalid

        Example:
            >>> duration = Validator.validate_duration(2.5, "move_duration")
        """
        if duration is None:
            if allow_none:
                return None
            else:
                raise ValidationError(
                    f"{name} cannot be None",
                    field=name,
                    value=duration
                )

        try:
            duration = float(duration)
        except (ValueError, TypeError):
            raise ValidationError(
                f"{name} must be a number",
                field=name,
                value=duration
            )

        if duration < min_val:
            raise ValidationError(
                f"{name} must be >= {min_val}s, got {duration}s",
                field=name,
                value=duration
            )

        if duration > max_val:
            raise ValidationError(
                f"{name} must be <= {max_val}s, got {duration}s",
                field=name,
                value=duration
            )

        return duration

    # ------------------------------------------------------------------------
    # Percentage and Speed Validation
    # ------------------------------------------------------------------------

    @staticmethod
    def validate_percentage(value: Any, name: str = "percentage",
                           min_val: float = PERCENTAGE_MIN,
                           max_val: float = PERCENTAGE_MAX,
                           allow_none: bool = False) -> Optional[float]:
        """
        Validate percentage is in range [0, 100].

        Args:
            value: Input percentage value
            name: Name of the parameter (for error messages)
            min_val: Minimum percentage (default: 0)
            max_val: Maximum percentage (default: 100)
            allow_none: Whether None is acceptable

        Returns:
            Validated float percentage or None (if allow_none=True)

        Raises:
            ValidationError: If percentage is invalid

        Example:
            >>> speed = Validator.validate_percentage(75, "speed_percentage")
        """
        if value is None:
            if allow_none:
                return None
            else:
                raise ValidationError(
                    f"{name} cannot be None",
                    field=name,
                    value=value
                )

        try:
            value = float(value)
        except (ValueError, TypeError):
            raise ValidationError(
                f"{name} must be a number",
                field=name,
                value=value
            )

        if not (min_val <= value <= max_val):
            raise ValidationError(
                f"{name} must be between {min_val} and {max_val}, got {value}",
                field=name,
                value=value
            )

        return value

    @staticmethod
    def validate_speed_percentage(value: Any, name: str = "speed_percentage",
                                  allow_none: bool = False) -> Optional[float]:
        """
        Validate speed percentage is in range [0, 100].

        Convenience wrapper around validate_percentage with speed-specific defaults.

        Args:
            value: Input speed percentage
            name: Name of the parameter (for error messages)
            allow_none: Whether None is acceptable

        Returns:
            Validated float speed percentage or None (if allow_none=True)

        Raises:
            ValidationError: If speed percentage is invalid

        Example:
            >>> speed = Validator.validate_speed_percentage(50)
        """
        return Validator.validate_percentage(
            value,
            name=name,
            min_val=MIN_SPEED_PERCENTAGE,
            max_val=MAX_SPEED_PERCENTAGE,
            allow_none=allow_none
        )

    # ------------------------------------------------------------------------
    # List and Array Validation
    # ------------------------------------------------------------------------

    @staticmethod
    def validate_list(value: Any, name: str = "list",
                     element_type: type = float,
                     min_length: Optional[int] = None,
                     max_length: Optional[int] = None,
                     allow_empty: bool = False) -> List:
        """
        Validate a list with optional length and element type constraints.

        Args:
            value: Input list
            name: Name of the parameter (for error messages)
            element_type: Expected type of list elements
            min_length: Minimum list length (optional)
            max_length: Maximum list length (optional)
            allow_empty: Whether empty lists are allowed

        Returns:
            Validated list with converted element types

        Raises:
            ValidationError: If list is invalid

        Example:
            >>> joints = Validator.validate_list([1, 2, 3], "joints", int, min_length=1)
        """
        if not isinstance(value, (list, tuple, np.ndarray)):
            raise ValidationError(
                f"{name} must be a list, tuple, or array",
                field=name,
                value=type(value).__name__
            )

        if not allow_empty and len(value) == 0:
            raise ValidationError(
                f"{name} cannot be empty",
                field=name,
                value=value
            )

        if min_length is not None and len(value) < min_length:
            raise ValidationError(
                f"{name} must have at least {min_length} elements, got {len(value)}",
                field=name,
                value=len(value)
            )

        if max_length is not None and len(value) > max_length:
            raise ValidationError(
                f"{name} must have at most {max_length} elements, got {len(value)}",
                field=name,
                value=len(value)
            )

        try:
            return [element_type(x) for x in value]
        except (ValueError, TypeError) as e:
            raise ValidationError(
                f"{name} elements must be convertible to {element_type.__name__}",
                field=name,
                value=value
            )

    # ------------------------------------------------------------------------
    # String and Enum Validation
    # ------------------------------------------------------------------------

    @staticmethod
    def validate_choice(value: Any, choices: List[Any], name: str = "value",
                       case_sensitive: bool = True) -> Any:
        """
        Validate value is one of the allowed choices.

        Args:
            value: Input value
            choices: List of allowed values
            name: Name of the parameter (for error messages)
            case_sensitive: Whether string comparison is case-sensitive

        Returns:
            Validated value (from choices list)

        Raises:
            ValidationError: If value is not in choices

        Example:
            >>> frame = Validator.validate_choice("WRF", ["WRF", "TRF"], "reference_frame")
        """
        if not case_sensitive and isinstance(value, str):
            value_compare = value.upper()
            choices_compare = [c.upper() if isinstance(c, str) else c for c in choices]
            if value_compare in choices_compare:
                # Return the original choice that matches
                idx = choices_compare.index(value_compare)
                return choices[idx]
        else:
            if value in choices:
                return value

        raise ValidationError(
            f"{name} must be one of {choices}, got '{value}'",
            field=name,
            value=value
        )

    # ------------------------------------------------------------------------
    # Numeric Range Validation
    # ------------------------------------------------------------------------

    @staticmethod
    def validate_range(value: Any, name: str = "value",
                      min_val: Optional[float] = None,
                      max_val: Optional[float] = None,
                      allow_none: bool = False) -> Optional[float]:
        """
        Validate numeric value is within specified range.

        Args:
            value: Input numeric value
            name: Name of the parameter (for error messages)
            min_val: Minimum allowed value (inclusive, optional)
            max_val: Maximum allowed value (inclusive, optional)
            allow_none: Whether None is acceptable

        Returns:
            Validated float value or None (if allow_none=True)

        Raises:
            ValidationError: If value is out of range

        Example:
            >>> current = Validator.validate_range(500, "gripper_current",
            ...                                    min_val=100, max_val=1000)
        """
        if value is None:
            if allow_none:
                return None
            else:
                raise ValidationError(
                    f"{name} cannot be None",
                    field=name,
                    value=value
                )

        try:
            value = float(value)
        except (ValueError, TypeError):
            raise ValidationError(
                f"{name} must be a number",
                field=name,
                value=value
            )

        if min_val is not None and value < min_val:
            raise ValidationError(
                f"{name} must be >= {min_val}, got {value}",
                field=name,
                value=value
            )

        if max_val is not None and value > max_val:
            raise ValidationError(
                f"{name} must be <= {max_val}, got {value}",
                field=name,
                value=value
            )

        return value

    # ------------------------------------------------------------------------
    # Boolean Validation
    # ------------------------------------------------------------------------

    @staticmethod
    def validate_bool(value: Any, name: str = "value") -> bool:
        """
        Validate and convert value to boolean.

        Args:
            value: Input value (bool, int, str)
            name: Name of the parameter (for error messages)

        Returns:
            Boolean value

        Raises:
            ValidationError: If value cannot be converted to bool

        Example:
            >>> enabled = Validator.validate_bool("true", "motor_enabled")
        """
        if isinstance(value, bool):
            return value

        if isinstance(value, (int, float)):
            return bool(value)

        if isinstance(value, str):
            value_lower = value.lower()
            if value_lower in ('true', 'yes', '1', 'on'):
                return True
            elif value_lower in ('false', 'no', '0', 'off'):
                return False

        raise ValidationError(
            f"{name} must be a boolean value",
            field=name,
            value=value
        )


# ============================================================================
# Convenience Functions
# ============================================================================

def validate_move_joint_params(angles: Any, duration: Any = None,
                               speed_percentage: Any = None) -> Tuple[List[float], Optional[float], Optional[float]]:
    """
    Validate MoveJoint command parameters.

    Args:
        angles: Target joint angles [J1-J6]
        duration: Move duration in seconds (optional)
        speed_percentage: Speed percentage 0-100 (optional)

    Returns:
        Tuple of (validated_angles, validated_duration, validated_speed_percentage)

    Raises:
        ValidationError: If any parameter is invalid
    """
    angles = Validator.validate_joint_angles(angles, "target_angles")
    duration = Validator.validate_duration(duration, "duration", allow_none=True)
    speed_percentage = Validator.validate_speed_percentage(speed_percentage, allow_none=True)

    return angles, duration, speed_percentage


def validate_move_pose_params(pose: Any, duration: Any = None,
                              velocity_percent: Any = None) -> Tuple[List[float], Optional[float], Optional[float]]:
    """
    Validate MovePose command parameters.

    Args:
        pose: Target pose [x, y, z, rx, ry, rz]
        duration: Move duration in seconds (optional)
        velocity_percent: Velocity percentage 0-100 (optional)

    Returns:
        Tuple of (validated_pose, validated_duration, validated_velocity_percent)

    Raises:
        ValidationError: If any parameter is invalid
    """
    pose = Validator.validate_pose(pose, "target_pose")
    duration = Validator.validate_duration(duration, "duration", allow_none=True)
    velocity_percent = Validator.validate_percentage(velocity_percent, "velocity_percent", allow_none=True)

    return pose, duration, velocity_percent


# ============================================================================
# Module Metadata
# ============================================================================

__version__ = "1.0.0"
__author__ = "PAROL6 Team"
__date__ = "2025-01-12"
__description__ = "Input validation utilities for PAROL6 robot command parameters"
