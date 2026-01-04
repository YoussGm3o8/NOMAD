"""
Mission Validator for NOMAD.

Validates EKF configuration and mission parameters before flight.
Critical safety check to prevent GPS/VIO mismatch during flight.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from enum import IntEnum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .mavlink_interface import MavlinkService
    from .state import StateManager


logger = logging.getLogger(__name__)


# ============================================================
# EKF Source Definitions (ArduPilot EKF3)
# ============================================================


class EKFSourcePosXY(IntEnum):
    """EKF3_SRC1_POSXY parameter values."""
    
    NONE = 0
    GPS = 3
    BEACON = 4
    OPTICAL_FLOW = 5
    EXTERNAL_NAV = 6  # VIO / Visual-Inertial Odometry
    WHEEL_ENCODER = 7


class EKFSourceVelXY(IntEnum):
    """EKF3_SRC1_VELXY parameter values."""
    
    NONE = 0
    GPS = 3
    OPTICAL_FLOW = 5
    EXTERNAL_NAV = 6  # VIO
    WHEEL_ENCODER = 7


class EKFSourceYaw(IntEnum):
    """EKF3_SRC1_YAW parameter values."""
    
    NONE = 0
    COMPASS = 1
    GPS = 2
    GPS_COMPASS_FALLBACK = 3
    EXTERNAL_NAV = 6  # VIO
    GSF = 8  # Gaussian Sum Filter


class EKFMode(IntEnum):
    """High-level EKF mode for mission tasks."""
    
    GPS = 1      # Task 1: Outdoor GPS navigation
    VIO = 2      # Task 2: Indoor Visual-Inertial Odometry


@dataclass
class EKFStatus:
    """Current EKF configuration status."""
    
    posxy_source: int
    velxy_source: int | None = None
    yaw_source: int | None = None
    mode: EKFMode | None = None
    mismatch: bool = False
    warning: str = ""
    
    @property
    def is_gps_mode(self) -> bool:
        """Check if EKF is configured for GPS navigation."""
        return self.posxy_source == EKFSourcePosXY.GPS
    
    @property
    def is_vio_mode(self) -> bool:
        """Check if EKF is configured for VIO/ExternalNav."""
        return self.posxy_source == EKFSourcePosXY.EXTERNAL_NAV


@dataclass
class ValidationResult:
    """Result of mission validation check."""
    
    valid: bool
    warnings: list[str]
    errors: list[str]
    ekf_status: EKFStatus | None = None
    
    @property
    def is_critical(self) -> bool:
        """Check if there are critical errors."""
        return len(self.errors) > 0


# ============================================================
# EKF Validator
# ============================================================


class EKFValidator:
    """
    Validates EKF configuration against expected mission mode.
    
    Critical safety check: Using GPS mode indoors (Task 2) or
    VIO mode outdoors (Task 1) can cause loss of control.
    """
    
    # Parameter names
    PARAM_POSXY = "EKF3_SRC1_POSXY"
    PARAM_VELXY = "EKF3_SRC1_VELXY"
    PARAM_YAW = "EKF3_SRC1_YAW"
    
    def __init__(
        self,
        mavlink_service: "MavlinkService",
        state_manager: "StateManager | None" = None,
    ) -> None:
        """
        Initialize EKF validator.
        
        Args:
            mavlink_service: MAVLink service for parameter queries
            state_manager: Optional state manager for status updates
        """
        self._mavlink = mavlink_service
        self._state = state_manager
        self._last_status: EKFStatus | None = None
    
    async def check_ekf_source(
        self,
        expected_mode: EKFMode,
        timeout: float = 5.0,
    ) -> EKFStatus:
        """
        Query the Flight Controller for EKF source configuration.
        
        Checks EKF3_SRC1_POSXY against the expected mode and
        returns a warning if there's a mismatch.
        
        Args:
            expected_mode: Expected EKF mode (GPS or VIO)
            timeout: Timeout for parameter query in seconds
        
        Returns:
            EKFStatus with current configuration and any warnings
        
        Example:
            >>> status = await validator.check_ekf_source(EKFMode.VIO)
            >>> if status.mismatch:
            ...     print(f"WARNING: {status.warning}")
        """
        try:
            # Query EKF3_SRC1_POSXY from flight controller
            posxy_value = await self._query_parameter(self.PARAM_POSXY, timeout)
            
            if posxy_value is None:
                return EKFStatus(
                    posxy_source=-1,
                    mismatch=True,
                    warning="EKF QUERY FAILED: Could not read EKF3_SRC1_POSXY"
                )
            
            # Determine current mode
            posxy_int = int(posxy_value)
            current_mode: EKFMode | None = None
            
            if posxy_int == EKFSourcePosXY.GPS:
                current_mode = EKFMode.GPS
            elif posxy_int == EKFSourcePosXY.EXTERNAL_NAV:
                current_mode = EKFMode.VIO
            
            # Check for mismatch
            mismatch = False
            warning = ""
            
            if expected_mode == EKFMode.GPS and posxy_int != EKFSourcePosXY.GPS:
                mismatch = True
                warning = (
                    f"EKF MISMATCH: DANGEROUS - Expected GPS (3) but got {posxy_int}. "
                    f"Task 1 requires GPS navigation. VIO will not work outdoors!"
                )
            elif expected_mode == EKFMode.VIO and posxy_int != EKFSourcePosXY.EXTERNAL_NAV:
                mismatch = True
                warning = (
                    f"EKF MISMATCH: DANGEROUS - Expected VIO/ExternalNav (6) but got {posxy_int}. "
                    f"Task 2 requires VIO. GPS will not work indoors!"
                )
            
            status = EKFStatus(
                posxy_source=posxy_int,
                mode=current_mode,
                mismatch=mismatch,
                warning=warning,
            )
            
            # Log and update state
            if mismatch:
                logger.critical(warning)
                if self._state:
                    await self._state.update_system_state(
                        warning=warning,
                        ekf_posxy_source=posxy_int,
                    )
            else:
                logger.info(
                    f"EKF source validated: POSXY={posxy_int} matches "
                    f"expected mode {expected_mode.name}"
                )
            
            self._last_status = status
            return status
            
        except Exception as e:
            logger.error(f"EKF validation error: {e}")
            return EKFStatus(
                posxy_source=-1,
                mismatch=True,
                warning=f"EKF VALIDATION ERROR: {e}"
            )
    
    async def _query_parameter(
        self,
        param_name: str,
        timeout: float = 5.0,
    ) -> float | None:
        """
        Query a parameter value from the flight controller.
        
        Args:
            param_name: Name of the parameter to query
            timeout: Timeout in seconds
        
        Returns:
            Parameter value or None if query failed
        """
        try:
            # Use MAVLink PARAM_REQUEST_READ message
            # The response comes via PARAM_VALUE message
            return await asyncio.wait_for(
                self._mavlink.get_parameter(param_name),
                timeout=timeout,
            )
        except asyncio.TimeoutError:
            logger.warning(f"Parameter query timeout: {param_name}")
            return None
        except Exception as e:
            logger.error(f"Parameter query error for {param_name}: {e}")
            return None
    
    @property
    def last_status(self) -> EKFStatus | None:
        """Get the last EKF status check result."""
        return self._last_status


# ============================================================
# Mission Validator
# ============================================================


class MissionValidator:
    """
    Comprehensive mission validation before flight.
    
    Validates:
    - EKF source configuration
    - Sensor health (future)
    - Battery status (future)
    - Geofence configuration (future)
    """
    
    def __init__(
        self,
        mavlink_service: "MavlinkService",
        state_manager: "StateManager | None" = None,
    ) -> None:
        """
        Initialize mission validator.
        
        Args:
            mavlink_service: MAVLink service for FC communication
            state_manager: Optional state manager for status updates
        """
        self._ekf_validator = EKFValidator(mavlink_service, state_manager)
        self._state = state_manager
    
    async def validate_task1(self) -> ValidationResult:
        """
        Validate configuration for Task 1 (Outdoor/GPS).
        
        Returns:
            ValidationResult with any warnings or errors
        """
        warnings = []
        errors = []
        
        # Check EKF source
        ekf_status = await self._ekf_validator.check_ekf_source(EKFMode.GPS)
        
        if ekf_status.mismatch:
            errors.append(ekf_status.warning)
        
        return ValidationResult(
            valid=len(errors) == 0,
            warnings=warnings,
            errors=errors,
            ekf_status=ekf_status,
        )
    
    async def validate_task2(self) -> ValidationResult:
        """
        Validate configuration for Task 2 (Indoor/VIO).
        
        Returns:
            ValidationResult with any warnings or errors
        """
        warnings = []
        errors = []
        
        # Check EKF source
        ekf_status = await self._ekf_validator.check_ekf_source(EKFMode.VIO)
        
        if ekf_status.mismatch:
            errors.append(ekf_status.warning)
        
        return ValidationResult(
            valid=len(errors) == 0,
            warnings=warnings,
            errors=errors,
            ekf_status=ekf_status,
        )
    
    @property
    def ekf_validator(self) -> EKFValidator:
        """Access the EKF validator directly."""
        return self._ekf_validator


# ============================================================
# Convenience Functions
# ============================================================


def check_ekf_source_sync(
    current_posxy: int,
    expected_mode: EKFMode,
) -> tuple[bool, str]:
    """
    Synchronous check of EKF source value against expected mode.
    
    Useful for testing and simple validation without async.
    
    Args:
        current_posxy: Current EKF3_SRC1_POSXY value from FC
        expected_mode: Expected EKF mode (GPS or VIO)
    
    Returns:
        Tuple of (is_valid, warning_message)
    
    Example:
        >>> is_valid, warning = check_ekf_source_sync(3, EKFMode.GPS)
        >>> is_valid
        True
        >>> is_valid, warning = check_ekf_source_sync(3, EKFMode.VIO)
        >>> is_valid
        False
        >>> "MISMATCH" in warning
        True
    """
    if expected_mode == EKFMode.GPS:
        expected_value = EKFSourcePosXY.GPS
        mode_name = "GPS (3)"
    else:
        expected_value = EKFSourcePosXY.EXTERNAL_NAV
        mode_name = "VIO/ExternalNav (6)"
    
    if current_posxy == expected_value:
        return True, ""
    
    warning = (
        f"EKF MISMATCH: DANGEROUS - Expected {mode_name} but got {current_posxy}. "
        f"Check EKF3_SRC1_POSXY parameter before flight!"
    )
    return False, warning
