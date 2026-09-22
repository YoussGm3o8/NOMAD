# SPDX-License-Identifier: Apache-2.0
"""Transition-only state used by the deterministic MAVSDK peer."""

from __future__ import annotations

from dataclasses import dataclass

from pymavlink.dialects.v20 import ardupilotmega as mavlink


@dataclass
class TransitionModel:
    vtol_state: int
    reaches_fixed_wing: bool
    reports_intermediate: bool
    finish_at: float | None = None

    @classmethod
    def create(
        cls,
        vtol_state: int | None,
        vehicle_type: int,
        params: dict[str, float],
        reaches_fixed_wing: bool,
        reports_intermediate: bool,
    ) -> TransitionModel:
        if vtol_state is None:
            is_quadplane = vehicle_type == mavlink.MAV_TYPE_FIXED_WING and params.get("Q_ENABLE") in (1.0, 2.0)
            vtol_state = mavlink.MAV_VTOL_STATE_MC if is_quadplane else mavlink.MAV_VTOL_STATE_UNDEFINED
        return cls(vtol_state, reaches_fixed_wing, reports_intermediate)
