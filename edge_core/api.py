"""
NOMAD Edge Core - REST API

Provides HTTP endpoints for system status and health monitoring.

Target: Python 3.13 | NVIDIA Jetson Orin Nano
"""

import asyncio
from typing import TYPE_CHECKING

from fastapi import FastAPI, WebSocket
from fastapi.encoders import jsonable_encoder
from fastapi.websockets import WebSocketDisconnect

from .state import StateManager

if TYPE_CHECKING:
    pass


def create_app(state_manager: StateManager) -> FastAPI:
    """
    Create the FastAPI application for Edge Core.

    Args:
        state_manager: StateManager instance for system state

    Returns:
        Configured FastAPI application
    """
    app = FastAPI(
        title="NOMAD Edge Core API",
        description="Drone-side API for NOMAD (AEAC 2026)",
        version="0.1.0",
    )

    # ==================== Root / Health ====================

    @app.get("/")
    async def root():
        """API root - returns status information."""
        return {
            "service": "NOMAD Edge Core",
            "version": "0.1.0",
            "description": "Connect via Mission Planner plugin for full control",
        }

    # ==================== Status Endpoints ====================

    @app.get("/status")
    async def get_status():
        """Get current system state."""
        return state_manager.get_state()

    @app.get("/health")
    async def health_check():
        """Health check endpoint."""
        state = state_manager.get_state()
        return {
            "status": "healthy" if state.connected else "degraded",
            "connected": state.connected,
            "gps_fix": state.gps_fix,
            "flight_mode": state.flight_mode,
        }

    @app.websocket("/ws/state")
    async def ws_state(websocket: WebSocket):
        """WebSocket endpoint for real-time state updates."""
        await websocket.accept()
        try:
            while True:
                state = state_manager.get_state()
                await websocket.send_json(jsonable_encoder(state))
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            return

    return app
