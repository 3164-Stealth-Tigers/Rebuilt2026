"""
NetworkTables publisher for vision data.
Publishes detection results to roboRIO using pynetworktables.
"""

import time
from typing import List, Optional

from networktables import NetworkTables

from config import NT_SERVER_IP, NTKeys
from detectors.base_detector import Detection


class NTPublisher:
    """Publishes vision data to NetworkTables for roboRIO consumption."""

    def __init__(self, server_ip: str = NT_SERVER_IP):
        """
        Initialize NetworkTables publisher.

        Args:
            server_ip: IP address of the NetworkTables server (roboRIO)
        """
        self.server_ip = server_ip
        self._connected = False
        self._robots_table = None
        self._fuel_table = None

    def connect(self) -> bool:
        """
        Connect to the NetworkTables server.

        Returns:
            True if connection initiated successfully
        """
        try:
            NetworkTables.initialize(server=self.server_ip)

            # Get table references
            self._robots_table = NetworkTables.getTable(NTKeys.Robots.TABLE)
            self._fuel_table = NetworkTables.getTable(NTKeys.Fuel.TABLE)

            # Set up connection listener
            NetworkTables.addConnectionListener(
                self._connection_listener,
                immediateNotify=True,
            )

            print(f"NetworkTables connecting to {self.server_ip}...")
            return True

        except Exception as e:
            print(f"Failed to initialize NetworkTables: {e}")
            return False

    def _connection_listener(self, connected: bool, info):
        """Handle connection state changes."""
        self._connected = connected
        if connected:
            print(f"NetworkTables connected to {info.remote_ip}")
        else:
            print("NetworkTables disconnected")

    @property
    def is_connected(self) -> bool:
        """Check if connected to NetworkTables server."""
        return self._connected

    def publish_robots(
        self,
        detections: List[Detection],
        closest: Optional[Detection],
        timestamp: float,
    ):
        """
        Publish robot detection data to NetworkTables.

        Args:
            detections: List of all robot detections
            closest: Closest robot detection (or None)
            timestamp: Frame timestamp
        """
        if self._robots_table is None:
            return

        table = self._robots_table

        # Robot count
        table.putNumber(NTKeys.Robots.COUNT, len(detections))

        # Closest robot data
        if closest is not None:
            table.putNumber(NTKeys.Robots.CLOSEST_X, closest.x)
            table.putNumber(NTKeys.Robots.CLOSEST_Y, closest.y)
            table.putNumber(NTKeys.Robots.CLOSEST_DISTANCE, closest.distance)
            table.putNumber(NTKeys.Robots.CLOSEST_VEL_X, closest.vel_x)
            table.putNumber(NTKeys.Robots.CLOSEST_VEL_Y, closest.vel_y)
            table.putNumber(NTKeys.Robots.CLOSEST_CONFIDENCE, closest.confidence)
        else:
            # No robot detected - publish default values
            table.putNumber(NTKeys.Robots.CLOSEST_X, 0.0)
            table.putNumber(NTKeys.Robots.CLOSEST_Y, 0.0)
            table.putNumber(NTKeys.Robots.CLOSEST_DISTANCE, 0.0)
            table.putNumber(NTKeys.Robots.CLOSEST_VEL_X, 0.0)
            table.putNumber(NTKeys.Robots.CLOSEST_VEL_Y, 0.0)
            table.putNumber(NTKeys.Robots.CLOSEST_CONFIDENCE, 0.0)

        # Timestamp for latency compensation
        table.putNumber(NTKeys.Robots.TIMESTAMP, timestamp)

    def publish_fuel(
        self,
        detections: List[Detection],
        best: Optional[Detection],
        intake_ready: bool,
        timestamp: float,
    ):
        """
        Publish FUEL detection data to NetworkTables.

        Args:
            detections: List of all FUEL detections
            best: Best FUEL target (or None)
            intake_ready: Whether best FUEL is ready for intake
            timestamp: Frame timestamp
        """
        if self._fuel_table is None:
            return

        table = self._fuel_table

        # FUEL count and presence
        table.putNumber(NTKeys.Fuel.COUNT, len(detections))
        table.putBoolean(NTKeys.Fuel.HAS_FUEL, len(detections) > 0)

        # Best FUEL target data
        if best is not None:
            table.putNumber(NTKeys.Fuel.BEST_X, best.x)
            table.putNumber(NTKeys.Fuel.BEST_Y, best.y)
            table.putNumber(NTKeys.Fuel.BEST_DISTANCE, best.distance)
            table.putNumber(NTKeys.Fuel.BEST_ANGLE, best.angle)
            table.putNumber(NTKeys.Fuel.BEST_CONFIDENCE, best.confidence)
            table.putBoolean(
                NTKeys.Fuel.IN_INTAKE_RANGE,
                best.distance <= 0.5,  # INTAKE_RANGE_DISTANCE
            )
        else:
            table.putNumber(NTKeys.Fuel.BEST_X, 0.0)
            table.putNumber(NTKeys.Fuel.BEST_Y, 0.0)
            table.putNumber(NTKeys.Fuel.BEST_DISTANCE, 0.0)
            table.putNumber(NTKeys.Fuel.BEST_ANGLE, 0.0)
            table.putNumber(NTKeys.Fuel.BEST_CONFIDENCE, 0.0)
            table.putBoolean(NTKeys.Fuel.IN_INTAKE_RANGE, False)

        # Intake ready status
        table.putBoolean(NTKeys.Fuel.INTAKE_READY, intake_ready)

        # Timestamp for latency compensation
        table.putNumber(NTKeys.Fuel.TIMESTAMP, timestamp)

    def disconnect(self):
        """Disconnect from NetworkTables server."""
        try:
            NetworkTables.shutdown()
            self._connected = False
            self._robots_table = None
            self._fuel_table = None
            print("NetworkTables disconnected")
        except Exception as e:
            print(f"Error disconnecting from NetworkTables: {e}")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False
