"""Webots device wrapper.

This file tries to auto-detect common RosBot devices. After the first run, check the
console device list and adjust the name lists if needed.
"""
import math
from typing import List, Optional, Tuple

import config
from coordinates import normalize_angle


class RobotInterface:
    def __init__(self, robot):
        self.robot = robot
        self.motors_left = []
        self.motors_right = []
        self.camera = None
        self.gps = None
        self.compass = None
        self.lidar = None
        self.distance_sensors = []
        self._detect_devices()

    def print_devices(self):
        print("========================================")
        print("AVAILABLE WEBOTS DEVICES")
        print("========================================")
        for i in range(self.robot.getNumberOfDevices()):
            dev = self.robot.getDeviceByIndex(i)
            print(f"{i:02d}: {dev.getName()} | node_type={dev.getNodeType()}")
        print("========================================")
        print("Detected left motors:", [m.getName() for m in self.motors_left])
        print("Detected right motors:", [m.getName() for m in self.motors_right])
        print("Detected camera:", self.camera.getName() if self.camera else None)
        print("Detected gps:", self.gps.getName() if self.gps else None)
        print("Detected compass:", self.compass.getName() if self.compass else None)
        print("Detected lidar:", self.lidar.getName() if self.lidar else None)
        print("Detected distance sensors:", [d.getName() for d in self.distance_sensors])
        print("========================================")

    def _get_optional(self, name: str):
        try:
            return self.robot.getDevice(name)
        except Exception:
            return None

    def _detect_devices(self):
        # Common RosBot wheel motor names.
        left_names = [
            "front left wheel motor", "rear left wheel motor", "left wheel motor",
            "left_motor", "left wheel", "wheel_left_joint", "fl_wheel_joint", "rl_wheel_joint",
        ]
        right_names = [
            "front right wheel motor", "rear right wheel motor", "right wheel motor",
            "right_motor", "right wheel", "wheel_right_joint", "fr_wheel_joint", "rr_wheel_joint",
        ]
        for name in left_names:
            dev = self._get_optional(name)
            if dev is not None:
                self.motors_left.append(dev)
        for name in right_names:
            dev = self._get_optional(name)
            if dev is not None:
                self.motors_right.append(dev)

        # Fallback: scan names.
        if not self.motors_left or not self.motors_right:
            for i in range(self.robot.getNumberOfDevices()):
                dev = self.robot.getDeviceByIndex(i)
                name = dev.getName().lower()
                if "motor" in name or "wheel" in name or "joint" in name:
                    if "left" in name or name.startswith("fl") or name.startswith("rl"):
                        if dev not in self.motors_left:
                            self.motors_left.append(dev)
                    if "right" in name or name.startswith("fr") or name.startswith("rr"):
                        if dev not in self.motors_right:
                            self.motors_right.append(dev)

        for motor in self.motors_left + self.motors_right:
            try:
                motor.setPosition(float("inf"))
                motor.setVelocity(0.0)
            except Exception:
                pass

        # Sensors.
        for name in ["camera", "Camera", "rgb camera", "front camera"]:
            self.camera = self._get_optional(name)
            if self.camera:
                try:
                    self.camera.enable(config.TIME_STEP)
                except Exception:
                    pass
                break

        for name in ["gps", "GPS"]:
            self.gps = self._get_optional(name)
            if self.gps:
                try:
                    self.gps.enable(config.TIME_STEP)
                except Exception:
                    pass
                break

        for name in ["compass", "Compass", "imu compass"]:
            self.compass = self._get_optional(name)
            if self.compass:
                try:
                    self.compass.enable(config.TIME_STEP)
                except Exception:
                    pass
                break

        for name in ["lidar", "Lidar", "Hokuyo URG-04LX-UG01", "laser"]:
            self.lidar = self._get_optional(name)
            if self.lidar:
                try:
                    self.lidar.enable(config.TIME_STEP)
                    try:
                        self.lidar.enablePointCloud()
                    except Exception:
                        pass
                except Exception:
                    pass
                break

        # Generic distance sensors.
        for i in range(self.robot.getNumberOfDevices()):
            dev = self.robot.getDeviceByIndex(i)
            name = dev.getName().lower()
            if "distance" in name or "ds" == name[:2]:
                try:
                    dev.enable(config.TIME_STEP)
                    self.distance_sensors.append(dev)
                except Exception:
                    pass

    def set_velocity(self, linear_v: float, angular_v: float):
        linear_v = max(-config.MAX_LINEAR_SPEED, min(config.MAX_LINEAR_SPEED, linear_v))
        angular_v = max(-config.MAX_ANGULAR_SPEED, min(config.MAX_ANGULAR_SPEED, angular_v))

        left_speed = (linear_v - angular_v * config.TRACK_WIDTH_M / 2.0) / config.WHEEL_RADIUS_M
        right_speed = (linear_v + angular_v * config.TRACK_WIDTH_M / 2.0) / config.WHEEL_RADIUS_M
        left_speed = max(-config.MAX_WHEEL_SPEED, min(config.MAX_WHEEL_SPEED, left_speed))
        right_speed = max(-config.MAX_WHEEL_SPEED, min(config.MAX_WHEEL_SPEED, right_speed))

        for motor in self.motors_left:
            motor.setVelocity(left_speed)
        for motor in self.motors_right:
            motor.setVelocity(right_speed)

    def stop(self):
        self.set_velocity(0.0, 0.0)

    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        """Return (x, z, heading) if GPS and compass are available."""
        if self.gps is None:
            return None
        values = self.gps.getValues()
        x = float(values[0])
        z = float(values[2])

        heading = 0.0
        if self.compass is not None:
            c = self.compass.getValues()
            # Verified against a headless MOTOR_TEST run (forward creep at
            # heading=-pi/2 produced dx=0, dz<0, matching dx=v*cos(h), dz=v*sin(h)
            # exactly): atan2(c[0], c[2]) is the correct convention for this robot,
            # matching coordinates.angle_to_target() / dwa.simulate_trajectory().
            heading = math.atan2(float(c[0]), float(c[2]))
            heading = normalize_angle(heading)
        return x, z, heading

    def get_lidar_ranges(self):
        if self.lidar is None:
            return None
        try:
            return self.lidar.getRangeImage()
        except Exception:
            return None

    def get_lidar_fov(self) -> float:
        if self.lidar is None:
            return math.radians(180)
        try:
            return float(self.lidar.getFov())
        except Exception:
            return math.radians(180)
