"""
moving_target.py

Defines the object the drone will track. Instead of a static point,
the target moves along a scripted path so the tracking behavior
actually has something to react to.

Usage:
    target = MovingTarget(path_type="circle", radius=1.5, height=1.0, speed=0.4)
    pos = target.position(t)   # returns np.array([x, y, z]) at simulation time t
"""

import numpy as np


class MovingTarget:
    def __init__(self, path_type="circle", radius=1.5, height=1.0,
                 speed=0.4, center=(0.0, 0.0), waypoints=None):
        """
        path_type : "circle" | "figure8" | "waypoints" | "wander"
        radius    : radius of circle / figure8 (meters)
        height    : constant altitude of the target (meters)
        speed     : angular/linear speed factor -- higher = faster
        center    : (x, y) center of the circle/figure8 path
        waypoints : list of (x, y, z) tuples, required if path_type == "waypoints"
        """
        self.path_type = path_type
        self.radius = radius
        self.height = height
        self.speed = speed
        self.center = np.array(center, dtype=float)
        self.waypoints = waypoints or []
        self._wander_target = None
        self._rng = np.random.default_rng(0)

    def position(self, t):
        """Returns the target's [x, y, z] position at simulation time t (seconds)."""
        if self.path_type == "circle":
            x = self.center[0] + self.radius * np.cos(self.speed * t)
            y = self.center[1] + self.radius * np.sin(self.speed * t)
            z = self.height
            return np.array([x, y, z])

        elif self.path_type == "figure8":
            x = self.center[0] + self.radius * np.sin(self.speed * t)
            y = self.center[1] + self.radius * np.sin(self.speed * t) * np.cos(self.speed * t)
            z = self.height
            return np.array([x, y, z])

        elif self.path_type == "waypoints":
            if not self.waypoints:
                raise ValueError("waypoints path_type requires a non-empty waypoints list")
            n = len(self.waypoints)
            seg_time = 3.0 / max(self.speed, 1e-3)  # seconds per leg
            idx = int(t // seg_time) % n
            next_idx = (idx + 1) % n
            frac = (t % seg_time) / seg_time
            a = np.array(self.waypoints[idx], dtype=float)
            b = np.array(self.waypoints[next_idx], dtype=float)
            return a + (b - a) * frac

        elif self.path_type == "wander":
            # Simple random-walk target that picks a new goal point
            # whenever it gets close to its current one.
            if self._wander_target is None or np.random.rand() < 0.002:
                self._wander_target = self.center + self._rng.uniform(-self.radius, self.radius, size=2)
            direction = self._wander_target - self.center
            self.center = self.center + direction * 0.01 * self.speed
            return np.array([self.center[0], self.center[1], self.height])

        else:
            raise ValueError(f"Unknown path_type: {self.path_type}")
