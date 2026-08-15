"""
camera_tracker.py

Two pieces:
1. ColorBlobTracker  - finds the target in a rendered RGB camera frame
                        using HSV color thresholding (fast, no training
                        data needed -- good enough for a colored sphere).
2. PIDController      - a small reusable PID used on each control axis
                        (yaw-to-center, distance-hold, altitude-hold).

Swap ColorBlobTracker for a real detector (e.g. a tiny YOLO model) later
without touching the controller code -- it just needs to return a
(cx, cy, found) tuple in pixel coordinates.
"""

import cv2
import numpy as np


class ColorBlobTracker:
    def __init__(self, lower_hsv=(35, 80, 80), upper_hsv=(85, 255, 255)):
        """
        Default range targets a green sphere. Adjust lower/upper HSV
        bounds to match whatever color you render the target object as.
        """
        self.lower = np.array(lower_hsv)
        self.upper = np.array(upper_hsv)

    def find_target(self, rgb_frame):
        """
        rgb_frame : HxWx3 uint8 RGB image from the drone's onboard camera.
        Returns: (cx, cy, radius_px, found: bool)
        """
        hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, None, False

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 15:  # ignore noise
            return None, None, None, False

        (x, y), radius = cv2.minEnclosingCircle(largest)
        return x, y, radius, True


class PIDController:
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limit = output_limit
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def step(self, error, dt):
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        if self.output_limit is not None:
            output = float(np.clip(output, -self.output_limit, self.output_limit))
        return output


class TrackingController:
    """
    Combines pixel-space tracking error with a distance-hold controller
    to produce a velocity command the drone should fly.

    Call update(cx, cy, radius_px, frame_w, frame_h, found, dt) every step.
    Returns a dict of velocity commands: vx, vy, vz, yaw_rate
    """
    def __init__(self, target_radius_px=40):
        self.yaw_pid = PIDController(kp=0.010, ki=0.0, kd=0.002, output_limit=2.0)
        self.dist_pid = PIDController(kp=0.05, ki=0.0, kd=0.01, output_limit=1.5)
        self.alt_pid = PIDController(kp=0.008, ki=0.0, kd=0.002, output_limit=0.8)
        self.target_radius_px = target_radius_px  # bigger = closer; used as "hold distance"
        self.state = "SEARCHING"

    def update(self, cx, cy, radius_px, frame_w, frame_h, found, dt):
        if not found:
            self.state = "SEARCHING"
            # Simple search behavior: slow yaw spin to reacquire.
            return {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.3}

        self.state = "TRACKING"
        center_x, center_y = frame_w / 2, frame_h / 2

        yaw_error = (cx - center_x)              # + means target is right of center
        alt_error = (center_y - cy)               # + means target is above center
        dist_error = (self.target_radius_px - radius_px)  # + means target too far (blob too small)

        yaw_rate = self.yaw_pid.step(yaw_error, dt)
        vz = self.alt_pid.step(alt_error, dt)
        vx = self.dist_pid.step(dist_error, dt)   # forward speed toward/away from target

        return {"vx": vx, "vy": 0.0, "vz": vz, "yaw_rate": yaw_rate}
