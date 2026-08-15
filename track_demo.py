"""
track_demo.py

Main entry point. Spins up a gym-pybullet-drones single-drone
environment, drives a MovingTarget through the scene, renders the
drone's onboard camera each step, runs it through ColorBlobTracker,
and feeds the result into TrackingController to steer the drone.

IMPORTANT: gym-pybullet-drones' exact class names/args have shifted
between versions. This script uses the commonly-documented API as of
writing (CtrlAviary + PID position control). If an import or call
signature below doesn't match your installed version, check:
    python -c "import gym_pybullet_drones; print(gym_pybullet_drones.__file__)"
and look at the `examples/` folder that ships with the package --
it's the fastest way to confirm the current API and fix names here.

Run:
    python track_demo.py
"""

import time
import numpy as np
import cv2
import pybullet as p

from moving_target import MovingTarget
from camera_tracker import ColorBlobTracker, TrackingController

# --- gym-pybullet-drones imports -------------------------------------------
# Check `examples/` in your installed package if these paths differ.
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl

# -----------------------------------------------------------------------------

SIM_FREQ = 240
CTRL_FREQ = 48
DURATION_SEC = 60
FRAME_W, FRAME_H = 320, 240


def get_drone_camera_image(env, nth_drone=0, img_w=FRAME_W, img_h=FRAME_H, fov=90):
    """
    Renders an onboard camera frame directly via PyBullet, independent of
    gym-pybullet-drones' own vision system (which varies/breaks across
    versions -- see track_demo.py history). Uses the drone's own physics
    state, so it stays in sync with the sim automatically.
    """
    state = env._getDroneStateVector(nth_drone)
    pos = state[0:3]
    quat = state[3:7]  # x, y, z, w

    rot = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
    forward = rot.dot(np.array([1.0, 0.0, 0.0]))  # drone's local +x = forward
    up = rot.dot(np.array([0.0, 0.0, 1.0]))

    cam_pos = pos + rot.dot(np.array([0.0, 0.0, 0.02]))  # small forward/up offset
    cam_target = cam_pos + forward * 1.0

    view_matrix = p.computeViewMatrix(cam_pos, cam_target, up)
    proj_matrix = p.computeProjectionMatrixFOV(
        fov=fov, aspect=img_w / img_h, nearVal=0.05, farVal=20.0
    )

    _, _, rgb_img, _, _ = p.getCameraImage(
        img_w, img_h, view_matrix, proj_matrix,
        renderer=p.ER_TINY_RENDERER,
        physicsClientId=env.CLIENT,
    )
    rgb = np.reshape(rgb_img, (img_h, img_w, 4))[:, :, :3].astype(np.uint8)
    return rgb


def spawn_target_sphere(env, radius=0.08, rgba=(0.0, 1.0, 0.0, 1.0)):
    """
    Creates a visible sphere in the PyBullet scene to act as the tracked
    object. rgba defaults to green to match ColorBlobTracker's default
    HSV range in camera_tracker.py -- change both together if you want
    a different color target.
    """
    visual_id = p.createVisualShape(
        p.GEOM_SPHERE, radius=radius, rgbaColor=rgba, physicsClientId=env.CLIENT
    )
    collision_id = -1  # no collision -- it's just a visual tracking target
    body_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_id,
        baseVisualShapeIndex=visual_id,
        basePosition=[0, 0, 1.0],
        physicsClientId=env.CLIENT,
    )
    return body_id


def move_target_sphere(env, body_id, position):
    p.resetBasePositionAndOrientation(
        body_id, position, [0, 0, 0, 1], physicsClientId=env.CLIENT
    )


def main():
    target = MovingTarget(path_type="circle", radius=1.5, height=1.0, speed=0.4)

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=np.array([[0.0, 0.0, 1.0]]),  # start near target's altitude --
                                                      # otherwise the drone hovers near
                                                      # the ground and the target sits
                                                      # outside the camera's vertical FOV,
                                                      # so it never gets detected in the
                                                      # first place (see debug session).
        physics=Physics.PYB,
        pyb_freq=SIM_FREQ,
        ctrl_freq=CTRL_FREQ,
        gui=True,
    )

    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)
    tracker = ColorBlobTracker()
    tracking_controller = TrackingController(target_radius_px=45)

    obs, info = env.reset()
    target_body_id = spawn_target_sphere(env)
    action = np.zeros((1, 4))

    start = time.time()
    step = 0
    dt = 1.0 / CTRL_FREQ
    commanded_yaw = 0.0  # our own tracked heading -- NOT re-based off measured
                          # yaw each step, which caused runaway oscillation
                          # (see debug session: yaw swung -7 rad in ~2s of
                          # sim time from a commanded 0.3 rad/s search rate)

    try:
        while time.time() - start < DURATION_SEC:
            t = time.time() - start
            target_pos = target.position(t)
            move_target_sphere(env, target_body_id, target_pos)

            # 1. Render the drone's onboard camera (custom renderer -- see
            #    get_drone_camera_image() above -- avoids depending on the
            #    library's own vision system, which varies across versions).
            rgb = get_drone_camera_image(env, nth_drone=0)

            # 2. Find the target in the frame.
            cx, cy, radius_px, found = tracker.find_target(rgb)

            # 3. Compute velocity command from tracking error.
            cmd = tracking_controller.update(
                cx or 0, cy or 0, radius_px or 0, FRAME_W, FRAME_H, found, dt
            )

            # 4. Convert velocity command into a target position for the
            #    PID position controller.
            state = obs[0]
            cur_pos = state[0:3]
            cur_quat = state[3:7]
            cur_rpy = state[7:10]     # roll, pitch, yaw (radians)
            cur_vel = state[10:13]
            cur_ang_vel = state[13:16]

            # "vx" from the tracking controller means "forward/back relative
            # to where the drone is currently facing" -- rotate it into
            # world coordinates using the drone's actual heading, otherwise
            # it just drives along world-X regardless of orientation.
            rot = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)
            forward = rot.dot(np.array([1.0, 0.0, 0.0]))
            world_vel = forward * cmd["vx"] + np.array([0.0, 0.0, cmd["vz"]])

            target_step_pos = cur_pos + world_vel * dt
            commanded_yaw += cmd["yaw_rate"] * dt  # integrate our OWN reference,
                                                      # not the noisy measured yaw

            action[0, :], _, _ = ctrl.computeControlFromState(
                control_timestep=dt,
                state=state,
                target_pos=target_step_pos,
                target_rpy=np.array([0, 0, commanded_yaw]),
            )

            obs, reward, terminated, truncated, info = env.step(action)

            # 5. HUD overlay for the recording / demo video.
            hud = rgb.copy()
            if found:
                cv2.circle(hud, (int(cx), int(cy)), int(radius_px), (0, 255, 0), 2)
                cv2.putText(hud, "TRACKING", (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
            else:
                cv2.putText(hud, "SEARCHING", (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 255), 2)
            cv2.imshow("Drone Camera", cv2.cvtColor(hud, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)

            step += 1
            if step % 24 == 0:  # roughly twice a second at CTRL_FREQ=48
                print(
                    f"[DEBUG] found={found} cx={cx} cy={cy} radius_px={radius_px} "
                    f"cmd={ {k: round(v, 4) for k, v in cmd.items()} } "
                    f"cur_pos={np.round(cur_pos, 3)} target_step_pos={np.round(target_step_pos, 3)} "
                    f"action_rpm={np.round(action[0], 1)}"
                )
            env.render()
    finally:
        env.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
