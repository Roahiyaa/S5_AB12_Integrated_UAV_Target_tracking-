# Object-Tracking Drone Simulation

A quadcopter (simulated in [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones))
visually tracks a moving target using its onboard camera, HSV color
detection, and a PID tracking controller.

## Project structure

```
object_tracking_drone/
├── requirements.txt      # deps, incl. gym-pybullet-drones
├── moving_target.py       # scripted target trajectories (circle/figure8/waypoints/wander)
├── camera_tracker.py      # ColorBlobTracker (vision) + PIDController + TrackingController
├── track_demo.py           # main script: wires env + target + tracker together
└── README.md
```

## Setup

```bash
python -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

gym-pybullet-drones pulls in PyBullet, gymnasium, and a few other deps —
first install can take a few minutes.

## Run

```bash
python track_demo.py
```

A PyBullet GUI window and an OpenCV "Drone Camera" HUD window should both
open. The HUD shows a green circle + "TRACKING" when the target is
detected, and "SEARCHING" (with the drone slowly yawing) when it's lost.

## If something doesn't match your installed version

`gym-pybullet-drones`'s API has shifted across versions (class names,
`_getDroneImages` vs a renamed helper, controller signatures, etc.).
If `track_demo.py` throws an import or attribute error:

1. Check where it's installed: `python -c "import gym_pybullet_drones; print(gym_pybullet_drones.__file__)"`
2. Look in the `examples/` folder next to that path — it always has a
   working single-drone script for the version you have installed.
3. Swap the mismatched import/call in `track_demo.py` to match. The
   target/tracker/controller modules (`moving_target.py`,
   `camera_tracker.py`) don't depend on the env internals, so they
   won't need changes.

## Suggested build order (weekend plan)

1. **Confirm the base env runs** — comment out the tracking pieces,
   just get `CtrlAviary` opening a GUI with the drone hovering.
2. **Add the target** — spawn a colored sphere at `target.position(t)`
   each step (`p.resetBasePositionAndOrientation` in PyBullet, or via
   the env's URDF loading if you want the target to have collision).
3. **Verify the camera + tracker** — print `found, cx, cy, radius_px`
   each step and confirm the color mask locks onto the target. Adjust
   the HSV range in `ColorBlobTracker` to match your target's color.
4. **Wire in the controller** — this is what's in `track_demo.py`
   already; tune the PID gains in `camera_tracker.py` if the drone
   overshoots or oscillates.
5. **Polish** — record the OpenCV HUD window or the PyBullet GUI to a
   video/gif for your portfolio (e.g. `imageio` frame capture, or just
   screen-record).

## Ideas for extending later

- Swap `ColorBlobTracker` for a small trained detector (e.g. YOLOv8n)
  for a more "real" vision pipeline.
- Multiple targets with priority-based re-acquisition.
- A minimap or top-down trajectory plot alongside the camera HUD.
- Simple obstacle avoidance layered on top of the tracking controller.
