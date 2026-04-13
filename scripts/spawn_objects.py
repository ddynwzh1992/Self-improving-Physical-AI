#!/usr/bin/env python3
"""
spawn_objects.py - Spawn objects in the Isaac Sim 6.0 scene.

Spawns primitive objects (boxes, cylinders, spheres) at specified positions
or randomly distributed in the scene. Useful for testing robot interactions.

Usage:
    python spawn_objects.py --type box --position "0.3,0.0,1.0" --scale "0.1,0.1,0.1"
    python spawn_objects.py --type sphere --count 5 --area "-1,1,-1,1" --height 1.0
    python spawn_objects.py --type cylinder --position "0.5,0.5,0.5" --color "1.0,0.0,0.0"
"""

import os
import sys
import argparse
import json
import random

# Accept EULA - MUST be before any isaacsim imports
os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Spawn objects in Isaac Sim")
parser.add_argument("--type", type=str, required=True, choices=["box", "cylinder", "sphere"],
                    help="Object type to spawn")
parser.add_argument("--position", type=str, default=None,
                    help="Position x,y,z (for single object)")
parser.add_argument("--scale", type=str, default="0.05,0.05,0.05",
                    help="Scale x,y,z")
parser.add_argument("--color", type=str, default=None,
                    help="Color r,g,b (0-1 range)")
parser.add_argument("--count", type=int, default=1,
                    help="Number of objects to spawn")
parser.add_argument("--area", type=str, default="-1.0,1.0,-1.0,1.0",
                    help="Random spawn area: x_min,x_max,y_min,y_max")
parser.add_argument("--height", type=float, default=1.0,
                    help="Spawn height for random objects")
parser.add_argument("--dynamic", action="store_true", default=True,
                    help="Make objects dynamic (physics-enabled)")
parser.add_argument("--static", action="store_true", default=False,
                    help="Make objects static (no physics)")
parser.add_argument("--mass", type=float, default=0.2,
                    help="Object mass in kg")
parser.add_argument("--steps", type=int, default=300,
                    help="Simulation steps after spawning (for settling)")
parser.add_argument("--capture", type=str, default=None,
                    help="Capture viewport image after spawning")
parser.add_argument("--seed", type=int, default=None,
                    help="Random seed for reproducibility")
args, _ = parser.parse_known_args()

if args.seed is not None:
    random.seed(args.seed)

print(f"[spawn_objects] Starting Isaac Sim (headless)...")
print(f"[spawn_objects] Spawning {args.count} {args.type}(s)")
simulation_app = SimulationApp({"headless": True})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import (
    DynamicCuboid, FixedCuboid, VisualCuboid,
    DynamicSphere, VisualSphere,
    DynamicCylinder, VisualCylinder,
)
from isaacsim.core.prims import XFormPrim

# Create world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Parse scale
scale = np.array([float(x) for x in args.scale.split(",")])

# Parse area bounds
area_bounds = [float(x) for x in args.area.split(",")]
x_min, x_max, y_min, y_max = area_bounds

# Default colors for variety
DEFAULT_COLORS = [
    np.array([0.8, 0.2, 0.2]),  # Red
    np.array([0.2, 0.6, 0.2]),  # Green
    np.array([0.2, 0.3, 0.8]),  # Blue
    np.array([0.8, 0.6, 0.1]),  # Orange
    np.array([0.6, 0.2, 0.6]),  # Purple
    np.array([0.1, 0.7, 0.7]),  # Cyan
    np.array([0.9, 0.9, 0.2]),  # Yellow
    np.array([0.5, 0.3, 0.1]),  # Brown
]

# User-specified color
user_color = None
if args.color:
    user_color = np.array([float(x) for x in args.color.split(",")])

is_dynamic = not args.static

spawned_objects = []

for i in range(args.count):
    # Determine position
    if args.position and args.count == 1:
        pos = np.array([float(x) for x in args.position.split(",")])
    else:
        # Random position within area bounds
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        z = args.height
        pos = np.array([x, y, z])

    # Determine color
    color = user_color if user_color is not None else DEFAULT_COLORS[i % len(DEFAULT_COLORS)]

    prim_path = f"/World/SpawnedObject_{i}"
    name = f"spawned_{args.type}_{i}"

    obj = None
    if args.type == "box":
        if is_dynamic:
            obj = DynamicCuboid(
                prim_path=prim_path,
                name=name,
                position=pos,
                scale=scale,
                color=color,
                mass=args.mass,
            )
        else:
            obj = VisualCuboid(
                prim_path=prim_path,
                name=name,
                position=pos,
                scale=scale,
                color=color,
            )
    elif args.type == "sphere":
        if is_dynamic:
            obj = DynamicSphere(
                prim_path=prim_path,
                name=name,
                position=pos,
                radius=scale[0],
                color=color,
                mass=args.mass,
            )
        else:
            obj = VisualSphere(
                prim_path=prim_path,
                name=name,
                position=pos,
                radius=scale[0],
                color=color,
            )
    elif args.type == "cylinder":
        if is_dynamic:
            obj = DynamicCylinder(
                prim_path=prim_path,
                name=name,
                position=pos,
                radius=scale[0],
                height=scale[2],
                color=color,
                mass=args.mass,
            )
        else:
            obj = VisualCylinder(
                prim_path=prim_path,
                name=name,
                position=pos,
                radius=scale[0],
                height=scale[2],
                color=color,
            )

    if obj is not None:
        spawned_objects.append({
            "index": i,
            "type": args.type,
            "position": pos.tolist(),
            "scale": scale.tolist(),
            "color": color.tolist(),
            "dynamic": is_dynamic,
            "prim_path": prim_path,
        })
        print(f"[spawn_objects] Spawned {args.type} #{i} at ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

# Run simulation to let objects settle
if args.steps > 0 and is_dynamic:
    print(f"[spawn_objects] Running {args.steps} steps to let objects settle...")
    world.reset()
    for step in range(args.steps):
        world.step(render=True)
        if (step + 1) % 100 == 0:
            print(f"[spawn_objects] Step {step + 1}/{args.steps}")

    # Report final positions
    print("[spawn_objects] Final positions after settling:")
    for info in spawned_objects:
        prim = XFormPrim(info["prim_path"])
        final_pos, _ = prim.get_world_poses()
        info["final_position"] = final_pos[0].tolist()
        print(f"  {info['type']} #{info['index']}: ({final_pos[0][0]:.3f}, {final_pos[0][1]:.3f}, {final_pos[0][2]:.3f})")

# Capture viewport if requested
if args.capture:
    print(f"[spawn_objects] Capturing viewport to: {args.capture}")
    from isaacsim.core.utils.viewports import set_camera_view
    from isaacsim.sensors.camera import Camera

    cam = Camera(
        prim_path="/World/SpawnCamera",
        position=np.array([2.0, -2.0, 2.0]),
        frequency=30,
        resolution=(1920, 1080),
    )
    cam.initialize()
    set_camera_view(
        eye=np.array([2.0, -2.0, 2.0]),
        target=np.array([0.0, 0.0, 0.3]),
        camera_prim_path="/World/SpawnCamera"
    )
    for _ in range(10):
        world.step(render=True)

    frame = cam.get_rgba()
    if frame is not None:
        from PIL import Image
        img = Image.fromarray(frame[:, :, :3])
        img.save(args.capture)
        print(f"[spawn_objects] Image saved: {args.capture}")

# Output summary
summary = {
    "spawned_count": len(spawned_objects),
    "type": args.type,
    "dynamic": is_dynamic,
    "objects": spawned_objects,
}
print(f"[spawn_objects] OBJECTS_JSON:{json.dumps(summary)}")

print("[spawn_objects] Shutting down...")
simulation_app.close()
print("[spawn_objects] Done.")
