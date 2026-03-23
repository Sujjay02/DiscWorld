#!/usr/bin/env python3
"""
Generates disc_world.world for Gazebo with 25 randomly scattered objects
across a 100x100m area for RL-based drone coverage optimization.

Seed is fixed so the layout is reproducible across runs.
"""

import random
import json
import math

random.seed(42)  # Fixed seed = reproducible layout

# --- Configuration ---
AREA_SIZE = 100       # 100x100m
NUM_OBJECTS = 25
MIN_SEPARATION = 4.0  # Minimum distance between objects (avoids stacking)
MARGIN = 5.0          # Keep objects away from world edge

# Object shape definitions: (type, size_range)
SHAPE_TYPES = ["box", "cylinder", "sphere"]
COLORS = [
    (1.0, 0.2, 0.2, 1),  # Red
    (0.2, 0.4, 1.0, 1),  # Blue
    (1.0, 0.8, 0.1, 1),  # Yellow
    (0.9, 0.5, 0.1, 1),  # Orange
    (0.6, 0.2, 0.8, 1),  # Purple
    (0.1, 0.8, 0.8, 1),  # Cyan
    (0.8, 0.2, 0.6, 1),  # Pink
]

# --- Generate random object properties ---
def random_object(index, rng):
    shape = rng.choice(SHAPE_TYPES)
    color = rng.choice(COLORS)
    yaw = rng.uniform(0, 2 * math.pi)

    if shape == "box":
        sx = round(rng.uniform(0.1, 0.5), 2)
        sy = round(rng.uniform(0.1, 0.5), 2)
        sz = round(rng.uniform(0.1, 0.4), 2)
        dims = {"sx": sx, "sy": sy, "sz": sz}
        z_offset = sz / 2
    elif shape == "cylinder":
        radius = round(rng.uniform(0.08, 0.3), 2)
        length = round(rng.uniform(0.1, 0.5), 2)
        dims = {"radius": radius, "length": length}
        z_offset = length / 2
    else:  # sphere
        radius = round(rng.uniform(0.1, 0.3), 2)
        dims = {"radius": radius}
        z_offset = radius

    return {"shape": shape, "color": color, "dims": dims, "z_offset": z_offset, "yaw": round(yaw, 3)}

# --- Generate well-separated random positions ---
def generate_positions(n, area, margin, min_sep, seed=42):
    random.seed(seed)
    positions = []
    attempts = 0
    while len(positions) < n:
        attempts += 1
        if attempts > 10000:
            raise RuntimeError("Could not place all objects with given separation constraint")
        x = random.uniform(-area/2 + margin, area/2 - margin)
        y = random.uniform(-area/2 + margin, area/2 - margin)
        too_close = any(
            ((x - px)**2 + (y - py)**2) < min_sep**2
            for px, py in positions
        )
        if not too_close:
            positions.append((round(x, 2), round(y, 2)))
    return positions

positions = generate_positions(NUM_OBJECTS, AREA_SIZE, MARGIN, MIN_SEPARATION)

# Generate random properties for each object (separate RNG to not affect positions)
obj_rng = random.Random(123)
objects = []
for i, (x, y) in enumerate(positions):
    props = random_object(i, obj_rng)
    props["x"] = x
    props["y"] = y
    props["z"] = round(props["z_offset"], 3)
    objects.append(props)

# Save positions to JSON for use by RL environment
with open("disc_positions.json", "w") as f:
    json.dump({
        "positions": [(o["x"], o["y"], o["z"]) for o in objects],
        "area_size": AREA_SIZE,
        "coverage_radius": 5.0
    }, f, indent=2)
print("Saved disc_positions.json")

# --- SDF geometry block for a shape ---
def geometry_sdf(obj):
    shape = obj["shape"]
    dims = obj["dims"]
    if shape == "box":
        return f"""<box><size>{dims['sx']} {dims['sy']} {dims['sz']}</size></box>"""
    elif shape == "cylinder":
        return f"""<cylinder><radius>{dims['radius']}</radius><length>{dims['length']}</length></cylinder>"""
    else:
        return f"""<sphere><radius>{dims['radius']}</radius></sphere>"""

# --- SDF model block for a single object ---
def object_model(index, obj):
    r, g, b, a = obj["color"]
    geom = geometry_sdf(obj)
    return f"""
    <model name="obj_{index}">
      <static>true</static>
      <pose>{obj['x']} {obj['y']} {obj['z']} 0 0 {obj['yaw']}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            {geom}
          </geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            {geom}
          </geometry>
        </collision>
      </link>
    </model>"""

objects_sdf = "\n".join(object_model(i+1, obj) for i, obj in enumerate(objects))

# --- Full world SDF ---
world_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="disc_world">

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Grass ground plane -->
    <model name="grass_ground">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{AREA_SIZE} {AREA_SIZE}</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{AREA_SIZE} {AREA_SIZE}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.6 0.2 1</ambient>
            <diffuse>0.3 0.6 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Physics -->
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- Scene -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- 25 Randomly Scattered Objects (seed=42, 100x100m area) -->
{objects_sdf}

  </world>
</sdf>
"""

with open("disc_world.world", "w") as f:
    f.write(world_sdf)

print(f"Generated disc_world.world with {NUM_OBJECTS} objects")
print("\nObject positions and shapes:")
for i, obj in enumerate(objects):
    print(f"  obj_{i+1:02d}: {obj['shape']:8s} at ({obj['x']:7.2f}, {obj['y']:7.2f}, {obj['z']})")

print(f"\nArea: {AREA_SIZE}x{AREA_SIZE}m")
print(f"Coverage radius per drone: 5.0m")
print(f"Min object separation: {MIN_SEPARATION}m")
