#!/usr/bin/env python3
"""
Generates disc_world.world for Gazebo with 25 randomly scattered discs
across a 100x100m area for RL-based drone coverage optimization.

Seed is fixed so the layout is reproducible across runs.
"""

import random
import json

random.seed(42)  # Fixed seed = reproducible layout

# --- Configuration ---
AREA_SIZE = 100       # 100x100m
NUM_DISCS = 25
MIN_SEPARATION = 4.0  # Minimum distance between discs (avoids stacking)
DISC_RADIUS = 0.15    # Visual size in Gazebo (meters)
DISC_HEIGHT = 0.02    # Flat disc
DISC_COLOR = (1, 0, 0, 1)  # Red RGBA
MARGIN = 5.0          # Keep discs away from world edge

# --- Generate well-separated random positions ---
def generate_positions(n, area, margin, min_sep, seed=42):
    random.seed(seed)
    positions = []
    attempts = 0
    while len(positions) < n:
        attempts += 1
        if attempts > 10000:
            raise RuntimeError("Could not place all discs with given separation constraint")
        x = random.uniform(-area/2 + margin, area/2 - margin)
        y = random.uniform(-area/2 + margin, area/2 - margin)
        # Check minimum separation from all existing discs
        too_close = any(
            ((x - px)**2 + (y - py)**2) < min_sep**2
            for px, py, _ in positions
        )
        if not too_close:
            positions.append((round(x, 2), round(y, 2), 0.01))
    return positions

positions = generate_positions(NUM_DISCS, AREA_SIZE, MARGIN, MIN_SEPARATION)

# Save positions to JSON for use by RL environment
with open("disc_positions.json", "w") as f:
    json.dump({"positions": positions, "area_size": AREA_SIZE, "coverage_radius": 5.0}, f, indent=2)
print("Saved disc_positions.json")

# --- SDF model block for a single disc ---
def disc_model(index, x, y, z):
    r, g, b, a = DISC_COLOR
    return f"""
    <model name="disc_{index}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{DISC_RADIUS}</radius>
              <length>{DISC_HEIGHT}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{DISC_RADIUS}</radius>
              <length>{DISC_HEIGHT}</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
    </model>"""

discs_sdf = "\n".join(disc_model(i+1, x, y, z) for i, (x, y, z) in enumerate(positions))

# --- Full world SDF ---
world_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="disc_world">

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

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

    <!-- 25 Randomly Scattered Discs (seed=42, 100x100m area) -->
{discs_sdf}

  </world>
</sdf>
"""

with open("disc_world.world", "w") as f:
    f.write(world_sdf)

print("Generated disc_world.world with 25 discs")
print("\nDisc positions (x, y, z):")
for i, (x, y, z) in enumerate(positions):
    print(f"  disc_{i+1:02d}: ({x:7.2f}, {y:7.2f}, {z})")

print(f"\nArea: {AREA_SIZE}x{AREA_SIZE}m")
print(f"Coverage radius per drone: 5.0m")
print(f"Min disc separation: {MIN_SEPARATION}m")
