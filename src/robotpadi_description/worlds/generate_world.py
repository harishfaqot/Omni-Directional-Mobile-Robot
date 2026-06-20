#!/usr/bin/env python3
# generate_world.py
# Run once: python3 generate_world.py > paddy_generated.world

N_GROUPS = 5
N_PLANTS = 10  # per row
X_OFFSET = -3.5  # shift right (meters)
Y_OFFSET = -3.0  # shift forward (meters)

header = '''<?xml version="1.0"?>
<!-- Seed: 2983 -->
<sdf version="1.9">
  <world name="virtual_maize_field">

    <!-- Change max step size to increase the simulation speed but decrease the accuracy. -->
    <physics name="10ms" type="ignored">
      <max_step_size>0.03</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="heightmap">
      <static>true</static>
      <link name="link">
        <visual name="ground_plane">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <texture>
                <diffuse>../models/materials/textures/grass_color.jpg</diffuse>
                <normal>../models/materials/textures/grass_normal.jpg</normal>
                <size>4</size>
              </texture>
              <texture>
                <diffuse>../models/materials/textures/dirt_diffusespecular.png</diffuse>
                <normal>../models/materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <texture>
                <diffuse>../models/materials/textures/fungus_diffusespecular.png</diffuse>
                <normal>../models/materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <blend>
                <min_height>0.2</min_height>
                <fade_dist>0.05</fade_dist>
              </blend>
              <blend>
                <min_height>4</min_height>
                <fade_dist>5</fade_dist>
              </blend>
              <uri>../models/maize/materials/textures/virtual_maize_field_heightmap.png</uri>
              <size>21.88 21.88 0.4</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>../models/maize/materials/textures/virtual_maize_field_heightmap.png</uri>
              <size>21.88 21.88 0.4</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
      </link>
    </model>
'''

footer = '''  </world>
</sdf>'''

BASE_PATH = "/home/ubuntu/github/Omni-Directional-Mobile-Robot/src/robotpadi_description/models"

print(header)

for g in range(N_GROUPS):
    y_base = g * 0.75
    print(f'    <!-- ===== GROUP {g} (Y base = {y_base + Y_OFFSET:.3f}) ===== -->')

    for p in range(N_PLANTS):
        # Row A — no stagger
        x = p * 0.25 + X_OFFSET
        y = y_base + Y_OFFSET
        model = "maize_01" if p % 2 == 0 else "maize_02"
        print(f'    <include><name>rice_g{g}rA_p{p}</name>'
              f'<pose>{x:.4f} {y:.4f} 0.3 0 0 0</pose>'
              f'<uri>{BASE_PATH}/{model}/model.sdf</uri></include>')

        # Row B — staggered
        x = p * 0.25 + 0.125 + X_OFFSET
        y = y_base + 0.25 + Y_OFFSET
        model = "maize_01" if p % 2 == 0 else "maize_02"
        print(f'    <include><name>rice_g{g}rB_p{p}</name>'
              f'<pose>{x:.4f} {y:.4f} 0.3 0 0 0</pose>'
              f'<uri>{BASE_PATH}/{model}/model.sdf</uri></include>')

print(footer)