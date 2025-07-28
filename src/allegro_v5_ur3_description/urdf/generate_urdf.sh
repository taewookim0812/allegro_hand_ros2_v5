#!/bin/bash

# Run xacro to generate the URDF
ros2 run xacro xacro combined_robot.urdf.xacro \
  safety_limits:=true \
  safety_pos_margin:=0.15 \
  safety_k_position:=20 \
  ur_type:=ur3 \
  tf_prefix:="" \
  name:=allegro_v5_ur3 \
  -o allegro_v5_ur3.urdf

echo "✔️  URDF generated: allegro_v5_ur3.urdf"

