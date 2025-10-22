#!/bin/bash
# Test script to manually publish a path to Gazebo

echo "Publishing test path to /cmd_path..."

# Simple 3-waypoint path
gz topic -t /cmd_path -m gz.msgs.Pose_V -p '{"pose": [{"position": {"x": 3.0, "y": 0.0, "z": 1.2138}, "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}, {"position": {"x": 0.0, "y": 3.0, "z": 1.2138}, "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}, {"position": {"x": -3.0, "y": 0.0, "z": 1.2138}, "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}}]}'

echo "Path published! Check Gazebo terminal for PathCallback messages."
