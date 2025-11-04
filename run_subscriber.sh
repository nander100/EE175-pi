#!/bin/bash

ROS_DOMAIN_ID=42
SCRIPT_DIR=$(pwd)

echo "Starting ROS2 Hand Tracking Subscriber in Docker..."
echo "Press Ctrl+C to stop"
echo ""

sudo docker run -it --rm \
  --network=host \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  -e ROS_LOCALHOST_ONLY=0 \
  -v $SCRIPT_DIR:/workspace \
  -w /workspace \
  ros:jazzy-ros-base \
  bash -c "source /opt/ros/jazzy/setup.bash && python3 hand_subscriber.py"