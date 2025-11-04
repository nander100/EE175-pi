# ROS2 Hand Tracking - Docker Setup Guide

This guide helps you receive hand tracking data from a RealSense camera on a Raspberry Pi (or any Linux machine) using Docker and ROS2.

## System Overview

- **Publisher (Laptop)**: Runs RealSense camera + MediaPipe hand tracking, publishes to ROS2 topics
- **Subscriber (Raspberry Pi)**: Receives hand tracking data via ROS2 topics using Docker

---

## Prerequisites

### On Raspberry Pi (Subscriber)
- Docker installed
- Connected to same network as publisher
- Internet connection for initial setup

### On Laptop (Publisher)
- RealSense D455 camera
- Python 3 with required packages
- ROS2 installed

---

## Part 1: Raspberry Pi Setup (Subscriber)

### Step 1: Install Docker

```bash
# Update system
sudo apt update

# Install Docker
sudo apt install docker.io

# Add your user to docker group (avoid needing sudo)
sudo usermod -aG docker $USER

# Apply group changes (or logout/login)
newgrp docker

# Test Docker installation
docker --version
```

### Step 2: Run the Subscriber

```bash
./run_subscriber.sh
```

You should see:
```
Hand tracking subscriber initialized!
Waiting for hand tracking data...
```

---
