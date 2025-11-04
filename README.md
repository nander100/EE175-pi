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

### Step 2: Pull ROS2 Docker Image

```bash
# Pull the ROS2 Jazzy base image (~500MB download)
docker pull ros:jazzy-ros-base
```


### Step 3: Create a Helper Script to Run Docker

Create `run_subscriber.sh`:

```bash
#!/bin/bash

# Configuration
ROS_DOMAIN_ID=42
SCRIPT_DIR=$(pwd)

echo "Starting ROS2 Hand Tracking Subscriber in Docker..."
echo "Press Ctrl+C to stop"
echo ""

docker run -it --rm \
  --network=host \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  -e ROS_LOCALHOST_ONLY=0 \
  -v $SCRIPT_DIR:/workspace \
  -w /workspace \
  ros:jazzy-ros-base \
  bash -c "source /opt/ros/jazzy/setup.bash && python3 hand_subscriber.py"
```

Make it executable:
```bash
chmod +x run_subscriber.sh
```

### Step 5: Run the Subscriber

```bash
./run_subscriber.sh
```

You should see:
```
Hand tracking subscriber initialized!
Waiting for hand tracking data...
```

---

## Part 2: Laptop Setup (Publisher)

### Step 1: Configure Network Settings

Add to your `~/.bashrc`:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

Then reload:
```bash
source ~/.bashrc
```

### Step 2: Run the Publisher

```bash
python3 control.py
```

You should see hand tracking data being published.

---

## Troubleshooting

### Subscriber Not Receiving Data

**1. Check ROS_DOMAIN_ID matches on both machines:**

On Raspberry Pi (inside Docker):
```bash
docker run -it --rm ros:jazzy-ros-base bash -c "echo \$ROS_DOMAIN_ID"
# Should output: 42
```

On Laptop:
```bash
echo $ROS_DOMAIN_ID
# Should output: 42
```

**2. Check network connectivity:**

```bash
# From Pi, ping laptop
ping <LAPTOP_IP>

# From laptop, ping Pi
ping <PI_IP>
```

**3. Check firewall settings:**

On Ubuntu laptop:
```bash
# Temporarily disable for testing
sudo ufw disable

# Or allow specific traffic
sudo ufw allow from <PI_IP>
```

**4. Verify topics are being published:**

On laptop:
```bash
ros2 topic list
# Should show:
# /hand/finger_bend
# /hand/wrist_rotation
# /hand/position

# Check if data is flowing
ros2 topic echo /hand/finger_bend
```

**5. Check Docker network mode:**

Ensure `--network=host` is in your docker run command. This allows the container to use the host's network directly.

### Docker Issues

**Permission denied:**
```bash
sudo usermod -aG docker $USER
newgrp docker
```

**Container won't start:**
```bash
# Check Docker is running
sudo systemctl status docker

# Start Docker
sudo systemctl start docker
```

**Image not found:**
```bash
# Re-pull the image
docker pull ros:jazzy-ros-base
```

---

## Understanding the Topics

### `/hand/finger_bend`
- **Type**: `Float32MultiArray`
- **Data**: `[bend_percentage]`
- **Range**: 0-100%
- **Description**: Index finger bend percentage (0% = straight, 100% = fully bent)

### `/hand/wrist_rotation`
- **Type**: `Float32MultiArray`
- **Data**: `[rotation_angle]`
- **Range**: -180° to 180°
- **Description**: Wrist rotation angle (pronation/supination)

### `/hand/position`
- **Type**: `Float32MultiArray`
- **Data**: `[x, y, z]`
- **Units**: meters
- **Description**: 3D position of the wrist in camera coordinates

---

## Customizing the Subscriber

### Example: Control a Robot Gripper

```python
def finger_bend_callback(self, msg):
    bend = msg.data[0]
    
    if bend > 80:
        self.get_logger().info('CLOSE GRIPPER')
        # Add your gripper close code here
    elif bend < 20:
        self.get_logger().info('OPEN GRIPPER')
        # Add your gripper open code here
```

### Example: Map Hand Position to Robot Movement

```python
def hand_position_callback(self, msg):
    x, y, z = msg.data[0], msg.data[1], msg.data[2]
    
    # Scale hand position to robot workspace
    robot_x = x * 100  # Convert to cm
    robot_y = y * 100
    robot_z = z * 100
    
    self.get_logger().info(f'Move robot to: [{robot_x}, {robot_y}, {robot_z}] cm')
    # Add your robot movement code here
```

---

## Advanced: Running Without the Helper Script

If you want to run the Docker container manually:

```bash
docker run -it --rm \
  --network=host \
  -e ROS_DOMAIN_ID=42 \
  -e ROS_LOCALHOST_ONLY=0 \
  -v $(pwd):/workspace \
  -w /workspace \
  ros:jazzy-ros-base \
  bash
```

Then inside the container:
```bash
source /opt/ros/jazzy/setup.bash
python3 hand_subscriber.py
```

---

## Quick Reference

### Start Everything

**Terminal 1 (Laptop):**
```bash
python3 control.py
```

**Terminal 2 (Raspberry Pi):**
```bash
./run_subscriber.sh
```

### Stop Everything

Press `Ctrl+C` in both terminals.

### List All Topics

```bash
# On laptop
ros2 topic list

# On Raspberry Pi (inside Docker)
docker run -it --rm --network=host -e ROS_DOMAIN_ID=42 ros:jazzy-ros-base bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic list"
```

### Echo a Topic

```bash
# On laptop
ros2 topic echo /hand/finger_bend

# On Raspberry Pi (inside Docker)
docker run -it --rm --network=host -e ROS_DOMAIN_ID=42 ros:jazzy-ros-base bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /hand/finger_bend"
```

---

## Network Configuration

Both devices must be on the same network. Common setups:

1. **Same WiFi Network**: Both connected to same router
2. **Ethernet**: Direct connection or through switch
3. **Hotspot**: One device creates hotspot, other connects

**Find your IP addresses:**
```bash
# On both machines
ip addr show | grep inet
```

---

## Support

If you encounter issues:

1. Check all troubleshooting steps above
2. Verify both machines are on same network
3. Ensure ROS_DOMAIN_ID is identical (42)
4. Check firewall settings
5. Verify Docker has network access

---

## System Requirements

- **Raspberry Pi**: Any model with Docker support (Pi 4/5 recommended)
- **Network**: Local network with multicast support
- **Disk Space**: ~500MB for Docker image
- **RAM**: 1GB minimum (2GB+ recommended)

---

## License

This project is provided as-is for educational and research purposes.