# Outside Docker
1. Pull the latest repo
Inside your project root directory (Project-NUEVO)
```git pull```

2. Set up the package
```git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git```

3. Check if the device is present
```sudo chmod 777 /dev/ttyUSB0```

3. Stop any running Docker containers
```docker compose -f $COMPOSE down```

4. Restart Docker and enter docker terminal
```bash
docker compose -f $COMPOSE up -d
docker compose -f $COMPOSE logs -f ros2_runtime
docker compose -f $COMPOSE exec ros2_runtime bash
```

# Inside Docker
1. Source ROS 2 environment
```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

2. Start ROS2 node
```bash
ros2 run rplidar_ros rplidar_node --ros-args   -p channel_type:="serial"   -p serial_port:="/dev/rplidar"   -p serial_baudrate:=460800   -p frame_id:="laser_frame"   -p angle_compensate:=true   -p scan_mode:="Standard"
```