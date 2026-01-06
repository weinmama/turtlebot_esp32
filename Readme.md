+---------------------------+
| Entwicklungsrechner       |
|                           |
| VS Code                   |
| PlatformIO Devcontainer   |
|  - Build ESP32 Firmware   |
|  - Quellcode & Debug      |
+-------------+-------------+
              |
        SSH / rsync / scp
              |
+-------------v-------------+
| Raspberry Pi              |
|                           |
| ROS 2                     |
| micro-ROS Agent (Docker)  |
| USB: /dev/ttyUSB0         |
|                           |
| ESP32 (micro-ROS Client)  |
+---------------------------+



# Raspberry

```
docker run -it --rm   --net=host   \
  --device=/dev/ttyUSB1:/dev/ttyUSB1   \
  microros/micro-ros-agent:jazzy  \
  -vvv serial --dev /dev/ttyUSB1 115200

```

``` 
docker run -it --rm   --net=host   --device=/dev/ttyUSB1   microros/micro-ros-agent:jazzy   serial --dev /dev/ttyUSB1 -b 115200 -v debug
```


## remote deployment on the raspberry pi

``` 
sudo apt install pipx

pipx ensurepath
```


```
ssh robot-pi "
  ~/.local/bin/esptool \
    --chip esp32 \
    --port /dev/ttyUSB1 \
    --baud 460800 \
    write_flash 0x10000 /tmp/firmware.bin
"
```

Tests

ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"