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

Im Turtlebox-Hauptrepository vom Entwicklungsrechner aus:

```bash
mise run deploy:esp32
```

Dies baut lokal und flasht alle benötigten Images per SSH auf `robot-pi`
über `/dev/esp32`. Dokumentation: `scripts/esp32/README.md` im Hauptrepository.
Nur bauen: `mise run build:esp32`.

Tests

ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

### Motorregelung prüfen

`/motor/state` ist ein Best-Effort-`Float32MultiArray` mit sieben Werten:
vorzeichenbehaftete Soll-Drehzahl links/rechts (U/min), Encoder-Drehzahl
links/rechts (U/min), PWM links/rechts und Regelintervall (Sekunden).
Die aktuelle Vorsteuerung und PI-Abstimmung wurden bei aufgebocktem Roboter
ermittelt; unter Last, bei niedriger Geschwindigkeit und unterschiedlichen
Akkuständen müssen sie noch überprüft werden. Der PWM-Offset ist kein
garantierter Mindestwert zum Anfahren unter Last.

`/pid_tune` nimmt `[Kp, Ki, Kd]` für beide Räder oder
`[Kp_L, Ki_L, Kd_L, Kp_R, Ki_R, Kd_R]` entgegen. Änderungen über dieses
Topic sind flüchtig und gehen beim Neustart verloren.
