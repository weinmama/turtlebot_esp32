# Turtlebot ESP32 – Pico-ROS / Zenoh

Dieser Branch verwendet **Pico-ROS über die vorhandene serielle Verbindung**
(UART0, 250000 Baud). Der Pi benötigt den getrennten seriellen Zenoh-Router und den
mitgelieferten Zeitdienst. Der micro-ROS-Agent und die DDS/Zenoh-Bridge entfallen.

**Build, Pi-Umstellung, Flashen und ROS-Kommandos:** [PICOROS_PORT.md](PICOROS_PORT.md).

```text
ESP32 (Pico-ROS) ← USB/seriell → Pi (Zenoh-Router) ←→ ROS / EKF / Nav2 / UI
```

```sh
git submodule update --init --recursive
pio run -e esp32 -t firmware_bundle
bash scripts/test.sh
```

`/cmd_vel` verwendet weiterhin `geometry_msgs/msg/TwistStamped` mit aktuellem
Zeitstempel. Radodometrie kommt direkt auf `/wheel/odom`; IMU auf `/imu/data`.
Modus-Heartbeat, gültige Zeitbasis und abgeschlossene stationäre
Gyro-Kalibrierung sind Voraussetzung für Fahrbefehle.

## Motorregelung

`/motor/state` ist ein `Float32MultiArray` mit sieben Werten:
vorzeichenbehaftete Soll-Drehzahl links/rechts (U/min), Encoder-Drehzahl
links/rechts (U/min), PWM links/rechts und Regelintervall (Sekunden).
Die aktuelle Vorsteuerung und PI-Abstimmung wurden bei aufgebocktem Roboter
ermittelt; unter Last, bei niedriger Geschwindigkeit und unterschiedlichen
Akkuständen müssen sie noch überprüft werden. Der PWM-Offset ist kein
garantierter Mindestwert zum Anfahren unter Last.

`/pid_tune` nimmt `[Kp, Ki, Kd]` für beide Räder oder
`[Kp_L, Ki_L, Kd_L, Kp_R, Ki_R, Kd_R]` entgegen. Änderungen über dieses
Topic sind flüchtig und gehen beim Neustart verloren.
