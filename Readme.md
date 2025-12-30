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

// L298N H-Bridge Connection PINs
#define L298N_enB 25  // PWM
#define L298N_enA 14  // PWM
#define L298N_in2 32  // Dir Motor B
#define L298N_in1 33  // Dir Motor B
#define L298N_in4 27  // Dir Motor A
#define L298N_in3 26  // Dir Motor A

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 23  // Interrupt
#define right_encoder_phaseB 15
#define left_encoder_phaseA 18   // Interrupt
#define left_encoder_phaseB 13

#define SCL 22
#define SDA 21