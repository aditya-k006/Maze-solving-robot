# Maze Solving Robot

A maze-solving robot built on the Raspberry Pi Pico using a flood-fill algorithm, mecanum wheels, TFMini LiDAR sensors for wall detection, and an MPU6050 IMU for heading control.

---

## Hardware

| Component | Quantity | Notes |
|---|---|---|
| Raspberry Pi Pico / Pico W | 1 | Main microcontroller |
| N20 micro metal gear motor | 4 | 300 RPM |
| Mecanum wheel (48mm) | 4 |  (2× left-hand, 2× right-hand) |
| L298N dual H-bridge | 2 | One per side (left/right) |
| TFMini LiDAR (or TFMini-S) | 3 | Front, Left, Right |
| MPU6050 IMU | 1 | I2C |
| 12V 2000 mAh LiPo/Li-ion battery | 1 | Powers motors |
| 5V regulator  | 1 | Powers Pico + sensors |
| Power switch | 1 | Main on/off |

> **TFMini note:** The standard TFMini has a minimum range of **30 cm**. For cell sizes smaller than ~25 cm, use the **TFMini-S** (minimum range 10 cm) for reliable side-wall detection.

---

## Pin Assignment

### L298N #1 — Left motors (Front-Left + Rear-Left)

| Signal | GPIO | Physical Pin |
|---|---|---|
| FL_IN1 | GP2 | 4 |
| FL_IN2 | GP3 | 5 |
| FL_EN (PWM) | GP10 | 14 |
| RL_IN1 | GP4 | 6 |
| RL_IN2 | GP5 | 7 |
| RL_EN (PWM) | GP11 | 15 |

### L298N #2 — Right motors (Front-Right + Rear-Right)

| Signal | GPIO | Physical Pin |
|---|---|---|
| FR_IN1 | GP6 | 9 |
| FR_IN2 | GP7 | 10 |
| FR_EN (PWM) | GP12 | 16 |
| RR_IN1 | GP8 | 11 |
| RR_IN2 | GP9 | 12 |
| RR_EN (PWM) | GP13 | 17 |

### MPU6050 (I2C0)

| Signal | GPIO | Physical Pin |
|---|---|---|
| SDA | GP16 | 21 |
| SCL | GP17 | 22 |
| VCC | 3.3V | 36 |
| GND | GND | 38 |

### TFMini Sensors (UART — only RX needed)

| Sensor | Interface | GPIO (RX) | Physical Pin |
|---|---|---|---|
| Front | UART0 | GP1 | 2 |
| Left | UART1 | GP21 | 27 |
| Right | PIO soft-UART | GP22 | 29 |

> Each TFMini TX wire can be left unconnected or tied to GND through a 10 kΩ resistor. The Pico does not send any commands to the sensor.

### Power

```
12V Battery (+) ──► Switch ──► L298N #1 VSS
                           └──► L298N #2 VSS
12V Battery (–) ──────────────► L298N GND (both)

L298N 5V pin ──► 5V regulator input  (or use separate 5V supply)
5V reg output ──► Pico VSYS (pin 39)
Pico 3.3V (pin 36) ──► MPU6050 VCC, TFMini VCC
```

---

## Mecanum Wheel Layout

```
    FRONT
  [\]  [/]
  FL    FR

  RL    RR
  [/]  [\]
    REAR
```

The `\` and `/` indicate roller orientation.  
If the robot **spins when trying to strafe**, swap the `vy` sign in `mecanum.c` line 37.

---

## Building

### Prerequisites

- [Pico SDK](https://github.com/raspberrypi/pico-sdk) installed and `PICO_SDK_PATH` set
- CMake ≥ 3.13
- `arm-none-eabi-gcc` toolchain

### Build steps

```bash
git clone https://github.com/YOUR_USERNAME/maze_robot.git
cd maze_robot
mkdir build && cd build
cmake ..
make -j4
```

Flash `maze_robot.uf2` to the Pico by holding BOOTSEL, plugging in USB, then copying the file.

---

## Calibration

### 1. Cell travel time (`CELL_TRAVEL_MS` in `config.h`)

Place the robot at the start of a straight corridor of known length. Run the robot and measure how far it travels in the default 900 ms. Adjust `CELL_TRAVEL_MS` until one forward move equals exactly one cell (180 mm by default).

### 2. PID gains (in `main.c`, `pid_init` call)

Drive the robot straight on a flat surface and watch the serial output.

| Symptom | Fix |
|---|---|
| Robot oscillates left/right | Reduce `kp` |
| Robot drifts slowly | Increase `kp` |
| Oscillation damps slowly | Increase `kd` |
| Steady offset remains | Increase `ki` slightly |

### 3. Wall detection thresholds (`config.h`)

- `WALL_PRESENT_MM`: increase if the robot misses walls, decrease if it sees phantom walls.
- `WALL_CLEAR_MM`: currently unused in detection logic but available for hysteresis.

### 4. Turn speed (`TURN_SPEED` in `config.h`)

Reduce if the robot overshoots 90° turns; increase if turns are too slow.  
`TURN_TOLERANCE_DEG` controls the angular dead-band for turn completion.

---

## Project Structure

```
maze_robot/
├── CMakeLists.txt
├── README.md
└── src/
    ├── config.h               ← ALL pin definitions and tuning constants
    ├── main.c                 ← Main solve loop
    ├── control/
    │   ├── pid.h
    │   └── pid.c              ← Generic PID with anti-windup
    ├── maze/
    │   ├── map.h / map.c      ← 16×16 cell grid with wall bitmasks
    │   └── floodfill.h / .c   ← BFS distance map + next-direction query
    ├── motor/
    │   ├── l298n.h / l298n.c  ← PWM motor driver
    │   └── mecanum.h / .c     ← Kinematic mixing (vx, vy, omega → wheels)
    └── sensors/
        ├── mpu6050.h / .c     ← I2C gyro (yaw integration + calibration)
        ├── tfmini.h / .c      ← UART 9-byte frame parser (all 3 sensors)
        └── uart_rx.pio        ← PIO soft-UART for 3rd TFMini sensor
```

---

## How It Works

1. **Flood fill initialisation** — BFS from the goal cell outward assigns each cell a distance value (number of steps to goal), assuming no walls except the outer boundary.
2. **Sense** — At each cell the robot reads Front/Left/Right distances and marks any new walls.
3. **Re-flood** — If new walls were found, BFS runs again to update distances.
4. **Navigate** — Robot always moves to the open neighbour with the lowest distance value.
5. **Heading control** — During straight driving the gyro Z rate is integrated to estimate yaw; a PID controller applies an omega (rotation) correction to keep the robot on track.

---

## License

MIT License — feel free to use, modify, and distribute with attribution.
