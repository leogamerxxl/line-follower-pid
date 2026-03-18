# Line Follower — PID Controller

Arduino Nano based line follower robot with PID control algorithm and 5-sensor TCRT5000 array.

Built as a personal project during high school. Chassis cut from acrylic sheet, custom sensor array PCB.

---

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | Arduino Nano (ATmega328P, 16MHz) |
| Sensor array | TCRT5000 × 5 (analog IR reflectance) |
| Motor driver | L298N dual H-bridge |
| Motors | 2× DC motor with wheels |
| Power | Li-Ion 3.7V (salvaged phone cell) |
| Chassis | Laser-cut acrylic, T-shape |

**Wiring:**

```
TCRT5000 array  →  Arduino Nano A0–A4
L298N IN1       →  D5   (left motor direction A)
L298N IN2       →  D6   (left motor direction B)
L298N IN3       →  D9   (right motor direction A)
L298N IN4       →  D10  (right motor direction B)
L298N ENA (PWM) →  D3   (left motor speed)
L298N ENB (PWM) →  D11  (right motor speed)
L298N 5V out    →  Arduino Nano 5V
Battery 3.7V    →  L298N VCC
```

---

## How It Works

### Sensor array
Five TCRT5000 infrared sensors are mounted at the front of the robot in a linear array. Each sensor returns an analog value — low when over a white surface, high when over a black line.

### Weighted position error
Each sensor is assigned a weight: `[-2, -1, 0, +1, +2]`. The robot computes a weighted average of all sensor readings to get a single error value:
- Error = 0 → robot is centered on the line
- Error < 0 → robot drifted right, needs to steer left
- Error > 0 → robot drifted left, needs to steer right

### PID controller
The error value feeds into a PID controller:

```
output = Kp × error + Ki × ∫error dt + Kd × d(error)/dt
```

The PID output is added/subtracted from the base motor speed to steer the robot back onto the line.

**Tuned values:**
```cpp
float Kp = 25.0;
float Ki = 0.0;
float Kd = 15.0;
```

### Calibration
At startup the robot runs a 3-second calibration routine. Moving the robot over the line and white surface records min/max values per sensor, enabling normalized readings regardless of ambient light.

---

## Files

```
line-follower-pid/
├── firmware/
│   └── line_follower_PID.ino   — main Arduino sketch
├── schematic/
│   └── schema_line_follower.svg — electrical schematic
└── README.md
```

---

## Build Notes

- At 3.7V the L298N drops ~0.5V internally, so motors run at ~3.2V — `BASE_SPEED = 180` compensates
- Disable Serial debug output before competition — it adds ~2ms latency per loop
- Integral term (Ki) is kept at 0 to avoid windup on tight curves — increase only if robot drifts on long straight sections
- Array spacing was ~8mm between sensors — narrower spacing requires higher Kp

---

## Author

**Cosmin Leonardo Cozaciuc**  
Electrical Engineering Student — UPB, Bucharest  
[leonardoczaciuc@gmail.com](mailto:leonardoczaciuc@gmail.com)
