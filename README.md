# IMU Silent SOS (FSM Verified)

Embedded proof-of-concept implementing a silent emergency trigger using an ESP32 and an MPU6050 IMU.

The system detects a deterministic **Up → Down → Up (UDU)** wrist gesture sequence.  
Gesture verification is enforced using a finite state machine (FSM) to prevent accidental triggers.

---

## Demo

[Watch demo video](assets/imu-silent-sos-fsm-video.mp4)

---

## Finite State Machine

![FSM Diagram](assets/fsm_diagram.png)

State flow:

`IDLE → GOT_UP → GOT_DOWN → SOS_DETECTED → COOLDOWN → IDLE`

If the required sequence is not completed within the defined timeout window, the system returns to `IDLE`.

Manual button activation forces a direct transition to `COOLDOWN`.

---

## Gesture Detection Logic

Let:

`dp = pitch - baselinePitch`

Event conditions:

- **UP event** → `dp > angEnter` AND `gyro > gyGate`
- **DOWN event** → `dp < -angEnter` AND `gyro < -gyGate`

The dominant gyroscope axis and direction are determined during calibration.

A 3-sample moving average filter is applied to the gyro signal for basic noise reduction.

---

## Design Approach

Single-threshold gesture detection is highly sensitive to noise and unintended wrist motion.  
Instead of triggering on a single event, this implementation enforces a strict sequential pattern using a finite state machine.

Additional mechanisms:

- Timeout control between transitions  
- Post-detection cooldown window  
- Runtime threshold tuning via serial interface  

This ensures deterministic behavior and reduces false positives.

---

## Hardware Setup

**Board:** ESP32  
**Sensor:** MPU6050 (I2C)

Connections:

- ESP32 3V3 → MPU6050 VCC  
- ESP32 GND → MPU6050 GND  
- ESP32 GPIO21 → MPU6050 SDA  
- ESP32 GPIO22 → MPU6050 SCL  

Manual SOS button:

- GPIO4 → Push button → GND  
  (Configured as `INPUT_PULLUP`)

---

## Sensor Configuration

- Accelerometer range: ±4g  
- Gyroscope range: ±500 deg/s  
- Digital low-pass filter bandwidth: 21 Hz  

---

## Timing Characteristics

- Main loop frequency: ~100 Hz  
- Gesture timeout window: 2 seconds  
- Cooldown duration: 2.5 seconds  

---

## Code Structure

Source file:

`src/imu_silent_sos_fsm_v1_0.ino`

Serial runtime commands:

`+ - h g p c`

---

## Limitations

- Detection reliability depends on calibration quality.
- Sudden non-intentional wrist movements may produce partial state transitions.
- Designed as a demonstration-level embedded system prototype.

---

## License

MIT
