# Smart-Omni-Directional-Football-Robot
An AI-powered omni-directional robot that detects and tracks a football using computer vision and YOLO on an NVIDIA Jetson Nano. It uses inverse kinematics to calculate wheel speeds and an Arduino Mega with encoder feedback and PID control to move toward the ball, using three omni wheels for movement in any direction.

## How It Works

1. **Ball Detection:** A camera streams frames to the Jetson Nano, where YOLO detects the ball.
2. **Position Estimation:** OpenCV and camera calibration are used to estimate the ball's position relative to the robot.
3. **Motion Calculation:** Position control and inverse kinematics determine the required speed of each wheel.
4. **Motor Control:** The Jetson sends wheel-speed commands to the Arduino Mega over serial communication.
5. **Feedback Control:** Encoders measure motor speed, and PID controllers adjust the motor outputs to follow the target speeds.

## Hardware

- NVIDIA Jetson Nano
- Arduino Mega
- USB camera
- 3 omni wheels
- 3 DC motors with encoders
- Motor driver hardware
- Robot chassis, battery, and wiring

## Software & Technologies

- **Python:** Main application and motion calculations
- **YOLO:** Ball detection
- **OpenCV & NumPy:** Image processing and position estimation
- **PyTorch & TensorRT:** Model inference
- **C++ / Arduino:** Motor control and encoder feedback
- **UART / Serial:** Communication between the Jetson and Arduino
- **PID Control:** Motor speed regulation

## Repository Structure

```text
.
├── README.md
├── src/
│   ├── jetson/
│   │   ├── main.py
│   │   ├── kinematics.py
│   │   ├── controller.py
│   │   └── serial_.py
│   └── arduino/
│       └── PID2.ino
├── models/
│   ├── *.pt
│   └── *.engine
├── design/
│   ├── mechanical/
│   └── electronics/
├── media/
│   ├── photos/
│   └── videos/
└── docs/
```

## Getting Started

### 1. Arduino Setup

Open the Arduino sketch in the Arduino IDE, select the Arduino Mega and the correct port, then upload the firmware.

The serial communication baud rate is set to `115200`.

### 2. Jetson Setup

Install the required Python libraries and configure a compatible JetPack, CUDA, PyTorch, and TensorRT environment for the Jetson Nano.

Make sure the camera is connected and the model path in `main.py` points to the intended model file.

Run the application from the repository root:

```bash
python3 src/jetson/main.py
```

### 3. Configuration

Camera calibration, camera position, wheel dimensions, model path, and control parameters are configured in the source code and may need to be adjusted for different hardware setups.

## Project Status

The project brings together computer vision, robotics, embedded systems, and feedback control in a compact, low-cost platform. Further work focuses on testing, calibration, and improving tracking performance.

