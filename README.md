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
├── src/                  # Project source code
│   ├── jetson/           # Python application running on the Jetson Nano
│   └── arduino/          # Arduino firmware for motor control
├── models/               # YOLO models and exported model files
├── design/               # Robot design files
│   ├── mechanical/       # CAD models and mechanical design
│   └── electronics/      # Circuit designs and wiring diagrams
├── media/                # Photos and videos of the robot
└── docs/                 # Additional documentation and setup guides
```

## Getting Started

### 1. Arduino Setup

Open the Arduino sketch in the Arduino IDE, select the Arduino Mega and the correct port, then upload the firmware.

The serial communication baud rate is set to `115200`.

### 2. Jetson Setup

Install the required Python libraries and configure a compatible JetPack, CUDA, PyTorch, and TensorRT environment for the Jetson Nano.

The development environment used for this project is:

| Component | Version |
|---|---|
| JetPack | 4.6.6 |
| L4T | 32.7.1 |
| CUDA | 10.2 |
| TensorRT | 8.2.1 |
| Python | 3.6.9 |
| PyTorch | 1.10.0 |
| Torchvision | 0.11.0 |
| Ultralytics | 8.0.50 |
| NumPy | 1.19.4 |
| PySerial | 3.5 |

OpenCV is also used by the project. Its version was not identified in the environment check.

Make sure the camera is connected and the model path in `main.py` points to the intended model file.

### 3. Configuration

Camera calibration, camera position, wheel dimensions, model path, and control parameters are configured in the source code and may need to be adjusted for different hardware setups.

**Model Configuration**

The project uses YOLO models. Model files may be available in `.pt`, `.onnx`, and `.engine` formats. The current implementation uses a TensorRT `.engine` file for inference.

Update `MODEL_PATH` in the Python code to point to the model you intend to use. TensorRT engine files may need to be regenerated when moving to a different Jetson, JetPack, or TensorRT environment.

**Camera Calibration and Position**

The camera calibration values were configured for the camera used in this project. When using a different camera, recalibrate it and update the intrinsic matrix (`K`), distortion coefficients (`dist`), and camera resolution.

Camera mounting parameters also affect position estimation. The relevant configuration includes camera dimensions and mounting measurements such as `CAM_W`, `CAM_H`, `CAMERA_HEIGHT`, and `CAMERA_PITCH`.

The camera on the assembled robot is mounted at a 90-degree counterclockwise rotation. The code accounts for this using `rotate_frame()` and `unrotate_pixel()`. If the camera is mounted in a different orientation, these transformations may need to be adjusted accordingly.

For guidance on camera calibration, see the [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.5.4/d4/d94/tutorial_camera_calibration.html).

**Motor and Control Configuration**

Wheel dimensions, encoder specifications, motor PID gains, and position-control parameters are also configured in the source code. These values may need to be tuned when using different motors, wheels, or chassis dimensions.

### 4. Running the Application

Run the application from the repository root:

```bash
python3.6 src/jetson/main.py
```

The `SHOW_WINDOW` setting in the source code controls whether the camera and detection output are displayed. Disable it when running without a graphical display.

### 5. Optional: Automatic Startup

The Jetson Nano can be configured to launch the robot application automatically on boot using a `systemd` service. This is an optional deployment setup and is not currently configured or included in the repository.

See the [systemd service documentation](https://www.freedesktop.org/software/systemd/man/latest/systemd.service.html) for more information.

## Hardware and Design Notes

There are some differences between the original CAD design and the final assembled robot:

- The original CAD design used an ESP32, while the final robot uses an Arduino Mega.
- The motors are mounted underneath the chassis rather than in the positions shown in the original design.
- The camera position in the assembled robot differs from the original CAD design.

The photos in `media/` show the assembled robot, while the mechanical design files are available in `design/mechanical/`. Wiring diagrams and electronic design files are available in `design/electronics/`.

## Project Status

The project brings together computer vision, robotics, embedded systems, and feedback control in a compact, low-cost platform. Further work focuses on testing, calibration, and improving tracking performance.
