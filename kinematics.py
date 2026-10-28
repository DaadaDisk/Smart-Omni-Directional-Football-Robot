import math
from serial_ import ArduinoSerial
class Kinematics:
    def __init__(self, R=0.135, r_w=0.04):
        self.R = R
        self.r_w = r_w
    def compute_wheel_rpm(self, vx, vy, omega):
        """
        Compute the RPM of each of the three omni wheels.
    
        Parameters:
            vx, vy : float
                Desired linear velocity of the robot in its local frame (m/s).
            omega : float
                Desired angular velocity about the robot's vertical axis (rad/s).
            R : float, optional
                Distance from the robot center to each wheel's contact point (m). Default 0.2.
            r_w : float, optional
                Radius of each wheel (m). Default 0.05.
    
        Returns:
            list of float
                RPM of wheels 1, 2, 3 in order.
                Wheel 1 has its axle aligned with the robot's Y‑axis.
                Wheels 2 and 3 are at +120° and +240° from wheel 1.
        """
        # Angles of the three wheel positions (radians)
        # Wheel 1 is on the positive Y-axis (θ = 90°)
        # Wheel 2 at θ = 90° + 120° = 210°
        # Wheel 3 at θ = 90° + 240° = 330°
        theta = [math.pi/2, 7*math.pi/6, 11*math.pi/6]
        rpms = []
        for th in theta:
            # Linear speed of the wheel's contact point along its rolling direction (tangential to the circle)
            # s = -vx*sin(θ) + vy*cos(θ) + ω*R
            s = -vx * math.sin(th) + vy * math.cos(th) + omega * self.R
    
            # Convert linear speed (m/s) to RPM
            rpm = s * 60 / (2 * math.pi * self.r_w)
            rpms.append(rpm)
        rpms = [-rpms[2], -rpms[0], -rpms[1]] 
        return rpms

if __name__ == "__main__":
    arduino = ArduinoSerial()
    arduino.open()
    kin = Kinematics()
    import time
    time.sleep(1)
    while True:
        x = input('>')
        if 'q' in x.lower():
            break
        v = [float(i) for i in x.split(' ')]
        w = kin.compute_wheel_rpm(v[0], v[1], v[2])
        line = f"{w[0]},{w[1]},{w[2]}\n"
        arduino.send(line)

    arduino.close()
