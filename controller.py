class PID:
    """
    A simple PID controller with anti‑windup and derivative on measurement.
    """
    def __init__(self, Kp, Ki, Kd, setpoint=0.0,
                 sample_time=None, output_limits=(None, None)):
        """
        Parameters:
            Kp, Ki, Kd : float
                Proportional, integral, derivative gains.
            setpoint : float, optional
                Initial target value.
            sample_time : float, optional
                Fixed time step (seconds). If None, you must pass `dt` in compute().
            output_limits : tuple of (min, max), optional
                Lower and upper bounds for the controller output.
                Use None for no bound (e.g., (None, 10.0)).
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self.sample_time = sample_time
        self.output_limits = output_limits

        # Internal state
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_measurement = None   # for derivative on measurement
        self._last_output = 0.0
        self._last_time = None

    def reset(self):
        """Reset the internal integrator and derivative state."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_measurement = None
        self._last_output = 0.0
        self._last_time = None

    def set_setpoint(self, setpoint):
        """Change the setpoint without resetting the controller."""
        self.setpoint = setpoint

    def compute(self, error=None, measurement=None, dt=None):
        """
        Compute the PID output.

        You must provide EITHER:
            - error = setpoint - measurement
        OR
            - measurement (then the error is computed using the internal setpoint)

        If `dt` is not given, the class uses its `sample_time` (must have been set).
        """
        # Determine error
        if error is None and measurement is None:
            raise ValueError("Either 'error' or 'measurement' must be supplied.")
        if error is not None:
            # Use the error directly; do not update internal setpoint
            pass
        else:
            error = self.setpoint - measurement

        # Determine time step
        if dt is None:
            if self.sample_time is None:
                raise ValueError("No dt provided and no sample_time set.")
            dt = self.sample_time

        if dt <= 0:
            raise ValueError("dt must be positive.")

        # ----- Proportional term -----
        P = self.Kp * error

        # ----- Integral term with anti‑windup -----
        self._integral += error * dt
        # Clamp integral to prevent windup (optional)
        if self.output_limits[0] is not None or self.output_limits[1] is not None:
            # Pre‑clamp the integral so that P + I + D won't exceed limits (simple method)
            # Better: use a back‑calculation method, but this works for many cases.
            i_max = (self.output_limits[1] - P - self.Kd * (error - self._prev_error)/dt) / self.Ki if self.Ki != 0 else float('inf')
            i_min = (self.output_limits[0] - P - self.Kd * (error - self._prev_error)/dt) / self.Ki if self.Ki != 0 else -float('inf')
            if self.Ki > 0:
                self._integral = max(min(self._integral, i_max), i_min)
            # If Ki is negative, invert the logic (not common), but we ignore for simplicity.
        I = self.Ki * self._integral

        # ----- Derivative term (on measurement, not error, to avoid derivative kick) -----
        if measurement is not None:
            # Derivative on measurement: D = -Kd * (measurement - prev_measurement) / dt
            if self._prev_measurement is None:
                derivative = 0.0
            else:
                derivative = -self.Kd * (measurement - self._prev_measurement) / dt
        else:
            # Fallback: derivative on error (classic)
            derivative = self.Kd * (error - self._prev_error) / dt

        # ----- Compute output and clamp -----
        output = P + I + derivative
        if self.output_limits[0] is not None:
            output = max(output, self.output_limits[0])
        if self.output_limits[1] is not None:
            output = min(output, self.output_limits[1])

        # ----- Store state for next call -----
        self._prev_error = error
        if measurement is not None:
            self._prev_measurement = measurement
        self._last_output = output
        self._last_time = dt  # store for possible use

        return output



if __name__ == "__main__":
    # Create a PID controller with gains
    pid = PID(Kp=2.0, Ki=0.5, Kd=1.0,
              setpoint=10.0,
              sample_time=0.01,          # fixed time step
              output_limits=(-300.0, 300.0)) # output clamped between -10 and 10



