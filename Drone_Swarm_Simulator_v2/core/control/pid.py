"""
PID controller with anti-windup and filtered derivative.
"""

import time
from typing import Optional


class PIDRegulator:
    """
    PID regulator with integral limit (anti-windup), output limit,
    and optional derivative low-pass (derivative_alpha).
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        integral_limit: float = 1000.0,
        output_limit: float = 500.0,
        derivative_alpha: float = 0.0,
    ) -> None:
        """
        Initialize the PID regulator.

        Args:
            kp: Proportional gain.
            ki: Integral gain.
            kd: Derivative gain.
            integral_limit: Clamp for integral term (anti-windup); 0 = no limit.
            output_limit: Clamp for total output (symmetric ±output_limit).
            derivative_alpha: Low-pass factor for derivative (0..1); 0 = no filtering.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.derivative_alpha = max(0.0, min(1.0, derivative_alpha))
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time: Optional[float] = None
        self._derivative_filtered = 0.0

    def update(self, error: float, dt: Optional[float] = None) -> float:
        """
        Update regulator with current error and return control output.

        Args:
            error: Current tracking error.
            dt: Time step since last update; if None, computed from wall time.

        Returns:
            Clamped PID output.
        """
        current_time = time.time()
        if dt is None:
            if self.last_time is None:
                dt = 0.1
            else:
                dt = current_time - self.last_time
                if dt <= 0:
                    dt = 0.1
        self.last_time = current_time

        p_term = self.kp * error
        self.integral += error * dt
        if self.integral_limit > 0:
            self.integral = max(
                -self.integral_limit,
                min(self.integral_limit, self.integral),
            )
        i_term = self.ki * self.integral

        if dt > 0:
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0.0
        if self.derivative_alpha > 0:
            self._derivative_filtered = (
                self.derivative_alpha * self._derivative_filtered
                + (1.0 - self.derivative_alpha) * derivative
            )
            derivative = self._derivative_filtered
        d_term = self.kd * derivative
        self.last_error = error

        output = p_term + i_term + d_term
        output = max(-self.output_limit, min(self.output_limit, output))
        return output

    def reset(self) -> None:
        """Reset integral, last error, last time, and filtered derivative."""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self._derivative_filtered = 0.0
