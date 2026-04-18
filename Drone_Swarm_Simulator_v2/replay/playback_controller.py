"""
Playback state and controls for replay (seek, speed, play/pause).

Thread-safe state holder used by replay_rviz for interactive playback.
No ROS or CSV dependencies; keeps replay_rviz as a thin ROS publisher.
"""

import threading
from typing import List, Tuple

# Speed limits for playback (same as reference 0.25–4.0)
SPEED_MIN: float = 0.25
SPEED_MAX: float = 4.0


class PlaybackState:
    """Thread-safe playback state: playing, current_index, speed."""

    def __init__(self, num_steps: int, initial_speed: float = 1.0) -> None:
        """Initialize state.

        Args:
            num_steps: Total number of steps (len(steps)); index in [0, num_steps-1].
            initial_speed: Initial speed multiplier (clamped to SPEED_MIN..SPEED_MAX).
        """
        self._lock = threading.Lock()
        self._playing = False
        self._current_index = 0
        self._num_steps = max(0, num_steps)
        self._speed = self._clamp_speed(initial_speed)

    @staticmethod
    def _clamp_speed(s: float) -> float:
        """Clamp speed to [SPEED_MIN, SPEED_MAX]."""
        return max(SPEED_MIN, min(SPEED_MAX, float(s)))

    def get_state(self) -> Tuple[bool, int, float]:
        """Return (playing, current_index, speed) under lock.

        Returns:
            Tuple of (is_playing, current_step_index, speed_multiplier).
        """
        with self._lock:
            return (self._playing, self._current_index, self._speed)

    def set_playing(self, playing: bool) -> None:
        """Set play/pause state.

        Args:
            playing: True to play, False to pause.
        """
        with self._lock:
            self._playing = bool(playing)

    def toggle_playing(self) -> bool:
        """Toggle playing; return new value.

        Returns:
            New playing state after toggle.
        """
        with self._lock:
            self._playing = not self._playing
            return self._playing

    def set_index(self, index: int) -> None:
        """Set current index; clamp to [0, num_steps-1].

        Args:
            index: Desired step index.
        """
        with self._lock:
            self._current_index = max(0, min(self._num_steps - 1, int(index)))

    def seek_to_time(self, t: float, step_times: List[float]) -> None:
        """Seek to step with time <= t.

        Args:
            t: Target time in seconds.
            step_times: step_times[i] = steps[i][0] (time at each step).
        """
        if not step_times:
            return
        # Largest i such that step_times[i] <= t
        idx = 0
        for i, st in enumerate(step_times):
            if st <= t:
                idx = i
            else:
                break
        with self._lock:
            self._current_index = max(0, min(self._num_steps - 1, idx))

    def advance_index(self) -> bool:
        """Increment current_index if not at end.

        Returns:
            True if index was advanced, False if already at last step.
        """
        with self._lock:
            if self._current_index < self._num_steps - 1:
                self._current_index += 1
                return True
            return False

    def set_speed(self, speed: float) -> None:
        """Set playback speed multiplier (clamped).

        Args:
            speed: Speed multiplier (e.g. 1.0 = real-time).
        """
        with self._lock:
            self._speed = self._clamp_speed(speed)

    def adjust_speed(self, delta: float) -> float:
        """Add delta to speed (e.g. +0.25 or -0.25).

        Args:
            delta: Change in speed (can be negative).

        Returns:
            New speed after clamping.
        """
        with self._lock:
            self._speed = self._clamp_speed(self._speed + delta)
            return self._speed
