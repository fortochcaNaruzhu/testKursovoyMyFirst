"""
Unit tests for replay.playback_controller.PlaybackState.

Covers: index clamp, seek_to_time, speed clamp, advance_index at last step,
get_state. No ROS required.
"""

import unittest

from replay.playback_controller import PlaybackState, SPEED_MIN, SPEED_MAX


class TestPlaybackStateIndexClamp(unittest.TestCase):
    """Test set_index clamps to [0, num_steps-1]."""

    def test_set_index_negative_clamps_to_zero(self) -> None:
        state = PlaybackState(num_steps=10)
        state.set_index(-1)
        playing, index, _ = state.get_state()
        self.assertEqual(index, 0)

    def test_set_index_beyond_num_steps_clamps_to_last(self) -> None:
        state = PlaybackState(num_steps=10)
        state.set_index(100)
        _, index, _ = state.get_state()
        self.assertEqual(index, 9)

    def test_set_index_exactly_num_steps_clamps_to_last(self) -> None:
        state = PlaybackState(num_steps=5)
        state.set_index(5)
        _, index, _ = state.get_state()
        self.assertEqual(index, 4)

    def test_set_index_valid_stays_unchanged(self) -> None:
        state = PlaybackState(num_steps=10)
        state.set_index(3)
        _, index, _ = state.get_state()
        self.assertEqual(index, 3)


class TestPlaybackStateSeekToTime(unittest.TestCase):
    """Test seek_to_time for first, middle, last step."""

    def test_seek_to_time_first_step(self) -> None:
        state = PlaybackState(num_steps=5)
        step_times = [0.0, 0.1, 0.2, 0.3, 0.4]
        state.seek_to_time(0.0, step_times)
        _, index, _ = state.get_state()
        self.assertEqual(index, 0)

    def test_seek_to_time_middle_step(self) -> None:
        state = PlaybackState(num_steps=5)
        step_times = [0.0, 0.1, 0.2, 0.3, 0.4]
        state.seek_to_time(0.25, step_times)
        _, index, _ = state.get_state()
        self.assertEqual(index, 2)

    def test_seek_to_time_last_step(self) -> None:
        state = PlaybackState(num_steps=5)
        step_times = [0.0, 0.1, 0.2, 0.3, 0.4]
        state.seek_to_time(1.0, step_times)
        _, index, _ = state.get_state()
        self.assertEqual(index, 4)

    def test_seek_to_time_empty_step_times_no_op(self) -> None:
        state = PlaybackState(num_steps=5)
        state.set_index(2)
        state.seek_to_time(0.5, [])
        _, index, _ = state.get_state()
        self.assertEqual(index, 2)


class TestPlaybackStateSpeedClamp(unittest.TestCase):
    """Test speed clamped to SPEED_MIN and SPEED_MAX."""

    def test_set_speed_below_min_clamps_to_speed_min(self) -> None:
        state = PlaybackState(num_steps=10, initial_speed=1.0)
        state.set_speed(0.1)
        _, _, speed = state.get_state()
        self.assertEqual(speed, SPEED_MIN)

    def test_set_speed_above_max_clamps_to_speed_max(self) -> None:
        state = PlaybackState(num_steps=10, initial_speed=1.0)
        state.set_speed(10.0)
        _, _, speed = state.get_state()
        self.assertEqual(speed, SPEED_MAX)

    def test_initial_speed_below_min_clamped(self) -> None:
        state = PlaybackState(num_steps=10, initial_speed=0.0)
        _, _, speed = state.get_state()
        self.assertEqual(speed, SPEED_MIN)

    def test_initial_speed_above_max_clamped(self) -> None:
        state = PlaybackState(num_steps=10, initial_speed=5.0)
        _, _, speed = state.get_state()
        self.assertEqual(speed, SPEED_MAX)

    def test_adjust_speed_clamps_at_max(self) -> None:
        state = PlaybackState(num_steps=10, initial_speed=SPEED_MAX)
        result = state.adjust_speed(1.0)
        self.assertEqual(result, SPEED_MAX)
        _, _, speed = state.get_state()
        self.assertEqual(speed, SPEED_MAX)

    def test_adjust_speed_clamps_at_min(self) -> None:
        state = PlaybackState(num_steps=10, initial_speed=SPEED_MIN)
        result = state.adjust_speed(-1.0)
        self.assertEqual(result, SPEED_MIN)
        _, _, speed = state.get_state()
        self.assertEqual(speed, SPEED_MIN)


class TestPlaybackStateAdvanceIndex(unittest.TestCase):
    """Test advance_index at last step and before."""

    def test_advance_index_at_last_step_returns_false(self) -> None:
        state = PlaybackState(num_steps=5)
        state.set_index(4)
        advanced = state.advance_index()
        self.assertFalse(advanced)
        _, index, _ = state.get_state()
        self.assertEqual(index, 4)

    def test_advance_index_before_last_returns_true_and_increments(self) -> None:
        state = PlaybackState(num_steps=5)
        state.set_index(2)
        advanced = state.advance_index()
        self.assertTrue(advanced)
        _, index, _ = state.get_state()
        self.assertEqual(index, 3)

    def test_advance_index_from_zero_increments(self) -> None:
        state = PlaybackState(num_steps=3)
        advanced = state.advance_index()
        self.assertTrue(advanced)
        _, index, _ = state.get_state()
        self.assertEqual(index, 1)


class TestPlaybackStateGetState(unittest.TestCase):
    """Test get_state returns correct (playing, index, speed)."""

    def test_get_state_returns_playing_index_speed(self) -> None:
        state = PlaybackState(num_steps=10, initial_speed=2.0)
        state.set_playing(True)
        state.set_index(3)
        playing, index, speed = state.get_state()
        self.assertTrue(playing)
        self.assertEqual(index, 3)
        self.assertEqual(speed, 2.0)

    def test_get_state_after_toggle_playing(self) -> None:
        state = PlaybackState(num_steps=10)
        state.set_playing(False)
        state.toggle_playing()
        playing, _, _ = state.get_state()
        self.assertTrue(playing)
        state.toggle_playing()
        playing, _, _ = state.get_state()
        self.assertFalse(playing)


class TestPlaybackStateEdgeCases(unittest.TestCase):
    """Edge cases: num_steps=0, single step."""

    def test_num_steps_zero_set_index_clamps_to_zero(self) -> None:
        state = PlaybackState(num_steps=0)
        state.set_index(5)
        _, index, _ = state.get_state()
        self.assertEqual(index, 0)

    def test_num_steps_one_advance_returns_false(self) -> None:
        state = PlaybackState(num_steps=1)
        advanced = state.advance_index()
        self.assertFalse(advanced)
        _, index, _ = state.get_state()
        self.assertEqual(index, 0)


if __name__ == "__main__":
    unittest.main()
