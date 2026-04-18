# 2D Position Visualizer

Real-time 2D plot of drone positions over UDP, plus optional 2D replay from experiment CSV. The live visualizer receives NED coordinates from scenarios during simulation; the replay uses `replay/csv_loader` (read-only, no imports from `core/` or `scenarios/`).

## Quick start

| Goal | Command |
|------|--------|
| **Live 2D** (standalone) | `python visualizer/drone_position_visualizer.py` |
| **Live 2D** (with simulation) | `python launch_simulation.py -s -c leader_forward_back --with-2d-visualizer` |
| **2D Replay** (from CSV) | `python visualizer/replay_2d.py --experiment experiments/<timestamp>` |

All commands are run from the **project root**. The visualizer is optional; simulation runs normally if the visualizer is not started.

## Coordinate convention (NED)

Plots use project **NED** convention: **X = North**, **Y = East**, **Z = Down**. Axes are labeled "X (m, North, NED)" and "Y (m, East, NED)" in both live and replay modes.

## Live 2D (UDP, port 15551)

### Run the live visualizer

From project root:

```bash
# Start visualizer (order does not matter: before or after simulation)
python visualizer/drone_position_visualizer.py

# With options (port, trail length, update interval)
python visualizer/drone_position_visualizer.py --port 15551 --trail 30 --interval 0.1
```

The visualizer listens on **UDP port 15551** (configurable via `--port`). A dedicated daemon thread receives position packets; the thread stops when the window is closed (`receiver.stop()`).

### Run with the launcher

```bash
python launch_simulation.py -s -c leader_forward_back --with-2d-visualizer
```

The launcher starts the 2D visualizer as a subprocess before the scenario. Scenarios that support it (e.g. `leader_forward_back`) call `publish_positions()` from the coordinate exchange loop to send NED positions to the visualizer.

### Live parameters

| Option       | Default | Description                    |
|-------------|---------|--------------------------------|
| `--port -p` | 15551   | UDP port to receive positions  |
| `--trail`   | 30      | Trail length (points); 0 = current only |
| `--interval`| 0.1     | Plot update interval (s)       |

## 2D Replay (CSV)

Play back a recorded experiment in 2D using `replay/csv_loader` (read-only). Run from **project root**:

```bash
python visualizer/replay_2d.py --experiment experiments/2026-02-27_22-02-24
python visualizer/replay_2d.py -e experiments/<timestamp> --rate 2.0 --trail 50
```

Replay uses the same NED convention (X = North, Y = East) as the live visualizer.

| Option          | Default | Description                          |
|-----------------|---------|--------------------------------------|
| `--experiment`  | (required) | Path to experiment directory      |
| `--rate`        | 1.0     | Playback speed multiplier             |
| `--trail`       | 30      | Trail length in steps (0 = current only) |
| `--interval`    | 0.05    | Minimum frame interval (s)            |

## Integration in scenarios

Optional import and call (failures are suppressed so simulation continues without the visualizer):

```python
try:
    from visualizer.position_publisher import publish_positions
    _publish_positions = publish_positions
except ImportError:
    _publish_positions = None

# In coordinate exchange loop, after updating RATES_SHARED:
if _publish_positions is not None:
    _publish_positions(positions, rates=RATES_SHARED)
```

API: `publish_positions(positions, host="127.0.0.1", port=15551, rates=None)`.

## Script paths

| Script | Purpose |
|--------|--------|
| `visualizer/drone_position_visualizer.py` | Live 2D over UDP (default port **15551**) |
| `visualizer/position_publisher.py` | Called by scenarios to send positions to the visualizer |
| `visualizer/replay_2d.py` | 2D replay from experiment CSV (uses `replay/csv_loader`) |

## Dependencies

- **Live:** `matplotlib`; stdlib: `socket`, `json`, `threading`
- **Replay:** `matplotlib`; `replay.csv_loader` (no `core/` or `scenarios/` imports)

---

**Summary:** The 2D visualizer provides (1) live X–Y plot over UDP on port 15551, started manually or via `--with-2d-visualizer`, and (2) 2D replay from experiment CSV via `replay_2d.py --experiment <path>`. Both use NED (X=North, Y=East). Scenarios integrate by calling `publish_positions()` from the coordinate exchange loop; see *Integration in scenarios* above.
