# Replay module

Load experiment logs (CSV + metadata) and play them back in ROS/RViz for 3D visualization. Two launch options: **(1) two terminals** (replay in the first, RViz in the second) or **(2) single-command script** that starts replay then RViz. See also `docs/new_Noetic_replay-setup.md`.

**Prerequisites (both options):** Docker, X11. Once per session run `export DISPLAY=:0` and `xhost +local:docker` so containers can use the host display.

## Launch options

### Option 1: Two terminals (current flow)

Use when you want to keep replay and RViz in separate terminals (e.g. to see replay logs). Steps below.

### Option 2: Single-command script

From project root, one script starts the replay container in the background and then runs the RViz container. When you close RViz, the replay container is stopped automatically.

```bash
export DISPLAY=:0
xhost +local:docker

./scripts/replay-with-rviz.sh EXPERIMENT_PATH [RATE] [INTERACTIVE]
```

- **EXPERIMENT_PATH** — path to experiment dir (e.g. `experiments/2026-02-28_18-35-11`).
- **RATE** — optional speed 0.25–4.0 (default 1.0).
- **INTERACTIVE** — optional: `1`, `true`, or `yes` for play/pause/seek/speed via ROS topics.

**Environment:** Set `DISPLAY` (e.g. `:0`) and run `xhost +local:docker` once per session so both containers can use the host X11.

### Option 3: One script — Docker + RViz + GUI (no ROS on host)

If **ROS is not installed on the host**, use this script. It starts the replay container (interactive) and a second container that runs both RViz and the replay GUI; both use ROS inside Docker. You get 3D view in RViz and a control window (play/pause/speed/seek) without installing ROS locally.

```bash
export DISPLAY=:0
xhost +local:docker

./scripts/replay-with-rviz-and-gui.sh EXPERIMENT_PATH [RATE]
```

- **EXPERIMENT_PATH** — path to experiment dir (e.g. `experiments/2026-02-28_18-35-11`).
- **RATE** — optional speed 0.25–4.0 (default 1.0).

When you close RViz, the script stops the replay container. The GUI window runs in the same container as RViz and closes together with it.

The script uses a custom image `swarm-replay-rviz-gui:noetic` (ROS Noetic + python3-tk). On first run it is built once from `docker/Dockerfile.replay-rviz-gui`; later runs start immediately without installing tkinter in the container.

---

## CSV format

Per-drone CSV files have header:

```text
t,x,y,z,rx,ry,rz,hasCollision
```

- `t`: time from experiment start (seconds)
- `x`, `y`, `z`: position in NED frame (meters)
- `rx`, `ry`, `rz`: Euler angles roll, pitch, yaw (radians)
- `hasCollision`: 0 or 1

## metadata.json

- `duration_sec`: max experiment duration (0 = no limit)
- `collision_radius_m`: drone sphere radius for collision detection (m)
- `num_drones`: number of drones
- `scenario`: scenario name

## Launch order (Docker, two terminals) — Option 1

Use **ROS 1 Noetic** in Docker. Replay and roscore run in the first container; RViz runs in a second container (same host network).

### Step 1. First terminal: start replay (roscore + replay in one container)

From project root:

```bash
export DISPLAY=:0
xhost +local:docker

./scripts/replay-docker.sh EXPERIMENT_PATH [RATE] [INTERACTIVE]
```

- **EXPERIMENT_PATH** — path to experiment dir (e.g. `experiments/2026-02-28_18-35-11`).
- **RATE** — optional speed 0.25–4.0 (default 1.0).
- **INTERACTIVE** — optional: `1`, `true`, or `yes` to enable play/pause/seek/speed via ROS topics (keyboard not available in Docker).

Leave this terminal/container running.

### Step 2. Second terminal (host): X11 once per session

```bash
export DISPLAY=:0
xhost +local:docker
```

### Step 3. Second terminal: start container for RViz

```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/user/Документы/Kursach/Drone_Swarm_Simulator_v2:/workspace \
  --network host \
  osrf/ros:noetic-desktop-full \
  bash
```

Replace the path in `-v` if the project is elsewhere. `LIBGL_ALWAYS_SOFTWARE=1` avoids libGL/amdgpu errors in the container; omit if you use host GPU passthrough.

### Step 4. Inside the second container: run RViz

```bash
source /opt/ros/noetic/setup.bash
rosrun rviz rviz
```

### Step 5. Configure RViz for drones

1. **Fixed Frame:** In Global Options set **Fixed Frame** to `world` (same as `frame_id` in `replay_rviz.py`).
2. **Add poses:** Add → By topic → `/swarm/drone_1/pose` (PoseStamped). Add `/swarm/drone_2/pose`, `/swarm/drone_3/pose`, etc. for more drones.
3. Optionally set display type (Arrow/Axes), color, and scale per pose.

### Step 6. Control playback (first terminal / first container)

- **Keyboard** (if replay is run on host with TTY): `Space` = play/pause, `a`/`d` or Left/Right = step, `+`/`-` = speed, `q` = quit.
- **ROS topics** (works in Docker): publish to `/replay/play`, `/replay/pause`, `/replay/seek` (Float64), `/replay/speed` (Float32). See table below.

---

## Replay with GUI

A small graphical control window (tkinter) publishes to the same ROS topics as above. **Start the replay process with `--interactive` first**, then start the GUI so both use the same ROS master.

**No ROS on host?** Use one script: `./scripts/replay-with-rviz-and-gui.sh EXPERIMENT_PATH [RATE]` — replay, RViz and GUI all run in Docker (see Option 3 above).

### Launch order (replay with GUI)

1. **Start replay with interactive mode** (so it subscribes to `/replay/play`, `/replay/pause`, `/replay/seek`, `/replay/speed`):
   - **Host:** `python replay/replay_rviz.py --experiment <PATH> [--rate R] --interactive`
   - **Docker:** `./scripts/replay-with-rviz.sh EXPERIMENT_PATH [RATE] 1` or `./scripts/replay-docker.sh EXPERIMENT_PATH [RATE] 1`
2. **Start the GUI** (same host or same Docker network so ROS_MASTER_URI matches):
   - **Host (ROS on host):** `source /opt/ros/noetic/setup.bash && python replay/replay_gui.py --experiment <PATH>`
   - Without `--experiment`: run `python replay/replay_gui.py --gui-only` for play/pause/speed only (no seek slider range).

**One-command host flow (replay + GUI):** With `roscore` already running in another terminal, run:  
`./scripts/replay-with-gui.sh EXPERIMENT_PATH [RATE]` — this starts replay in the background and then the GUI; closing the GUI stops the replay.

### GUI controls

| Control   | Action                                                                 |
|----------|------------------------------------------------------------------------|
| **Play** | Publishes `/replay/play` (Empty) — start or resume playback.           |
| **Pause**| Publishes `/replay/pause` (Empty) — pause playback.                    |
| **Speed**| Slider 0.25–4.0 — publishes `/replay/speed` (Float32).                  |
| **Seek** | Slider 0 to duration (s) — publishes `/replay/seek` (Float64).        |

Seek range is computed from the experiment CSV (min/max time) when `--experiment` is given; otherwise the seek slider is omitted (use `--gui-only`).

### Docker and GUI

- **Replay in Docker, GUI on host:** If ROS Noetic runs on the host and can reach the replay container’s roscore (e.g. same network or ROS_MASTER_URI pointing to the container), run the GUI on the host: `python replay/replay_gui.py --experiment experiments/...`. The replay scripts use `--network host`, so roscore in the container is on `localhost:11311`; on the host set `export ROS_MASTER_URI=http://localhost:11311` and run the GUI to control replay in the container.
- **GUI inside replay container:** The replay Docker image may not show a tkinter window by default; for a GUI inside Docker you’d need X11 forwarding or a VNC/desktop. Prefer running the GUI on the host when ROS is available there.

---

## Arguments (replay_rviz.py)

| Argument        | Required | Description                                                                 |
|-----------------|----------|-----------------------------------------------------------------------------|
| `--experiment`  | Yes      | Path to experiment directory (contains `metadata.json` and `drone_*.csv`).  |
| `--rate`        | No       | Playback speed multiplier (default 1.0). With `--interactive`, initial speed.|
| `--interactive` | No       | Enable play/pause, seek, speed via keyboard and ROS topics.                 |

**replay_gui.py:** `--experiment PATH` — experiment directory (for seek slider range); `--gui-only` — no experiment path, play/pause/speed only.

## ROS topics

| Topic                     | Type                       | Description                                      |
|---------------------------|----------------------------|--------------------------------------------------|
| `/swarm/drone_<id>/pose`  | `geometry_msgs/PoseStamped`| Pose of drone `id` (ENU frame).                  |
| `/swarm/metadata`         | `std_msgs/String`          | Experiment metadata JSON (once at start).         |
| `/replay/play`            | `std_msgs/Empty`           | Start or resume playback (interactive mode).     |
| `/replay/pause`           | `std_msgs/Empty`           | Pause playback (interactive mode).               |
| `/replay/seek`            | `std_msgs/Float64`         | Seek to time in seconds (interactive mode).       |
| `/replay/speed`           | `std_msgs/Float32`         | Set speed multiplier 0.25–4.0 (interactive mode).|

Poses are published in ENU for RViz (converted from NED in the CSV). Use Ctrl+C to stop replay (SIGINT handled).

---

## ROS 2 (RViz 2) — воспроизведение тех же логов

Формат эксперимента **тот же**: `metadata.json` и `drone_1.csv`, `drone_2.csv`, … Число дронов на экране = числу CSV / полю `num_drones` в метаданных. Нода публикует траектории из записанных строк (NED → ENU), по шагам синхронизированным по времени `t`, как в ROS 1.

**Зависимости:** установленный ROS 2 (например **Humble** или **Jazzy**), пакеты `rclpy`, `geometry_msgs`, `std_msgs` (обычно уже в desktop).

### Запуск

Из корня проекта (после `source /opt/ros/<дистрибутив>/setup.bash`):

**Один терминал — RViz2 открывается сам** (конфиг `replay/rviz2_swarm_replay.rviz`: сетка + `/swarm/markers`, Fixed Frame `world`):

```bash
python replay/replay_rviz2.py --experiment experiments/2026-02-28_18-35-11 --rate 1.0 --rviz
```

Свой файл конфигурации:

```bash
python replay/replay_rviz2.py --experiment ... --rviz --rviz-config /путь/к/моему.rviz
```

**Два терминала** (как раньше): без `--rviz` во втором запускаете `rviz2` и настраиваете вручную.

```bash
# Терминал 1 — нода воспроизведения
python replay/replay_rviz2.py --experiment experiments/2026-02-28_18-35-11 --rate 1.0

# Терминал 2 — RViz 2
source /opt/ros/humble/setup.bash   # или jazzy
rviz2
```

Опции:

| Аргумент | Описание |
|----------|-----------|
| `--experiment` | Каталог с `metadata.json` и `drone_*.csv` (обязательно). |
| `--rate` | Множитель скорости (по умолчанию 1.0). |
| `--interactive` | Пауза/seek/скорость с клавиатуры и топиков `/replay/*` (как в ROS 1). |
| `--frame-id` | `frame_id` в PoseStamped (по умолчанию `world` — задайте такой же **Fixed Frame** в RViz2). |
| `--rviz` | Запустить **RViz2** с готовым конфигом (`replay/rviz2_swarm_replay.rviz`). |
| `--rviz-config` | Путь к своему `.rviz` (если не указан — используется `replay/rviz2_swarm_replay.rviz`). |
| `--viz-substeps N` | До **N** промежуточных кадров между строками лога. `1` = только CSV. Слишком большое N может **забить RViz** и дать джиттер при том, что счётчик FPS вырос. |
| `--viz-cap-hz HZ` | Ограничить среднюю частоту публикаций (Гц). `0` = без лимита. При рывках: `--viz-substeps 6 --viz-cap-hz 30`. |

В `replay/rviz2_swarm_replay.rviz` по умолчанию **30 FPS**; средний FPS в углу RViz ≠ плавность — при «рваном» виде сначала уменьните `--viz-substeps` или включите `--viz-cap-hz`, а не только поднимайте FPS в RViz.

### Настройка RViz2

**С флагом `--rviz`** настраивать ничего не нужно: откроется окно с сеткой и дисплеем **Swarm markers** на топик `/swarm/markers`.

**Автоматически (рекомендуется):** нода публикует **`/swarm/markers`** (`visualization_msgs/MarkerArray`) — сферы для всех дронов разными цветами; при столкновении (`hasCollision`) цвет красный.

1. **Global Options → Fixed Frame:** `world` (или значение `--frame-id`).
2. **Add → By topic** → выберите **`/swarm/markers`** → RViz2 сам предложит дисплей **MarkerArray**; одного дисплея достаточно для **любого** числа дронов из лога.

**Вручную (как раньше):** для каждого дрона отдельно **Add → By topic** → `/swarm/drone_1/pose`, `/swarm/drone_2/pose`, … (`geometry_msgs/PoseStamped`).

Отключить маркеры, если нужны только позы: **`--no-markers`**.

Топики: `/swarm/markers`, `/swarm/drone_<id>/pose`, `/swarm/metadata` (`std_msgs/String` JSON). Интерактивное управление: `/replay/play`, `/replay/pause`, `/replay/seek` (`Float64`), `/replay/speed` (`Float32`).

Скрипт: `replay/replay_rviz2.py`.

---

## Running without Docker (host with ROS Noetic)

If ROS Noetic is installed on the host:

```bash
# Terminal 1
source /opt/ros/noetic/setup.bash
roscore

# Terminal 2
cd /path/to/Drone_Swarm_Simulator_v2
python replay/replay_rviz.py --experiment experiments/2025-01-15_10-30-00 --rate 1.0

# Terminal 3 (optional, for interactive controls use --interactive in terminal 2)
source /opt/ros/noetic/setup.bash
rosrun rviz rviz
```

Or as module: `python -m replay.replay_rviz --experiment experiments/... --rate 1.0`.

---

## References

- Full setup and troubleshooting: `docs/new_Noetic_replay-setup.md`
- Script usage and workspace path: `scripts/replay-docker.sh` (default project root = directory above `scripts/`)
- **Исправления 2D-визуализатора и MAVLink (01.03.26):** в `docs/found errors/01_03_26_errors_in_project.md` описаны внесённые изменения: один поток на одно MAVLink-подключение (MAVLinkWorker), отказ от хранения истории в реальном времени в 2D-визуализаторе (только текущие позиции).
- **Исправления replay и core (03.03.26):** в `docs/found errors/03_03_26_errors_in_project.md` — один скрипт для Docker replay+RViz, синхронное пошаговое воспроизведение в RViz, начальные позиции дронов по Y (+2 м на дрон), вынос DroneController в `core/control`.
