# Логирование `antena_logic` и воспроизведение в ROS2 / RViz2

Документ описывает, **как пишутся логи** сценария `Drone_Swarm_Simulator_v2/scenarios/antena_logic.py`, **с какой частотой**, **где регулировать частоту**, **для кого** создаются файлы, а также **как эти логи читаются и визуализируются** через ROS2 и RViz2 (скрипт `replay/replay_rviz2.py`).

---

## 1. Что именно записывается (формат «эксперимента» для replay)

Сценарий `antena_logic` создаёт каталог эксперимента, в котором лежат:

- `metadata.json` — метаданные запуска (обязательные поля для загрузчика replay).
- `drone_1.csv`, `drone_2.csv`, `drone_3.csv`, `drone_4.csv` — **по одному CSV на каждый дрон**.

Заголовок каждого `drone_*.csv` строго такой (требование `replay/csv_loader.py`):

```text
t,x,y,z,rx,ry,rz,hasCollision
```

Смысл колонок (NED, метры и радианы):

- `t` — время **относительно начала записи** (секунды), считается как `now - START_TIME`.
- `x, y, z` — позиция в **общей NED-рамке** (см. ниже про «common frame»).
- `rx, ry, rz` — углы Эйлера (рад), берутся из `coords_monitor.get_attitude()`; при ошибке — нули.
- `hasCollision` — сейчас всегда `0` (столкновения в этом сценарии не вычисляются).

---

## 2. Для кого пишутся логи

Логи пишутся **для всех дронов сценария** (в текущей конфигурации это **ровно 4 дрона**): каждому `id` соответствует свой файл `drone_<id>.csv`.

Якорь (`--anchor-id`) **не исключается** из логирования: он тоже попадает в свой `drone_<anchor_id>.csv` наравне с остальными.

---

## 3. С какой частотой пишутся логи и что является «тактом» записи

### 3.1. Где происходит запись

Запись выполняется внутри фонового потока `exchange_loop()` в `antena_logic.py`:

1. На каждой итерации цикла собирается снимок позиций всех дронов в **общей NED-рамке** (`pos_common`).
2. После этого (при соблюдении условия по времени) для **каждого** дрона вызывается `write_row(...)`.

Код записи строк и ограничения частоты:

```496:550:/home/user/Kursov3/Drone_Swarm_Simulator_v2/scenarios/antena_logic.py
    def exchange_loop() -> None:
        last_pub = 0.0
        pub_period = 1.0 / 20.0
        last_log_time = 0.0
        log_hz = float(getattr(args, "log_hz", 20.0))
        log_period = (1.0 / log_hz) if log_hz and log_hz > 0 else 0.0
        while True:
            if STOP_EVENT.is_set():
                return
            pos_common: Dict[int, Dict[str, float]] = {}
            for c in controllers:
                did = int(c.config["id"])
                pos_common[did] = _pos_common(c)
            # ...
            now = time.time()
            if log_period <= 0.0 or (now - last_log_time) >= log_period:
                t_rel = now - START_TIME if START_TIME > 0 else 0.0
                for c in controllers:
                    did = int(c.config["id"])
                    p = pos_common.get(did) or {"x": 0.0, "y": 0.0, "z": 0.0}
                    att = {"rx": 0.0, "ry": 0.0, "rz": 0.0}
                    if c.coords_monitor is not None:
                        try:
                            att = c.coords_monitor.get_attitude()
                        except Exception:
                            att = att
                    try:
                        write_row(
                            experiment_log_files[did],
                            did,
                            float(t_rel),
                            float(p.get("x", 0.0)),
                            float(p.get("y", 0.0)),
                            float(p.get("z", 0.0)),
                            float(att.get("rx", 0.0)),
                            float(att.get("ry", 0.0)),
                            float(att.get("rz", 0.0)),
                            0,
                        )
                    except Exception:
                        pass
                last_log_time = now
            # ...
            time.sleep(0.02)
```

### 3.2. Частота итераций цикла обмена

В конце цикла стоит `time.sleep(0.02)`, то есть «тик обмена» примерно **50 Гц** (если вычисления быстрые).

### 3.3. Частота записи в CSV

Параметр CLI:

- `--log-hz` (по умолчанию **20.0**).

Логика:

- если `--log-hz > 0`, то записываем не чаще, чем раз в `1 / log_hz` секунд;
- если `--log-hz 0`, то `log_period` становится `0`, и запись выполняется **на каждом тике** `exchange_loop` (практически до **50 Гц**).

Определение аргумента:

```378:395:/home/user/Kursov3/Drone_Swarm_Simulator_v2/scenarios/antena_logic.py
    parser.add_argument(
        "--log-hz",
        type=float,
        default=20.0,
        help="Log write rate (Hz). 0 = write every exchange tick (50 Hz).",
    )
```

### 3.4. Где регулировать частоту

- **Прямо при запуске сценария**: флаг `--log-hz`.
- **Через лаунчер** `Drone_Swarm_Simulator_v2/launch_simulation.py`: он пробрасывает аргументы подпроцессу сценария; если добавить в команду `--log-hz 30`, значение дойдёт до `antena_logic.py` (аналогично `--duration`, `--experiment-dir`, `--run-id`).

---

## 4. Куда пишутся файлы (каталог эксперимента)

Путь к каталогу задаётся так:

1. Если указан `--experiment-dir`, используется он (абсолютный путь).
2. Иначе, если указан `--run-id`, используется `Drone_Swarm_Simulator_v2/experiments/exp_<run-id>/`.
3. Иначе создаётся папка с меткой времени `Drone_Swarm_Simulator_v2/experiments/antena_logic_YYYY-MM-DD_HH-MM-SS/`.

Фрагмент:

```459:494:/home/user/Kursov3/Drone_Swarm_Simulator_v2/scenarios/antena_logic.py
    experiment_dir = args.experiment_dir
    if experiment_dir is None:
        if args.run_id:
            experiment_dir = os.path.join(_project_root, "experiments", f"exp_{args.run_id}")
        else:
            experiment_dir = os.path.join(
                _project_root, "experiments", time.strftime("antena_logic_%Y-%m-%d_%H-%M-%S")
            )
    experiment_dir = os.path.abspath(str(experiment_dir))
    os.makedirs(experiment_dir, exist_ok=True)
    # ... открытие drone_<id>.csv ...
    write_metadata(
        experiment_dir,
        float(args.duration),
        0.2,
        num_drones,
        "antena_logic",
        extra={...},
    )
```

`metadata.json` пишется через `write_metadata` из `core/logging/csv_logger.py`.

---

## 5. Где основной и вспомогательный код записи логов

### 5.1. Основная логика (сценарий)

Файл: `Drone_Swarm_Simulator_v2/scenarios/antena_logic.py`

- **Создание каталога**, открытие файлов `drone_*.csv`, запись `metadata.json` — в `main()`.
- **Потоковая запись строк** — внутри `exchange_loop()` (см. раздел 3).

Импорт вспомогательных функций записи:

```29:31:/home/user/Kursov3/Drone_Swarm_Simulator_v2/scenarios/antena_logic.py
from core.control import DroneController, PIDRegulator
from core.logging.csv_logger import CSV_HEADER, write_metadata, write_row
from core.mavlink.utils import RC_NEUTRAL
```

### 5.2. Вспомогательный код (общий модуль CSV)

Файл: `Drone_Swarm_Simulator_v2/core/logging/csv_logger.py`

Здесь определены:

- константа заголовка `CSV_HEADER`;
- функция `write_row(...)` — формирует одну строку CSV и делает `flush`;
- функция `write_metadata(...)` — пишет `metadata.json`.

### 5.3. Останов записи и закрытие файлов

При завершении сценария (`finally` в `main()`):

- выставляется `STOP_EVENT`, чтобы остановить `exchange_loop`;
- вызывается `_stop_all(controllers)`;
- закрываются файловые дескрипторы `drone_*.csv`.

```585:596:/home/user/Kursov3/Drone_Swarm_Simulator_v2/scenarios/antena_logic.py
    finally:
        STOP_EVENT.set()
        _stop_all(controllers)
        for _did, f in experiment_log_files.items():
            try:
                f.close()
            except Exception:
                pass
        logger.info("[antena_logic] Logs closed. Replay:")
        logger.info("  source /opt/ros/jazzy/setup.bash")
        logger.info("  cd %s", _project_root)
        logger.info("  python3 replay/replay_rviz2.py --experiment %s --rate 1.0 --rviz", experiment_dir)
```

---

## 6. «Общая NED-рамка» (почему это важно для логов и replay)

В SITL у каждого дрона свой home со смещением по востоку. В сценарии позиция приводится к **единой системе координат** функцией `_pos_common()` (смещение по `y` на `(id-1)*2` м). Именно **эти** `x,y,z` попадают в CSV.

Это важно для корректной **относительной** геометрии роя при воспроизведении.

---

## 7. Какой код отвечает за считывание логов

### 7.1. Загрузка и валидация эксперимента

Файл: `Drone_Swarm_Simulator_v2/replay/csv_loader.py`

Функции:

- `load_experiment(experiment_dir)` — проверяет `metadata.json` и наличие `drone_1.csv … drone_N.csv`, валидирует заголовок и число колонок.
- `iter_steps(...)` — выравнивает строки по времени `t` и отдаёт «шаги» для воспроизведения.

Требования к `metadata.json` (обязательные ключи):

```12:13:/home/user/Kursov3/Drone_Swarm_Simulator_v2/replay/csv_loader.py
REQUIRED_METADATA_KEYS = ("duration_sec", "collision_radius_m", "num_drones", "scenario")
CSV_HEADER = "t,x,y,z,rx,ry,rz,hasCollision"
```

---

## 8. Какой код отвечает за визуализацию в ROS2 / RViz2

### 8.1. Нода воспроизведения

Файл: `Drone_Swarm_Simulator_v2/replay/replay_rviz2.py`

Это ROS2-скрипт на `rclpy`, который:

1. Загружает эксперимент через `load_experiment` / `iter_steps`.
2. Публикует позы дронов в RViz2.

Ключевые моменты из шапки файла:

```1:8:/home/user/Kursov3/Drone_Swarm_Simulator_v2/replay/replay_rviz2.py
ROS 2 node: 3D replay of drone swarm experiments in RViz 2.

Loads the same experiment format as replay_rviz.py (metadata.json + drone_*.csv).
Publishes geometry_msgs/PoseStamped on /swarm/drone_<id>/pose (ENU, converted from NED).
Also publishes visualization_msgs/MarkerArray on /swarm/markers — one RViz2 display
```

### 8.2. Преобразование координат для RViz

В логах координаты **NED**. Для RViz используется **ENU**:

```78:80:/home/user/Kursov3/Drone_Swarm_Simulator_v2/replay/replay_rviz2.py
def _ned_to_enu(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """NED → ENU for RViz (x_north→y, y_east→x, z_down→-z)."""
    return (float(y), float(x), float(-z))
```

Ориентация: углы из CSV конвертируются в кватернион (`_quaternion_from_euler`).

---

## 9. Как запускаются ROS2 и RViz2

Типовой порядок (как в README проекта):

1. Активировать окружение ROS2:

```bash
source /opt/ros/jazzy/setup.bash
```

2. Из корня симулятора запустить replay:

```bash
cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
python3 replay/replay_rviz2.py --experiment experiments/exp_<id> --rate 1.0 --rviz
```

Флаг `--rviz` запускает `rviz2` с конфигом по умолчанию (см. следующий раздел).

---

## 10. Какие ROS2-топики используются

Из `replay_rviz2.py` (основные):

| Топик | Тип | Назначение |
|------|-----|------------|
| `/swarm/drone_<id>/pose` | `geometry_msgs/PoseStamped` | Поза дрона `id` (ENU) |
| `/swarm/markers` | `visualization_msgs/MarkerArray` | Сферы для всех дронов одним дисплеем |
| `/swarm/metadata` | `std_msgs/String` | JSON метаданных эксперимента (один раз в начале линейного режима) |

Опционально (интерактивный режим `--interactive`):

- `/replay/play`, `/replay/pause` — `std_msgs/Empty`
- `/replay/seek` — `std_msgs/Float64`
- `/replay/speed` — `std_msgs/Float32`

---

## 11. Как создаётся «мир» визуализации (RViz2)

### 11.1. Fixed Frame и сетка

«Мир» в RViz2 — это прежде всего:

- **Global Options → Fixed Frame** (в конфиге по умолчанию: `world`);
- **дисплей Grid** (плоскость сетки в плоскости XY относительно Fixed Frame);
- **дисплей MarkerArray** на `/swarm/markers`.

Это задаётся файлом конфигурации RViz2:

`Drone_Swarm_Simulator_v2/replay/rviz2_swarm_replay.rviz`

Фрагмент:

```1:54:/home/user/Kursov3/Drone_Swarm_Simulator_v2/replay/rviz2_swarm_replay.rviz
# RViz2: swarm replay (/swarm/markers), Fixed Frame = world
...
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Plane: XY
      Reference Frame: <Fixed Frame>
    - Class: rviz_default_plugins/MarkerArray
      Name: Swarm markers
      Topic:
        Value: /swarm/markers
  Global Options:
    Fixed Frame: world
    Frame Rate: 30
```

То есть «мир» — это **фиксированная инерциальная рамка `world`**, относительно которой рисуются маркеры/позы. Отдельного симулятора физики здесь нет: RViz2 только **отображает** опубликованные сообщения.

### 11.2. Что рисуется как объекты

`replay_rviz2.py` строит `MarkerArray` со **сферами** (`Marker.SPHERE`) для каждого дрона, с палитрой цветов; при `hasCollision=1` цвет становится красным (в `antena_logic` сейчас всегда 0).

---

## 12. Логика визуализации по логам (пошагово)

1. **Загрузка**: `load_experiment` проверяет структуру каталога и корректность CSV.
2. **Итерация по времени**: `iter_steps` выдаёт последовательность «снимков» `(t, [row_drone1, row_drone2, ...])`, синхронизируя дронов по времени.
3. **Публикация**:
   - для каждого дрона публикуется `PoseStamped` на `/swarm/drone_<id>/pose`;
   - дополнительно публикуется сводный `MarkerArray` на `/swarm/markers` (удобно для RViz: один дисплей).
4. **Сглаживание (опционально)**: аргументы `--viz-substeps`, `--viz-cap-hz`, `--viz-spatial-step-m` позволяют интерполировать между строками лога, чтобы движение в RViz выглядело плавнее (см. help в `replay_rviz2.py`).

---

## 13. Связь с «живой» 2D-визуализацией (не RViz2)

В `antena_logic.py` также может работать UDP-паблишер `visualizer/position_publisher.py` для **matplotlib 2D** (это не ROS2). Это отдельный канал визуализации и **не заменяет** ROS2 replay.

---

## 14. Практический чеклист

1. Запустить сценарий с ограничением времени и явным `run-id` / `experiment-dir`.
2. Убедиться, что в каталоге появились `metadata.json` и `drone_1..4.csv`.
3. Запустить:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
python3 replay/replay_rviz2.py --experiment <путь_к_каталогу> --rate 1.0 --rviz
```

Если нужна более плавная анимация — добавить `--viz-substeps` и при необходимости `--viz-cap-hz` (см. комментарии в `replay_rviz2.py`).
