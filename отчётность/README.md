# Drone Swarm Simulator v2

Симулятор роя однородных дронов для исследований алгоритмов управления (ArduPilot SITL, MAVLink, Python 3.10+). Результаты экспериментов пишутся в CSV и могут воспроизводиться в ROS/RViz.

## Структура проекта

```
Drone_Swarm_Simulator_v2/
├── launch_simulation.py   # Единая точка входа (одиночный запуск)
├── run_batch.py           # Пакетный запуск серии экспериментов
├── config/                # Параметры ArduPilot (iris.parm и др.)
├── core/                  # Ядро: MAVLink (поза по SIM_STATE в SITL), мониторы, PID, логирование
├── scenarios/             # Сценарии (leader_forward_back, square_pid, ...)
├── replay/                # Воспроизведение логов в ROS/RViz
├── visualizer/            # 2D визуализация (matplotlib): онлайн и replay по CSV
├── experiments/           # Результаты запусков (не коммитить)
└── docs/                  # Документация исследований (локально)
```

## Установка окружения (setup_env.py)

Рекомендуемая установка выполняется на **Linux (Ubuntu)** с правами `sudo`. На Windows скрипт может частично отработать, но окружение для дипломной работы предполагается на Linux.

1. **Клонируйте репозиторий** и перейдите в каталог `Drone_Swarm_Simulator_v2`:

   ```bash
   git clone <url-репозитория>
   cd Drone_Swarm_Simulator_v2
   ```

2. **Запустите скрипт настройки окружения**:

   ```bash
   python3 setup_env.py
   ```

   Скрипт делает следующее:

   - устанавливает системные зависимости через `apt` (только на Ubuntu):
     - `git`, `build-essential`
     - `python<версия>-dev`, `python<версия>-venv`
     - `python3-tk` (tkinter для matplotlib)
     - библиотеки GTK/SDL/картинки/видео для сборки `wxPython`:
       `libgtk-3-dev`, `libglib2.0-dev`, `libsdl2-dev`, `libjpeg-dev`, `libpng-dev`,
       `libtiff-dev`, `libnotify-dev`, `freeglut3-dev`,
       `libgstreamer1.0-dev`, `libgstreamer-plugins-base1.0-dev`,
       `libwebkit2gtk-4.1-dev`
   - создаёт виртуальное окружение `../drone_env` (относительно корня проекта)
   - активирует его и устанавливает Python-зависимости через `pip`:
     - `pymavlink`, `pexpect`, `empy==3.3.4`, `dronecan`, `setuptools`, `PyYAML`
     - `numpy`, `matplotlib`, `pyserial`, `future`, `lxml`
     - `wxPython` (для MAVProxy console) — **сборка может занимать 10–20 минут**
     - `opencv-python` (для модуля map в MAVProxy)
   - клонирует `MAVProxy` из GitHub в подкаталог `MAVProxy` рядом с проектом и устанавливает его в режиме editable (`pip install -e`).
   - добавляет `~/.local/bin` в `PATH` в `~/.bashrc` (на Ubuntu), чтобы бинарники `pip` и `MAVProxy` были доступны из командной строки.

3. **Особенности и рекомендации**:

   - На свежих версиях Ubuntu (например, **Ubuntu 25.10**) пакет `wxPython` может отсутствовать в бинарном виде, поэтому `pip` может собирать его **из исходников** — это нормально, просто требует времени и установленного набора dev-библиотек (см. список выше).
   - Виртуальное окружение `../drone_env` должно быть создано **на той системе, где вы запускаете симуляции**. Не переносите его целиком с Windows на Linux — создайте новое окружение на Linux с помощью `setup_env.py`.
   - На Windows часть шагов (apt, обновление `~/.bashrc`) будет пропущена или ограничена, поэтому рекомендованный и поддерживаемый сценарий — запуск `setup_env.py` на Ubuntu и дальнейшая работа в этом окружении.


## Быстрый старт

```bash
# Окружение
source ../drone_env/bin/activate

# Одиночный запуск (из корня проекта)
python launch_simulation.py -s -c leader_forward_back -n 3 --duration 60

# Пакетный запуск нескольких экспериментов
python run_batch.py --runs 3 --drones 2 --duration 60 --scenario leader_forward_back
```

**Опциональная 2D визуализация в реальном времени:** флаг `--with-2d-visualizer` при запуске лаунчера; подробности — в **visualizer/README.md**.

Подробнее о пакетном запуске, формате конфигурации и расположении результатов см. раздел ниже и **документацию в `docs/experiments/`**.

## Пакетные эксперименты (run_batch.py)

- **Конфигурация:** только через CLI (отдельного конфиг-файла нет).
- **Результаты без `--batch-id`:** `experiments/exp_1`, `experiments/exp_2`, ...
- **Результаты с `--batch-id`:** `experiments/batch_<id>_run_1`, `experiments/batch_<id>_run_2`, ...  
  Дополнительно создаётся индекс: `experiments/batch_<id>_runs_index.json`.
- В каждой директории запуска: `metadata.json` и `drone_*.csv`.
- Воспроизведение: любой такой каталог можно передать в `replay/replay_rviz.py --experiment <путь>` (см. `replay/README.md`).

Полное описание: **docs/experiments/batch_runs.md** (на русском).

## Воспроизведение в RViz

```bash
python replay/replay_rviz.py --experiment experiments/exp_1 --rate 1.0
```

Формат CSV, metadata и топики ROS — в **replay/README.md**.

## Переменные окружения и зависимости

- Python 3.10+, pymavlink, ArduPilot SITL (клон в `../ardupilot` по желанию).
- Для replay: ROS (например Noetic), `rospy`.
- Виртуальное окружение: `../drone_env` (относительно проекта).

## Лицензия и дипломная работа

Документация в `docs/` предназначена для дипломной работы и хранится локально (не публикуется в репозитории).




запуск визуализации ROS2 
запускать терминал из папки Drone_Swarm_Simulator_v2

source /opt/ros/jazzy/setup.bash

python3 scripts/follower_pair_csv_to_experiment.py \
  logs/two_drones_log_run_exchange_sync.csv \
  -o logs/replay_from_run

python3 replay/replay_rviz2.py \
  --experiment logs/replay_from_run \
  --rate 1.0 \
  --rviz \
  --viz-substeps 8 \
  --viz-cap-hz 0




cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
source ../drone_env/bin/activate
python launch_simulation.py -s -c linear_chain2

Будет 4+ дрона (если не указать -n 6 и т.д.) и окно matplotlib. Для 6 дронов:
python launch_simulation.py -s -c linear_chain2 -n 6

Вариант с чистым коридорным законом в базисе AB: `python launch_simulation.py -s -c linear_chain3` (те же параметры `-n`, `--no-2d-visualizer`).



cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
source ../drone_env/bin/activate
python launch_simulation.py -s -c antenna


cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
source ../drone_env/bin/activate
python launch_simulation.py -s -c antena_logic --duration 60 --run-id test3



cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
source /opt/ros/jazzy/setup.bash
python3 replay/replay_rviz2.py --experiment experiments/exp_test3 --rate 1.0 --rviz --viz-substeps 8 --viz-cap-hz 0

source /opt/ros/jazzy/setup.bash
cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
python3 replay/replay_rviz2.py --experiment experiments/2026-04-19_17-16-19 --rviz --interactive --timeline-ui


запуск в test_logov / chek_mode
Console 1

cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
source ../drone_env/bin/activate

python launch_simulation.py -s -c antena_logic -n 1 --duration 60 --no-sitl-console \
  --experiment-dir /home/user/Kursov3/test_logov/chek_mode

  Console 2 (запускать после коннекта с дроном)
  cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
source ../drone_env/bin/activate

OUTDIR=/home/user/Kursov3/test_logov/chek_mode/mavlink_raw_full
mkdir -p "$OUTDIR"

python3 /home/user/Kursov3/test_logov/mavlink_dump_all_txt.py \
  --port 14751 --duration 60 \
  --out "$OUTDIR/mavlink_all_port14751.jsonl"

Зпауск визуализатора (Добавлено небо и пол + таймлайн. всё выведено по умолчанию)
source /opt/ros/jazzy/setup.bash
cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
python3 replay/replay_rviz2.py --experiment /home/user/Kursov3/test_logov/antena_logic_copy_run2 --rviz





#######Запуск сценария "Антенна" рабочий####### 
cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
source ../drone_env/bin/activate

OUTDIR=/home/user/Kursov3/test_logov/fntena_logic_copy2_run
mkdir -p "$OUTDIR"

python launch_simulation.py -s -c fntena_logic_copy2 -n 4 \
  --no-sitl-console --no-2d-visualizer \
  --duration 60 \
  --experiment-dir "$OUTDIR" \
  --log-mode mavlink


#######Запуск визуализации "Антенна" рабочий####### 
source /opt/ros/jazzy/setup.bash
cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
python3 replay/replay_rviz2.py --experiment /home/user/Kursov3/test_logov/fntena_logic_copy2_run --rviz





#######Запуск построения графиков####### 
cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
source ../drone_env/bin/activate

python3 scripts/graphics.py \
  --experiment /home/user/Kursov3/test_logov/antena_logic_copy_run2 \
  --focus-drone 2


  cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
source ../drone_env/bin/activate

python3 scripts/graphics.py \
  --experiment /home/user/Kursov3/test_logov/antena_shum_run \
  --focus-drone 2




  cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
source ../drone_env/bin/activate

OUTDIR=/home/user/Kursov3/test_logov/fntena_logic_copy2_shum3_run
mkdir -p "$OUTDIR"

python launch_simulation.py -s -c fntena_logic_copy2_shum -n 4 \
  --no-sitl-console --no-2d-visualizer \
  --duration 60 \
  --experiment-dir "$OUTDIR" \
  --log-mode mavlink



  cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
source ../drone_env/bin/activate

OUTDIR=/home/user/Kursov3/test_logov/antena_NOshum_run
mkdir -p "$OUTDIR"

  python launch_simulation.py -s -c fntena_logic_copy2_shum -n 4 \
  --no-sitl-console --no-2d-visualizer \
  --duration 60 \
  --experiment-dir "$OUTDIR" \
  --log-mode mavlink \
  --decision-position-source local



  source /opt/ros/jazzy/setup.bash
cd /home/user/Kursov3/Drone_Swarm_Simulator_v2
python3 replay/replay_rviz2.py --experiment /home/user/Kursov3/test_logov/antena_shum_run --rviz