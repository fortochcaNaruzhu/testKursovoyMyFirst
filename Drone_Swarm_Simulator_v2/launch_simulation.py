#!/usr/bin/env python3
"""
Launcher for Drone Swarm Simulator v2: SITL + optional Webots + scenario.

Usage:
  python launch_simulation.py                    # interactive menu (SITL, scenario, 2D visualizer)
  python launch_simulation.py --help
  python launch_simulation.py -s -c leader_forward_back   # SITL-only, no 2D visualizer
  python launch_simulation.py -s -c leader_forward_back --with-2d-visualizer   # SITL + 2D visualizer
  python launch_simulation.py -s -c linear_chain2         # chain: auto >=4 drones + 2D plot (use --no-2d-visualizer to disable)

Simulation modes:
  --webots, -w      Webots 3D + SITL
  --sitl-only, -s   SITL only (default)

In interactive mode you can choose to start the 2D matplotlib visualizer (UDP port 15551).

Scenarios run from project root so they can import core.
"""

import argparse
import logging
import math
import os
import signal
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

project_root: str = os.path.dirname(os.path.abspath(__file__))
APM_HOME: str = os.path.join(project_root, "../..", "ardupilot")
SIM_VEHICLE_PATH: str = os.path.join(APM_HOME, "Tools", "autotest", "sim_vehicle.py")

# MAVLink UDP `--out=` from sim_vehicle.py (must match start_sitl_* port math).
SITL_BASE_UDP_PORT = 14551
SITL_UDP_PORT_STEP = 10

# Per-drone home: sim_vehicle.py uses -l/--custom-location (lat,lon,alt,heading). We use one base
# and offset East by (drone_index * 2) m so that in a common NED frame drone 0 is at Y=0, 1 at Y=2, etc.
# East offset in degrees = meters_east / (111320 * cos(lat_rad)).
BASE_HOME_LAT = 47.0
BASE_HOME_LON = 8.0
BASE_HOME_ALT = 0.0
METERS_PER_DEGREE_EAST = 111320.0 * math.cos(math.radians(BASE_HOME_LAT))


def _sitl_home_for_drone(drone_index: int) -> str:
    """Build lat,lon,alt,heading string for SITL so local NED Y (East) = drone_index * 2 m in common frame.

    Args:
        drone_index: 0-based drone index (0 -> Y=0, 1 -> Y=2, ...).

    Returns:
        String for sim_vehicle.py -l/--custom-location (e.g. "47.0,8.0,0,0" or "47.0,8.000018,0,0").
    """
    lon_offset_deg = (drone_index * 2.0) / METERS_PER_DEGREE_EAST
    lon = BASE_HOME_LON + lon_offset_deg
    return f"{BASE_HOME_LAT},{lon},{BASE_HOME_ALT},0"

# Scenario id, description, script path relative to project_root, cwd = project_root
SCENARIOS: List[Tuple[str, str, str, str]] = [
    (
        "solo",
        "Solo: heartbeat + takeoff to 1m (solo)",
        "scenarios/solo.py",
        project_root,
    ),
    (
        "antenna",
        "Antenna: 4-drone heartbeat + takeoff to 1m (antenna)",
        "scenarios/antenna.py",
        project_root,
    ),
    (
        "antena_logic",
        "Antena logic: nearest-neighbor vertical antenna (antena_logic)",
        "scenarios/antena_logic.py",
        project_root,
    ),
    (
        "leader_forward_back",
        "Leader forward-back (leader_forward_back)",
        "scenarios/leader_forward_back.py",
        project_root,
    ),
    (
        "square_pid",
        "Square PID (square_formation)",
        "scenarios/square_formation.py",
        project_root,
    ),
    (
        "linear_chain",
        "Линейная цепь (linear_chain)",
        "scenarios/linear_chain.py",
        project_root,
    ),
    (
        "linear_chain2",
        "Линейная цепь v2 — закон Anatoliy (linear_chain2)",
        "scenarios/linear_chain2.py",
        project_root,
    ),
    (
        "linear_chain3",
        "Линейная цепь v3 — коридорный закон в базисе AB (linear_chain3)",
        "scenarios/linear_chain3.py",
        project_root,
    ),
]

# Scenarios that need more than two vehicles (e.g. linear chain: two anchors + internals).
SCENARIO_MIN_DRONES: Dict[str, int] = {
    "linear_chain": 4,
    "linear_chain2": 4,
    "linear_chain3": 4,
    "antenna": 4,
    "antena_logic": 4,
}

# Many parallel MAVProxy + ArduCopter SITL processes need more spacing; otherwise CPU / IO
# spikes when instance 8–10 start can starve earlier links (MAVLink timeouts).
_LARGE_SWARM_DRONE_THRESHOLD = 8
_SITL_HEARTBEAT_WAIT_MAX_WORKERS = 4


def sitl_inter_instance_delay_sec(num_drones: int) -> float:
    """Pause after each ``sim_vehicle.py`` start (except the last) to reduce load spikes."""
    if num_drones <= 1:
        return 0.0
    base = 5.0
    if num_drones <= 4:
        return base
    # Extra time per drone beyond four (capped).
    extra = min(8.0, max(0, num_drones - 4) * 0.85)
    return min(14.0, base + extra)


def _heartbeat_source_sysid(msg: Any) -> Optional[int]:
    if msg is None:
        return None
    gt = getattr(msg, "get_type", None)
    if callable(gt) and gt() == "BAD_DATA":
        return None
    gs = getattr(msg, "get_srcSystem", None)
    if callable(gs):
        try:
            return int(gs())
        except (TypeError, ValueError):
            pass
    for name in ("srcSystem", "sysid"):
        v = getattr(msg, name, None)
        if v is not None:
            try:
                return int(v)
            except (TypeError, ValueError):
                pass
    return None


def _wait_sitl_heartbeat_on_udp(
    udp_port: int,
    expected_sysid: int,
    timeout_sec: float,
) -> None:
    """Block until a HEARTBEAT from ``expected_sysid`` is seen on ``udp_port``."""
    from pymavlink import mavutil

    conn_str = f"udp:127.0.0.1:{udp_port}"
    master = mavutil.mavlink_connection(conn_str)
    try:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            m = master.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=min(remaining, 1.0),
            )
            if m is None:
                continue
            src = _heartbeat_source_sysid(m)
            if src is None:
                continue
            if src != expected_sysid:
                continue
            logger.info(
                "[Launcher] MAVLink ready: sysid=%s on 127.0.0.1:%s",
                expected_sysid,
                udp_port,
            )
            return
        raise TimeoutError(
            f"no HEARTBEAT from sysid {expected_sysid} on {conn_str} within {timeout_sec}s"
        )
    finally:
        try:
            master.close()
        except Exception:
            pass


def wait_all_sitl_heartbeats(
    num_drones: int,
    timeout_sec: float = 120.0,
    *,
    max_workers: Optional[int] = None,
) -> None:
    """Wait for MAVLink HEARTBEAT on each SITL UDP port (same layout as ``start_sitl_only``).

    Uses a bounded thread pool so many drones do not open N simultaneous pymavlink readers
    (reduces UDP / scheduler pressure during the readiness probe).
    """
    if num_drones <= 0:
        return
    try:
        import pymavlink  # noqa: F401
    except ImportError as e:
        logger.error("[Launcher] pymavlink is required to wait for SITL: %s", e)
        raise SystemExit(1) from e

    workers = max_workers if max_workers is not None else max(
        1, min(_SITL_HEARTBEAT_WAIT_MAX_WORKERS, num_drones)
    )
    logger.info(
        "[Launcher] SITL heartbeat wait using up to %d parallel connection(s) (%d drone(s)).",
        workers,
        num_drones,
    )

    def one(i: int) -> None:
        port = SITL_BASE_UDP_PORT + i * SITL_UDP_PORT_STEP
        _wait_sitl_heartbeat_on_udp(port, i + 1, timeout_sec)

    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(one, i) for i in range(num_drones)]
        for fut in as_completed(futures, timeout=timeout_sec + 15.0):
            fut.result()


def start_sitl_only(
    proj_root: str,
    num_drones: int = 2,
    param_file: Optional[str] = None,
    *,
    inter_instance_delay_sec: float = 5.0,
    sitl_console: bool = True,
) -> List[subprocess.Popen]:
    """Start SITL instances without Webots.

    Args:
        proj_root: Project root directory path.
        num_drones: Number of drone instances to start.
        param_file: Optional path to ArduPilot parameter file (relative to proj_root).
        inter_instance_delay_sec: Sleep after each start (except the last) to limit load spikes.
        sitl_console: If True, pass ``--console`` to ``sim_vehicle.py`` (MAVProxy UI per drone).

    Returns:
        List of started subprocess Popen objects, or empty list on error.
    """
    if not os.path.isfile(SIM_VEHICLE_PATH):
        logger.error("Not found %s; ArduPilot must be at ../ardupilot relative to project.", SIM_VEHICLE_PATH)
        return []
    extra_params = []
    if param_file:
        p = os.path.join(proj_root, param_file)
        if os.path.isfile(p):
            extra_params.append(f"--add-param-file={os.path.abspath(p)}")
    processes = []
    cwd = APM_HOME if os.path.isdir(APM_HOME) else proj_root
    for i in range(num_drones):
        udp_port = SITL_BASE_UDP_PORT + i * SITL_UDP_PORT_STEP
        home_str = _sitl_home_for_drone(i)
        args = [
            sys.executable,
            SIM_VEHICLE_PATH,
            "-v", "ArduCopter", "-w",
            f"--instance={i}",
            f"--sysid={i + 1}",
            f"--out=127.0.0.1:{udp_port}",
            "-l", home_str,
        ]
        if sitl_console:
            args.append("--console")
        if extra_params:
            args.extend(extra_params)
        logger.info("[SITL] instance=%s, UDP -> 127.0.0.1:%s, custom_location=%s", i, udp_port, home_str)
        proc = subprocess.Popen(args, cwd=cwd)
        processes.append(proc)
        if i < num_drones - 1 and inter_instance_delay_sec > 0:
            time.sleep(inter_instance_delay_sec)
    return processes


def start_sitl_webots(
    proj_root: str,
    num_drones: int = 2,
    param_file: Optional[str] = None,
    *,
    inter_instance_delay_sec: float = 5.0,
    sitl_console: bool = True,
) -> Tuple[List[subprocess.Popen], None]:
    """Start SITL instances with Webots (webots-python model).

    Args:
        proj_root: Project root directory path.
        num_drones: Number of drone instances to start.
        param_file: Optional path to ArduPilot parameter file (relative to proj_root).
        inter_instance_delay_sec: Sleep after each start (except the last).
        sitl_console: If True, pass ``--console`` to ``sim_vehicle.py``.

    Returns:
        Tuple of (list of SITL Popen processes, None placeholder).
    """
    if not os.path.isfile(SIM_VEHICLE_PATH):
        logger.error("Not found %s", SIM_VEHICLE_PATH)
        return [], None
    BASE_TCP = 5770
    p = os.path.join(proj_root, param_file) if param_file else None
    extra = [f"--add-param-file={os.path.abspath(p)}"] if p and os.path.isfile(p) else []
    processes = []
    cwd = APM_HOME if os.path.isdir(APM_HOME) else proj_root
    for i in range(num_drones):
        tcp_port = BASE_TCP + i * 10
        udp_port = SITL_BASE_UDP_PORT + i * SITL_UDP_PORT_STEP
        home_str = _sitl_home_for_drone(i)
        args = [
            sys.executable,
            SIM_VEHICLE_PATH,
            "-v", "ArduCopter", "-w",
            "--model", "webots-python",
            f"--instance={i}",
            f"--sysid={i + 1}",
            f"--out=127.0.0.1:{tcp_port}",
            f"--out=127.0.0.1:{udp_port}",
            "-l", home_str,
        ]
        if sitl_console:
            args.append("--console")
        if extra:
            args.extend(extra)
        logger.info("[SITL] instance=%s, TCP=%s, UDP=%s, custom_location=%s", i, tcp_port, udp_port, home_str)
        proc = subprocess.Popen(args, cwd=cwd)
        processes.append(proc)
        if i < num_drones - 1 and inter_instance_delay_sec > 0:
            time.sleep(inter_instance_delay_sec)
    return processes, None


def launch_webots(
    proj_root: str, num_drones: int = 2
) -> Optional[subprocess.Popen]:
    """Start Webots world with configured drone count.

    Args:
        proj_root: Project root directory path.
        num_drones: Number of drones to insert into the world.

    Returns:
        Popen instance of Webots process, or None if world file not found or error.
    """
    WORLDS_DIR = os.path.join(proj_root, "worlds")
    INPUT_WORLD = os.path.join(WORLDS_DIR, "irisAuto.wbt")
    OUTPUT_WORLD = os.path.join(WORLDS_DIR, "temp_world.wbt")
    if not os.path.isfile(INPUT_WORLD):
        logger.error("World not found: %s", INPUT_WORLD)
        return None
    with open(INPUT_WORLD, "r") as f:
        content = f.read()
    insert_marker = "# Insert drones"
    pos = content.find(insert_marker)
    if pos == -1:
        logger.error("Marker # Insert drones not found in world")
        return None
    pos += len(insert_marker)
    dx, dy, z = 2.0, 2.0, 0.0549632125
    drones = []
    for i in range(num_drones):
        row, col = i % 10, i // 10
        x, y = col * dx, row * dy
        iris_block = (
            f'\nIris {{\n  translation {x} {y} {z}\n  rotation 0 1 0 0\n'
            f'  name "Iris_{i}"\n  controller "ardupilot_vehicle_controller"\n'
            f'  controllerArgs [ "--instance" "{i}" "--motors" '
            '"m1_motor, m2_motor, m3_motor, m4_motor" ]\n}}\n'
        )
        drones.append(iris_block)
    new_content = content[:pos] + "".join(drones) + content[pos:]
    with open(OUTPUT_WORLD, "w") as f:
        f.write(new_content)
    env = os.environ.copy()
    env["WEBOTS_PROTO_PATH"] = os.path.join(proj_root, "protos")
    logger.info("[Webots] Starting...")
    return subprocess.Popen(["webots", OUTPUT_WORLD], env=env)


def run_interactive_menu() -> Tuple[bool, Tuple[str, str, str, str], int, bool]:
    """Interactive menu for mode, scenario, drone count, and 2D visualizer.

    Returns:
        Tuple of (use_webots, scenario_tuple, num_drones, use_2d_visualizer).
        scenario_tuple is (scenario_id, description, script_path, cwd).
    """
    print("\n=== Simulation launcher ===\n")
    print("1. Mode:")
    print("   1) Webots 3D + SITL")
    print("   2) SITL only (no Webots)")
    mode = input("   Choice [1/2, default 2]: ").strip() or "2"
    use_webots = mode == "1"
    print("\n2. Scenario:")
    for i, (sid, desc, _, _) in enumerate(SCENARIOS, 1):
        print(f"   {i}) {sid}: {desc}")
    idx = input(f"   Number [1-{len(SCENARIOS)}, default 1]: ").strip()
    idx = int(idx) if idx.isdigit() else 1
    scenario = SCENARIOS[idx - 1] if 1 <= idx <= len(SCENARIOS) else SCENARIOS[0]
    print("\n3. Number of drones:")
    d = input("   Count [default 2]: ").strip()
    num_drones = int(d) if d.isdigit() and int(d) >= 1 else 2
    sid = scenario[0]
    min_need = SCENARIO_MIN_DRONES.get(sid, 1)
    if num_drones < min_need:
        print(
            f"   Note: scenario '{sid}' needs at least {min_need} drones "
            f"(2 anchors + internals); using {min_need}."
        )
        num_drones = min_need
    print("\n4. 2D visualization (matplotlib, UDP port 15551):")
    v = input("   Start 2D visualizer? [y/N, default: y]: ").strip().lower() or "y"
    use_2d_visualizer = v in ("y", "yes", "1")
    return use_webots, scenario, num_drones, use_2d_visualizer


def main() -> None:
    """Parse arguments, start SITL (and optionally Webots), then run scenario.

    If scenario file is missing or SITL cannot be started, exits with code 1.
    Registers signal handlers for SIGINT/SIGTERM to terminate child processes.
    """
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    parser = argparse.ArgumentParser(
        description=(
            "Launcher: SITL + optional Webots + scenario; "
            "use --with-2d-visualizer for live 2D plot"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python launch_simulation.py -s -c leader_forward_back
  python launch_simulation.py -s -c square_pid -n 2
  python launch_simulation.py -s -c linear_chain2
  python launch_simulation.py -s -c linear_chain2 -n 6 --no-2d-visualizer
  python launch_simulation.py -s -c linear_chain3
        """,
    )
    g = parser.add_mutually_exclusive_group(required=False)
    g.add_argument("-w", "--webots", action="store_true", help="Webots 3D + SITL")
    g.add_argument("-s", "--sitl-only", action="store_true", help="SITL only (default)")
    parser.add_argument(
        "-c", "--scenario", type=str,
        help="Scenario ID: " + ", ".join(s[0] for s in SCENARIOS),
    )
    parser.add_argument(
        "-n", "--drones", type=int, default=2, metavar="N",
        help="Number of drones (default 2)",
    )
    parser.add_argument(
        "--param-file",
        type=str,
        default="config/iris.parm",
        help=(
            "ArduPilot parameter file (default: config/iris.parm). "
            "Used for SITL in all modes (SITL-only, Webots, interactive). "
            "If file is missing, SITL runs with defaults."
        ),
    )
    parser.add_argument(
        "--duration", type=float, default=0, metavar="T",
        help=(
            "Experiment duration (s); 0 = no limit. "
            "Passed to all scenarios; leader_forward_back has full support."
        ),
    )
    parser.add_argument(
        "--experiment-dir", type=str, default=None,
        help=(
            "Experiment log folder. Passed to all scenarios; "
            "leader_forward_back has full support."
        ),
    )
    parser.add_argument(
        "--run-id",
        type=str,
        default=None,
        help=(
            "Suffix for logs (e.g. logs/two_drones_log_<run-id>_exchange_sync.csv); "
            "passed to leader_forward_back as --run-id."
        ),
    )
    viz_grp = parser.add_mutually_exclusive_group()
    viz_grp.add_argument(
        "--with-2d-visualizer",
        action="store_true",
        help="Start 2D matplotlib visualizer subprocess before the scenario.",
    )
    viz_grp.add_argument(
        "--no-2d-visualizer",
        action="store_true",
        help="Do not start 2D visualizer (disables auto-on for linear_chain / linear_chain2 / linear_chain3).",
    )
    parser.add_argument(
        "--exchange-hz",
        type=float,
        default=50.0,
        help=(
            "Coordinate exchange loop rate (Hz); match SITL position stream. "
            "Passed to scenario (default 50)."
        ),
    )
    parser.add_argument(
        "--sitl-heartbeat-timeout",
        type=float,
        default=120.0,
        metavar="SEC",
        help=(
            "Max seconds to wait per drone for MAVLink HEARTBEAT on each SITL UDP port "
            "(bounded parallel wait). Default 120."
        ),
    )
    sitl_con_grp = parser.add_mutually_exclusive_group()
    sitl_con_grp.add_argument(
        "--no-sitl-console",
        action="store_true",
        help=(
            "Do not pass --console to sim_vehicle.py (no MAVProxy window per drone). "
            "Recommended for large swarms."
        ),
    )
    sitl_con_grp.add_argument(
        "--sitl-console",
        action="store_true",
        help=(
            "Force --console for every SITL. Default: on for small swarms, off for "
            f"{_LARGE_SWARM_DRONE_THRESHOLD}+ drones."
        ),
    )
    parser.add_argument(
        "--sitl-start-stagger",
        type=float,
        default=None,
        metavar="SEC",
        help=(
            "Seconds to wait after each SITL start before launching the next "
            "(default: scales with drone count)."
        ),
    )
    args = parser.parse_args()

    if args.webots or args.sitl_only:
        use_webots = args.webots
        num_drones = args.drones
        scenario = (
            next((s for s in SCENARIOS if s[0] == args.scenario), SCENARIOS[0])
            if args.scenario
            else SCENARIOS[0]
        )
        sid = scenario[0]
        if args.no_2d_visualizer:
            use_2d_visualizer = False
        elif args.with_2d_visualizer:
            use_2d_visualizer = True
        else:
            # Same convenience as interactive menu default for chain scenarios.
            use_2d_visualizer = sid in ("linear_chain", "linear_chain2", "linear_chain3")
    else:
        use_webots, scenario, num_drones, use_2d_visualizer = run_interactive_menu()

    _, scenario_desc, script_rel, scenario_cwd = scenario
    scenario_id = scenario[0]
    min_need = SCENARIO_MIN_DRONES.get(scenario_id, 1)
    if num_drones < min_need:
        logger.info(
            "[Launcher] Scenario '%s' needs at least %d drones; "
            "raising --drones from %d to %d (2 anchors + internals).",
            scenario_id,
            min_need,
            num_drones,
            min_need,
        )
        num_drones = min_need
    script_path = (
        os.path.join(project_root, script_rel)
        if not os.path.isabs(script_rel)
        else script_rel
    )
    if not os.path.isfile(script_path):
        logger.error("Scenario not found: %s", script_path)
        sys.exit(1)

    if args.sitl_console:
        use_sitl_console = True
    elif args.no_sitl_console:
        use_sitl_console = False
    elif num_drones >= _LARGE_SWARM_DRONE_THRESHOLD:
        use_sitl_console = False
        logger.info(
            "[Launcher] %d+ drones: omitting per-SITL MAVProxy --console by default "
            "(use --sitl-console if you need it).",
            _LARGE_SWARM_DRONE_THRESHOLD,
        )
    else:
        use_sitl_console = True

    stagger_sec = (
        float(args.sitl_start_stagger)
        if args.sitl_start_stagger is not None
        else sitl_inter_instance_delay_sec(num_drones)
    )
    logger.info(
        "[Launcher] SITL stagger=%.1fs between instances, MAVProxy console=%s",
        stagger_sec,
        use_sitl_console,
    )

    processes = []
    if use_webots:
        webots_proc = launch_webots(project_root, num_drones)
        if webots_proc:
            processes.append(webots_proc)
            time.sleep(5)
    if use_webots:
        sitl_procs, _ = start_sitl_webots(
            project_root,
            num_drones,
            args.param_file,
            inter_instance_delay_sec=stagger_sec,
            sitl_console=use_sitl_console,
        )
        processes.extend(sitl_procs)
    else:
        sitl_procs = start_sitl_only(
            project_root,
            num_drones,
            args.param_file,
            inter_instance_delay_sec=stagger_sec,
            sitl_console=use_sitl_console,
        )
        processes.extend(sitl_procs)

    if not processes:
        sys.exit(1)

    hb_timeout = float(getattr(args, "sitl_heartbeat_timeout", 120.0))
    logger.info(
        "[Launcher] Waiting for MAVLink HEARTBEAT from each SITL (%.0fs per drone)...",
        hb_timeout,
    )
    try:
        wait_all_sitl_heartbeats(num_drones, timeout_sec=hb_timeout)
    except TimeoutError as e:
        logger.error("[Launcher] SITL MAVLink not ready: %s", e)
        for p in processes:
            try:
                p.terminate()
            except Exception:
                pass
        time.sleep(1)
        for p in processes:
            try:
                p.wait(timeout=2)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
        sys.exit(1)
    except SystemExit:
        raise
    except Exception as e:
        logger.exception("[Launcher] Failed while waiting for SITL: %s", e)
        for p in processes:
            try:
                p.terminate()
            except Exception:
                pass
        sys.exit(1)

    settle_sec = min(8.0, 2.0 + 0.25 * max(0, num_drones - 4))
    if settle_sec > 0:
        logger.info("[Launcher] Post-SITL settle %.1fs before scenario...", settle_sec)
        time.sleep(settle_sec)

    if use_2d_visualizer:
        visualizer_script = os.path.join(project_root, "visualizer", "drone_position_visualizer.py")
        if os.path.isfile(visualizer_script):
            visualizer_proc = subprocess.Popen(
                [sys.executable, "visualizer/drone_position_visualizer.py"],
                cwd=project_root,
                stdout=sys.stdout,
                stderr=sys.stderr,
            )
            processes.append(visualizer_proc)
            logger.info("[Launcher] Started 2D visualizer subprocess.")
        else:
            logger.warning("[Launcher] 2D visualizer script not found: %s", visualizer_script)

    logger.info("[Scenario] Starting: %s", scenario_desc)
    scenario_cmd = [sys.executable, script_rel, "--drones", str(num_drones)]
    if getattr(args, "duration", 0) > 0:
        scenario_cmd.extend(["--duration", str(args.duration)])
    if getattr(args, "experiment_dir", None):
        scenario_cmd.extend(["--experiment-dir", args.experiment_dir])
    if getattr(args, "run_id", None):
        scenario_cmd.extend(["--run-id", str(args.run_id)])
    exchange_hz = getattr(args, "exchange_hz", 50.0)
    if exchange_hz > 0:
        scenario_cmd.extend(["--exchange-hz", str(exchange_hz)])
    scenario_proc = subprocess.Popen(
        scenario_cmd,
        cwd=scenario_cwd,
        stdout=sys.stdout,
        stderr=sys.stderr,
    )
    processes.append(scenario_proc)

    def shutdown(signum: Optional[int] = None, frame: Optional[object] = None) -> None:
        logger.info("[Launcher] Stopping processes...")
        for p in processes:
            try:
                p.terminate()
            except Exception:
                pass
        time.sleep(1)
        for p in processes:
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    try:
        scenario_proc.wait()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()


if __name__ == "__main__":
    main()
