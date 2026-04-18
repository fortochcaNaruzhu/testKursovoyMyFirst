import os
import subprocess
import sys


VENV_DIR_NAME = "drone_env"


def main() -> None:
    """Create and configure Python environment for Drone_Swarm_Simulator_v2."""
    project_root = os.path.dirname(os.path.abspath(__file__))
    venv_path = os.path.join(project_root, "..", VENV_DIR_NAME)

    if os.name == "posix":
        python_version = f"{sys.version_info.major}.{sys.version_info.minor}"
        apt_packages = [
            "git",  # version control, required for MAVProxy clone
            "build-essential",
            f"python{python_version}-dev",
            f"python{python_version}-venv",
            "python3-tk",  # tkinter for matplotlib
            "libgtk-3-dev",  # GTK3 for wxPython
            "libglib2.0-dev",
            "libsdl2-dev",  # SDL2 for wxPython
            "libjpeg-dev",
            "libpng-dev",
            "libtiff-dev",
            "libnotify-dev",
            "freeglut3-dev",
            "libgstreamer1.0-dev",
            "libgstreamer-plugins-base1.0-dev",
            "libwebkit2gtk-4.1-dev",
        ]
        print("Устанавливаем системные зависимости через apt...")
        try:
            subprocess.check_call(
                ["sudo", "apt", "install", "-y"] + apt_packages
            )
        except subprocess.CalledProcessError:
            print("[WARNING] Не удалось установить некоторые apt-пакеты. Продолжаем...")

    print(f"Создаём виртуальное окружение '{venv_path}'...")
    subprocess.check_call([sys.executable, "-m", "venv", venv_path])

    if os.name == "posix":
        activate_script = os.path.join(venv_path, "bin", "activate")
        pip_executable = os.path.join(venv_path, "bin", "pip3")
        python_executable = os.path.join(venv_path, "bin", "python3")
        path_export = 'export PATH="$PATH:$HOME/.local/bin"'
    else:
        activate_script = os.path.join(venv_path, "Scripts", "activate")
        pip_executable = os.path.join(venv_path, "Scripts", "pip")
        python_executable = os.path.join(venv_path, "Scripts", "python")
        path_export = 'set PATH="%PATH%;%USERPROFILE%\\.local\\bin"'
        print("[!] Этот скрипт частично адаптирован для Windows, но рекомендуется использовать Linux.")

    def run_in_venv(command: str) -> None:
        if os.name == "posix":
            full_command = f"source {activate_script} && {command}"
            subprocess.check_call(full_command, shell=True, executable="/bin/bash")
        else:
            full_command = f"call {activate_script} && {command}"
            subprocess.check_call(full_command, shell=True)

    print("Устанавливаем основные библиотеки через pip...")
    packages = [
        "pymavlink",
        "pexpect",
        "empy==3.3.4",
        "dronecan",
        "setuptools",
        "PyYAML",
        "numpy",
        "matplotlib",
        "tqdm",
        "pyserial",
        "future",
        "lxml",
        "wxPython",  # console module
        "opencv-python",  # map module
    ]
    run_in_venv(f"{pip_executable} install {' '.join(packages)}")

    print("Клонируем MAVProxy из GitHub...")
    mavproxy_dir = os.path.join(project_root, "MAVProxy")
    if not os.path.exists(mavproxy_dir):
        subprocess.check_call(["git", "clone", "https://github.com/ArduPilot/MAVProxy.git", mavproxy_dir])
    else:
        print("Папка MAVProxy уже существует, пропускаем клонирование.")

    print("Устанавливаем MAVProxy...")
    subprocess.check_call([python_executable, "-m", "pip", "install", "-e", mavproxy_dir])

    bashrc_path = os.path.expanduser("~/.bashrc")
    if os.name == "posix" and os.path.exists(bashrc_path):
        with open(bashrc_path, "r", encoding="utf-8") as bashrc_file:
            bashrc_content = bashrc_file.read()
        if path_export not in bashrc_content:
            print("Добавляем путь ~/.local/bin в PATH...")
            with open(bashrc_path, "a", encoding="utf-8") as bashrc_append:
                bashrc_append.write(f"\n{path_export}\n")
    else:
        print("[ERROR] не удалось обновить bashrc (Windows или файл отсутствует).")

    print("\nНастройка завершена!")
    if os.name == "posix":
        print(f"Чтобы начать работать, активируйте окружение:\nsource {activate_script}")
    else:
        print(f"Чтобы начать работать, активируйте окружение командой:\n{activate_script}")


if __name__ == "__main__":
    main()
