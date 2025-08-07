import os
import subprocess
import sys
import platform
import re
import json

def detect_python_architecture():
    arch = platform.architecture()[0]
    print(f"[INFO] Python architecture detected: {arch}")
    return arch

def create_venv(python_exe, venv_path=".venv"):
    if not os.path.exists(venv_path):
        print(f"[INFO] Creating virtual environment at {venv_path} using {python_exe}")
        subprocess.check_call([python_exe, "-m", "venv", venv_path])
    else:
        print(f"[INFO] Virtual environment already exists at {venv_path}")

def activate_venv(venv_path=".venv"):
    activate_script = os.path.join(venv_path, "Scripts", "activate.bat")
    if os.path.exists(activate_script):
        print(f"[INFO] To activate the virtual environment, run:\n{activate_script}")
    else:
        print("[ERROR] Activation script not found.")

def install_requirements(venv_path=".venv", requirements_file="requirements.txt"):
    pip_exe = os.path.join(venv_path, "Scripts", "pip.exe")
    if os.path.exists(requirements_file):
        print(f"[INFO] Installing packages from {requirements_file} with --prefer-binary")
        subprocess.check_call([
            pip_exe,
            "install",
            "--prefer-binary",
            "-r",
            requirements_file
        ])
    else:
        print(f"[WARNING] {requirements_file} not found. No packages installed.")



def main():
    python_exe = sys.executable
    detect_python_architecture()
    create_venv(python_exe)
    activate_venv()
    install_requirements()
if __name__ == "__main__":
    main()
