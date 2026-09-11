import subprocess
import sys
from pathlib import Path

def install(package: str) -> None:
    subprocess.check_call([sys.executable, "-m", "pip", "install", package])

requirements_path = Path(__file__).with_name("GUI_requirements.txt")

with requirements_path.open("r", encoding="utf-8") as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        install(line)
