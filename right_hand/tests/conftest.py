import sys
from pathlib import Path

# The project is a flat folder of scripts; make them importable from tests/.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
