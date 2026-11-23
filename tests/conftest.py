import sys
from pathlib import Path

# Allow tests to import the synthetic.py helper module
sys.path.insert(0, str(Path(__file__).resolve().parent))
