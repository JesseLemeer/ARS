import sys
from pathlib import Path


def ensure_project_root_on_path(file_path):
    project_root = Path(file_path).resolve().parent.parent
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))
