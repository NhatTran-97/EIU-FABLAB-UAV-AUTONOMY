"""Resource paths: source tree (<pkg>/qml, <pkg>/config) or installed package (share/<pkg>/...)."""

from pathlib import Path

PKG = 'ground_station_ui'


def package_root():
    """Package root: the source tree when available (run from source), otherwise share/<pkg> (installed)."""
    src = Path(__file__).resolve().parents[1]
    if (src / 'config').is_dir():
        return src
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(PKG))


def resource_dir(name):
    return package_root() / name


def data_path(p):
    """Resolve a gcs.yaml path: absolute and ~ paths are kept; relative paths are relative to the package root.

    Data (map cache, airspace) lives inside the package, so copying the directory is enough to move it.
    """
    path = Path(str(p)).expanduser()
    return path if path.is_absolute() else package_root() / path
