"""Duong dan tai nguyen: source tree (<pkg>/qml, <pkg>/config) hoac ban da cai (share/<pkg>/...)."""

from pathlib import Path

PKG = 'ground_station_ui'


def package_root():
    """Thu muc goc package: source tree neu co (chay thang), khong thi share/<pkg> (da cai)."""
    src = Path(__file__).resolve().parents[1]
    if (src / 'config').is_dir():
        return src
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(PKG))


def resource_dir(name):
    return package_root() / name


def data_path(p):
    """Duong dan trong gcs.yaml: tuyet doi / ~ giu nguyen; tuong doi -> tinh tu goc package.

    Du lieu (cache ban do, vung bay) nam trong package -> chep ca thu muc sang may khac la chay.
    """
    path = Path(str(p)).expanduser()
    return path if path.is_absolute() else package_root() / path
