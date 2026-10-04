"""Danh sach tinh nang (tab ben trai). Them tinh nang moi = them 1 dong + 1 trang QML,
menu va StackLayout tu sinh -- khong sua Main.qml.
"""

FEATURES = [
    {'id': 'map', 'title': 'Map', 'page': 'pages/MapPage.qml', 'enabled': True},
    {'id': 'view3d', 'title': '3D', 'page': 'pages/View3DPage.qml', 'enabled': True},
    {'id': 'telemetry', 'title': 'Telemetry', 'page': 'pages/TelemetryPage.qml', 'enabled': True},
    {'id': 'mission', 'title': 'Mission Plan', 'page': 'pages/PlaceholderPage.qml', 'enabled': False},
    {'id': 'diagnostics', 'title': 'Diagnostics', 'page': 'pages/DiagnosticsPage.qml', 'enabled': True},
    {'id': 'camera', 'title': 'Camera / Video', 'page': 'pages/PlaceholderPage.qml', 'enabled': False},
    {'id': 'links', 'title': 'Connections', 'page': 'pages/LinksPage.qml', 'enabled': True},
    {'id': 'settings', 'title': 'Settings', 'page': 'pages/SettingsPage.qml', 'enabled': True},
]
