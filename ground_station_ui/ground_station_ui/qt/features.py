"""Feature list (left tabs). Adding a feature = one entry here + one QML page; the menu and the
StackLayout are generated, Main.qml is not edited.
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
