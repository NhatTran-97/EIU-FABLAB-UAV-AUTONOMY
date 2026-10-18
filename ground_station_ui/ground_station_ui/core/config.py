"""Kiem tra config/gcs.yaml luc khoi dong: bao loi cu the (file + muc) thay vi KeyError giua chung
hoac am tham bo qua. validate(cfg) -> (errors, warnings); co error -> app khong chay."""

LINK_TYPES = ('serial', 'udp')


def validate(cfg):
    errors, warnings = [], []
    if not isinstance(cfg, dict):
        return ['gcs.yaml: noi dung khong phai mapping'], []

    links = cfg.get('links') or []
    names = []
    for i, ln in enumerate(links):
        where = f'links[{i}]'
        if not isinstance(ln, dict) or not ln.get('name'):
            errors.append(f'{where}: thieu "name"')
            continue
        name = ln['name']
        where = f'links[{i}] ({name})'
        if name in names:
            errors.append(f'{where}: trung ten link')
        names.append(name)
        t = ln.get('type', 'serial')
        if t not in LINK_TYPES:
            errors.append(f'{where}: type "{t}" khong hop le (chi {", ".join(LINK_TYPES)})')
        if t == 'udp':
            listen = str(ln.get('listen', ''))
            host, _, port = listen.rpartition(':')
            if not host or not port.isdigit():
                errors.append(f'{where}: "listen" phai dang host:port (dang co "{listen}")')
        if t == 'serial' and 'baud' in ln and not str(ln['baud']).isdigit():
            errors.append(f'{where}: baud "{ln["baud"]}" khong phai so')
    if not names:
        warnings.append('links: khong co link nao -> app khong nhan duoc du lieu')

    veh = cfg.get('vehicle') or {}
    for ln in veh.get('autopilot_links') or []:
        if ln not in names:
            errors.append(f'vehicle.autopilot_links: link "{ln}" khong co trong links')
    comp = veh.get('companion_link', 'jetson')
    if names and comp not in names:
        warnings.append(f'vehicle.companion_link: link "{comp}" khong co trong links -> khong nhan du lieu Jetson')
    cmd_link = (cfg.get('commands') or {}).get('link', 'jetson')
    if names and cmd_link not in names:
        errors.append(f'commands.link: link "{cmd_link}" khong co trong links')

    sd = (cfg.get('map') or {}).get('search_default')
    if sd is not None and not (isinstance(sd, (list, tuple)) and len(sd) == 2
                               and all(isinstance(v, (int, float)) for v in sd)
                               and -90 <= sd[0] <= 90 and -180 <= sd[1] <= 180):
        errors.append(f'map.search_default: phai dang [lat, lon] (dang co {sd!r})')

    layers = (cfg.get('map') or {}).get('layers') or []
    ids = []
    for i, ly in enumerate(layers):
        if not isinstance(ly, dict) or not ly.get('id'):
            errors.append(f'map.layers[{i}]: thieu "id"')
            continue
        if ly['id'] in ids:
            errors.append(f'map.layers[{i}]: trung id "{ly["id"]}"')
        ids.append(ly['id'])
    for ly in layers:
        if isinstance(ly, dict):
            for src in ly.get('compose') or []:
                if src not in ids:
                    errors.append(f'map.layers ({ly.get("id")}): compose tham chieu lop "{src}" khong ton tai')
            if not ly.get('compose') and not ly.get('url') and not ly.get('file'):
                warnings.append(f'map.layers ({ly.get("id")}): khong co url / file / compose')

    rate = cfg.get('ui_rate_hz', 5)
    if not isinstance(rate, (int, float)) or rate <= 0:
        errors.append(f'ui_rate_hz: phai > 0 (dang co {rate!r})')
    return errors, warnings
