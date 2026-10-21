"""Validate config/gcs.yaml at startup. validate(cfg) -> (errors, warnings); any error aborts startup."""

LINK_TYPES = ('serial', 'udp')


def validate(cfg):
    errors, warnings = [], []
    if not isinstance(cfg, dict):
        return ['gcs.yaml: content is not a mapping'], []

    links = cfg.get('links') or []
    names = []
    for i, ln in enumerate(links):
        where = f'links[{i}]'
        if not isinstance(ln, dict) or not ln.get('name'):
            errors.append(f'{where}: missing "name"')
            continue
        name = ln['name']
        where = f'links[{i}] ({name})'
        if name in names:
            errors.append(f'{where}: duplicate link name')
        names.append(name)
        t = ln.get('type', 'serial')
        if t not in LINK_TYPES:
            errors.append(f'{where}: invalid type "{t}" (allowed: {", ".join(LINK_TYPES)})')
        if t == 'udp':
            listen = str(ln.get('listen', ''))
            host, _, port = listen.rpartition(':')
            if not host or not port.isdigit():
                errors.append(f'{where}: "listen" must be host:port (got "{listen}")')
            remote = ln.get('remote')
            if remote is not None and not str(remote).rpartition(':')[2].isdigit():
                errors.append(f'{where}: "remote" must be host:port (got "{remote}")')
        if t == 'serial' and 'baud' in ln and not str(ln['baud']).isdigit():
            errors.append(f'{where}: baud "{ln["baud"]}" is not a number')
    if not names:
        warnings.append('links: no links configured -> the app will receive no data')

    veh = cfg.get('vehicle') or {}
    for ln in veh.get('autopilot_links') or []:
        if ln not in names:
            errors.append(f'vehicle.autopilot_links: link "{ln}" is not in links')
    comps = veh.get('companion_links') or [veh.get('companion_link', 'jetson')]
    for comp in comps:
        if names and comp not in names:
            warnings.append(f'vehicle.companion_link(s): link "{comp}" is not in links -> no Jetson data')
    for ln in veh.get('autopilot_links') or []:
        if ln in comps:
            errors.append(f'vehicle: link "{ln}" is in both autopilot_links and companion_links')
    cmd_link = (cfg.get('commands') or {}).get('link', 'jetson')
    if names and cmd_link not in names:
        errors.append(f'commands.link: link "{cmd_link}" is not in links')

    sd = (cfg.get('map') or {}).get('search_default')
    if sd is not None and not (isinstance(sd, (list, tuple)) and len(sd) == 2
                               and all(isinstance(v, (int, float)) for v in sd)
                               and -90 <= sd[0] <= 90 and -180 <= sd[1] <= 180):
        errors.append(f'map.search_default: must be [lat, lon] (got {sd!r})')

    layers = (cfg.get('map') or {}).get('layers') or []
    ids = []
    for i, ly in enumerate(layers):
        if not isinstance(ly, dict) or not ly.get('id'):
            errors.append(f'map.layers[{i}]: missing "id"')
            continue
        if ly['id'] in ids:
            errors.append(f'map.layers[{i}]: duplicate id "{ly["id"]}"')
        ids.append(ly['id'])
    for ly in layers:
        if isinstance(ly, dict):
            for src in ly.get('compose') or []:
                if src not in ids:
                    errors.append(f'map.layers ({ly.get("id")}): compose references unknown layer "{src}"')
            if not ly.get('compose') and not ly.get('url') and not ly.get('file'):
                warnings.append(f'map.layers ({ly.get("id")}): no url / file / compose')

    rate = cfg.get('ui_rate_hz', 5)
    if not isinstance(rate, (int, float)) or rate <= 0:
        errors.append(f'ui_rate_hz: must be > 0 (got {rate!r})')
    return errors, warnings
