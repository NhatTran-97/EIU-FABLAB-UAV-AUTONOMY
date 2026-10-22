"""System problem summary (problem-first), shared by the top-bar Alert Center, System Health on the
Diagnostics tab and the pre-flight check before Take-off.

evaluate(...) -> [{'level': 'critical'|'warning', 'source', 'text'}], critical first.
Critical = do not take off; warning = needs attention.
"""

BATTERY_WARN = 25
BATTERY_CRIT = 15
RADIO_BUF_WARN = 30


def evaluate(vehicle, links, env_effective='indoor', env_mismatch='', px4_problems=()):
    g = vehicle.groups
    st, gps, bat = g['status'].values, g['gps'].values, g['battery'].values
    comp = g['companion'].values
    out = []

    def add(level, source, text):
        out.append({'level': level, 'source': source, 'text': text})

    px4_link = links.get('px4', {})
    if not st.get('linkAlive'):
        if px4_link.get('isOpen') or st.get('heartbeatAge', -1) >= 0:
            add('critical', 'PX4', 'PX4 link lost (radio 915)')
        else:
            add('critical', 'PX4', 'PX4 not connected — see Connections')
    else:
        for s in g['sensors'].items:
            if s.get('status') == 'error':
                add('critical', 'PX4', f"{s['name']} sensor error")
        pct = bat.get('percent', -1)
        if pct is not None and 0 <= pct <= BATTERY_CRIT:
            add('critical', 'Battery', f'Battery critical: {pct}%')
        elif pct is not None and 0 <= pct <= BATTERY_WARN:
            add('warning', 'Battery', f'Battery low: {pct}%')
        if env_effective == 'outdoor' and (gps.get('fixType') or 0) < 3:
            add('critical' if st.get('armed') else 'warning', 'GPS', 'Outdoor but no GPS 3D fix')
        buf = px4_link.get('txbuf')
        if buf is not None and buf < RADIO_BUF_WARN:
            add('warning', 'Radio 915', f'Radio 915 congested (buffer {buf}%)')

    if env_mismatch:
        add('warning', 'Config', env_mismatch)
    for text in px4_problems:                    # RTL / geofence / landing parameters (param_checks.py)
        add('warning', 'PX4 params', text)

    jet_link = links.get(getattr(vehicle, 'companion_link', 'jetson'), {})
    if not comp.get('linkAlive'):
        if jet_link.get('isOpen') or comp.get('heartbeatAge', -1) >= 0:
            add('warning', 'Jetson', 'Jetson link lost (radio 433)')
    else:
        for c in g['companion'].items:
            lv = c.get('level')
            if lv == 'error':
                add('critical' if c.get('id') == 4 else 'warning', c['name'], f"{c['name']}: {c.get('state', 'error')}")
            elif lv in ('warn', 'stale'):
                state = 'no data' if lv == 'stale' else c.get('state', '')
                add('warning', c['name'], f"{c['name']}: {state}")
        buf = jet_link.get('txbuf')
        if buf is not None and buf < RADIO_BUF_WARN:
            add('warning', 'Radio 433', f'Radio 433 congested (buffer {buf}%)')

    out.sort(key=lambda a: a['level'] != 'critical')
    return out
