"""Giai ma che do bay PX4 tu HEARTBEAT.custom_mode (px4_custom_mode.h).

custom_mode = (sub_mode << 24) | (main_mode << 16)
"""

MAIN_MODES = {
    1: 'Manual', 2: 'Altitude', 3: 'Position', 5: 'Acro', 6: 'Offboard', 7: 'Stabilized',
}
AUTO_SUB_MODES = {
    1: 'Ready', 2: 'Takeoff', 3: 'Hold', 4: 'Mission', 5: 'Return', 6: 'Land',
    8: 'Follow Me', 9: 'Precision Land', 10: 'VTOL Takeoff',
}
POSCTL_SUB_MODES = {0: 'Position', 1: 'Orbit', 2: 'Position Slow'}

MAIN_AUTO = 4
MAIN_POSCTL = 3


def decode(custom_mode):
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    if main == MAIN_AUTO:
        return AUTO_SUB_MODES.get(sub, f'Auto ({sub})')
    if main == MAIN_POSCTL:
        return POSCTL_SUB_MODES.get(sub, 'Position')
    return MAIN_MODES.get(main, f'Unknown ({main})')
