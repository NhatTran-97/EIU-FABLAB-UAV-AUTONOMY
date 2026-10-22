"""PX4 safety parameter checks: RTL, geofence, landing, speed (core/vehicle/param_checks.py)."""

from ground_station_ui.core.vehicle.param_checks import check, load_params_file

GOOD = {'RTL_RETURN_ALT': 20.0, 'RTL_DESCEND_ALT': 10.0, 'RTL_LAND_DELAY': 0.0, 'RTL_CONE_ANG': 45,
        'GF_MAX_VER_DIST': 40.0, 'GF_MAX_HOR_DIST': 200.0, 'GF_ACTION': 2, 'COM_DISARM_LAND': 2.0,
        'MPC_XY_VEL_MAX': 12.0, 'MPC_XY_P': 0.95}


def levels(rows):
    return {r['name']: r['level'] for r in rows}


def test_consistent_parameters_pass_and_unread_ones_are_unknown():
    rows, problems = check(GOOD)
    assert problems == [] and set(levels(rows).values()) == {'ok'}
    rows, _ = check({})
    assert set(levels(rows).values()) == {'unknown'}


def test_rtl_geofence_landing_and_mission_conflicts():
    v = dict(GOOD, RTL_RETURN_ALT=20.0, GF_MAX_VER_DIST=20.0, RTL_LAND_DELAY=-1.0, COM_DISARM_LAND=0.0,
             GF_MAX_HOR_DIST=30.0, MPC_XY_VEL_MAX=4.0)
    home = (11.0, 106.0)
    mission = [{'lat': 11.0 + 50 / 111195.0, 'lon': 106.0, 'alt': 25.0}]          # 50 m north, 25 m high
    rows, problems = check(v, mission=mission, home=home, speed=5.0)
    lv = levels(rows)
    for name in ('RTL_RETURN_ALT', 'RTL_LAND_DELAY', 'COM_DISARM_LAND', 'GF_MAX_VER_DIST', 'GF_MAX_HOR_DIST',
                 'MPC_XY_VEL_MAX'):
        assert lv[name] == 'warn', name
    text = ' '.join(problems)
    assert 'geofence ceiling' in text and 'never lands' in text and 'outside the geofence' in text


def test_difference_from_the_project_parameter_file(tmp_path):
    f = tmp_path / 'outdoor.params'
    f.write_text('# comment\n1\t1\tRTL_RETURN_ALT\t30.0\t9\n1\t1\tGF_ACTION\t2\t6\n')
    expected = load_params_file(f)
    assert expected == {'RTL_RETURN_ALT': 30.0, 'GF_ACTION': 2.0}
    rows, problems = check(GOOD, expected)
    assert levels(rows)['RTL_RETURN_ALT'] == 'warn' and 'parameter file has 30' in problems[0]
    assert levels(rows)['GF_ACTION'] == 'ok'
