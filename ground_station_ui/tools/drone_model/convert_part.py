"""Linh kien chinh hang (STEP -> STL tung khoi bang step2stl.py) -> .glb gon cho build_drone.py.

    /snap/bin/blender --background --python convert_part.py -- <preset> <thu_muc_stl> <out.glb>
    preset: c12     (Skydroid C12, GrabCAD)   -> c12.glb
            x2216   (SunnySky X2216, GrabCAD) -> x2216.glb
            gps_m9n (GPS Holybro M9N)         -> gps_m9n.glb

Ra glb: he truc drone (X = mui, Y = trai, Z = len), met. Moi preset: xoay STEP ve he drone, dat goc,
gan mau theo so khoi (xem bbox in ra boi step2stl.py), gioi han so tam giac (nhe cho QML).
"""

import glob
import math
import os
import sys

import bpy

COLORS = {
    'graphite': ((0.13, 0.13, 0.14, 1), 0.5, 0.4),
    'black': ((0.02, 0.02, 0.02, 1), 0.0, 0.5),
    'white': ((0.92, 0.92, 0.9, 1), 0.0, 0.5),
    'motor_blue': ((0.08, 0.28, 0.75, 1), 0.8, 0.3),
    'silver': ((0.72, 0.72, 0.74, 1), 0.7, 0.35),
    'copper': ((0.72, 0.42, 0.20, 1), 0.8, 0.35),
    'c12_body': ((0.085, 0.085, 0.09, 1), 0.1, 0.6),   # vo C12
    'c12_orange': ((0.95, 0.40, 0.04, 1), 0.0, 0.45),  # vien cam quanh mat kinh / nap pitch
    'glass': ((0.02, 0.04, 0.09, 1), 0.9, 0.05),       # mat kinh ong kinh
}

PRESETS = {
    # STEP: truoc = +Y, len = +Z, goc = mat tren tam treo. -> xoay -90 do quanh Z (truoc -> +X).
    'c12': dict(
        rotate=(0, 0, -math.pi / 2), origin_mm=(0, 0, 0),
        # 1, 8 tam treo + 2-7 giam chan: den; 9 dong co yaw, 10 tay L, 11 khung + nap pitch, 12 khoi camera: xam than
        material={9: 'c12_body', 10: 'c12_body', 11: 'c12_body', 12: 'c12_body'},
        default_material='black',
        extras='c12_trim',                                # vien cam + mat kinh (khong co trong STEP)
        max_tris={12: 12000, 11: 9000, 10: 6000}, default_max_tris=2500,
    ),
    # STEP: truc dong co theo Y, dau truc (phia canh) o y = 4.8, de o y = 41.3; tam truc x = 12, z = 25.6.
    # -> xoay -90 do quanh X (truc -> +Z, dau truc len tren); goc = tam mat day de (dat len de dong co).
    'x2216': dict(
        rotate=(-math.pi / 2, 0, 0), origin_mm=(12.0, 25.6, -41.3),     # (sau khi xoay) tam truc, day de
        # 1 nap chuong, 3 vo co tem, 18 de
        material={1: 'motor_blue', 3: 'white', 18: 'motor_blue', 19: 'silver', 20: 'motor_blue',
                  21: 'motor_blue', 28: 'silver', 29: 'silver', 40: 'silver',
                  **{k: 'copper' for k in range(4, 18)}},
        default_material='silver',                        # vit, bac dan
        max_tris={1: 3000, 3: 1500, 18: 2000}, default_max_tris=150,
    ),
    # GPS Holybro (M9N, cung vo voi M10 tren drone): vo tron D51.2 x 14.4, duoi ra day o +Y -> mui ten
    # chi -Y. -> xoay +90 do quanh Z (mui ten -> +X = mui drone); goc = tam mat day (dat len dau cot GPS).
    'gps_m9n': dict(
        rotate=(0, 0, math.pi / 2), origin_mm=(0, 0, -7.2),
        material={1: 'black'},                            # vo
        default_material='white',                         # 2-3 cua so den, 4 mui ten, 5-16 chu, 17-18 nut
        max_tris={1: 6000}, default_max_tris=400,
    ),
}


def _mesh(name, verts, faces, mat_name):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.update()
    o = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(o)
    o.data.materials.append(material(mat_name))
    return o


def _ring(name, outline_out, outline_in, depth_axis, d0, d1, mat_name):
    """Vanh day (d0 -> d1 theo truc depth_axis) giua 2 duong bao 2D cung so diem (u, v)."""
    def p3(u, v, d):
        return {'x': (d, u, v), 'y': (u, d, v)}[depth_axis]
    n = len(outline_out)
    verts = [p3(u, v, d) for d in (d0, d1) for (u, v) in outline_out] + \
            [p3(u, v, d) for d in (d0, d1) for (u, v) in outline_in]
    oa, ob, ia, ib = 0, n, 2 * n, 3 * n           # out@d0, out@d1, in@d0, in@d1
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [(oa + i, oa + j, ob + j, ob + i), (ia + j, ia + i, ib + i, ib + j),
                  (ob + i, ob + j, ib + j, ib + i), (oa + j, oa + i, ia + i, ia + j)]
    return _mesh(name, verts, faces, mat_name)


def _circle(cu, cv, r, n=48):
    return [(cu + r * math.cos(2 * math.pi * k / n), cv + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def _rounded_rect(cu, cv, w, h, r, per_corner=10):
    pts = []
    for (sx, sy, a0) in ((1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270)):
        ccu, ccv = cu + sx * (w / 2 - r), cv + sy * (h / 2 - r)
        for k in range(per_corner + 1):
            a = math.radians(a0 + 90 * k / per_corner)
            pts.append((ccu + r * math.cos(a), ccv + r * math.sin(a)))
    return pts


def c12_trim():
    """Chi tiet mau theo anh san pham C12 (do tren anh chieu cua STEP, mm, X = mui, Z = len):
    vien cam vuong bo goc quanh mat kinh (mat truoc X = 31.3, tam Z = -59.6, ~32 x 38),
    2 ong kinh (tren Z -49.5, duoi Z -68.1): vanh kim loai + mat kinh; vong cam tren mat ngoai 2 nap
    pitch (tam X 10.7, Z -59.4, R ~12, mat ngoai Y = +-31.3)."""
    m = 0.001
    fx = 31.3 * m
    _ring('c12_bezel', _rounded_rect(0, -59.6 * m, 33.5 * m, 39.5 * m, 6 * m),
          _rounded_rect(0, -59.6 * m, 30.5 * m, 36.5 * m, 4.5 * m), 'x', fx - 0.4 * m, fx + 0.6 * m, 'c12_orange')
    for z, r_glass, r_rim in ((-49.5, 4.8, 7.6), (-68.1, 3.2, 5.4)):
        _ring('c12_lens_rim', _circle(0, z * m, r_rim * m), _circle(0, z * m, r_glass * m), 'x',
              fx - 0.4 * m, fx + 0.9 * m, 'silver')
        disc = _circle(0, z * m, r_glass * m)
        verts = [(fx + 0.5 * m, u, v) for (u, v) in disc] + [(fx + 0.5 * m, 0, z * m)]
        c = len(disc)
        _mesh('c12_glass', verts, [(c, i, (i + 1) % c) for i in range(c)], 'glass')
    for sy in (-1, 1):
        y0 = sy * 31.3 * m
        _ring('c12_pitch_ring', _circle(10.7 * m, -59.4 * m, 12.3 * m), _circle(10.7 * m, -59.4 * m, 10.6 * m),
              'y', y0 - sy * 0.4 * m, y0 + sy * 0.6 * m, 'c12_orange')


def material(name):
    m = bpy.data.materials.get(name)
    if m is None:
        rgba, metal, rough = COLORS[name]
        m = bpy.data.materials.new(name)
        m.use_nodes = True
        bsdf = m.node_tree.nodes.get('Principled BSDF')
        bsdf.inputs['Base Color'].default_value = rgba
        bsdf.inputs['Metallic'].default_value = metal
        bsdf.inputs['Roughness'].default_value = rough
        m.diffuse_color = rgba
    return m


def main():
    preset_name, parts_dir, out = sys.argv[sys.argv.index('--') + 1:][:3]
    cfg = PRESETS[preset_name]
    bpy.ops.wm.read_factory_settings(use_empty=True)
    ox, oy, oz = (v * 0.001 for v in cfg['origin_mm'])
    total = 0
    for path in sorted(glob.glob(os.path.join(parts_dir, 'part_*.stl'))):
        tag = int(os.path.basename(path)[5:8])
        bpy.ops.wm.stl_import(filepath=path)
        o = bpy.context.selected_objects[0]
        o.name = f'{preset_name}_part_{tag:03d}'
        o.scale = (0.001, 0.001, 0.001)
        o.rotation_euler = cfg['rotate']
        bpy.context.view_layer.objects.active = o
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        o.data.transform(__import__('mathutils').Matrix.Translation((-ox, -oy, -oz)))
        bpy.ops.object.mode_set(mode='EDIT')              # STL tach dinh theo tam giac -> gop lai
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.remove_doubles(threshold=0.00002)
        bpy.ops.object.mode_set(mode='OBJECT')
        tris = sum(len(p.vertices) - 2 for p in o.data.polygons)
        limit = cfg['max_tris'].get(tag, cfg['default_max_tris'])
        if tris > limit:
            mod = o.modifiers.new('decimate', 'DECIMATE')
            mod.ratio = limit / tris
            bpy.ops.object.modifier_apply(modifier=mod.name)
        o.data.materials.clear()
        o.data.materials.append(material(cfg['material'].get(tag, cfg['default_material'])))
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))
        n = sum(len(p.vertices) - 2 for p in o.data.polygons)
        total += n
        print(f'{o.name}: {tris} -> {n} tris')
    if cfg.get('extras'):
        globals()[cfg['extras']]()
    lo = [min(v.co[i] for o in bpy.data.objects for v in o.data.vertices) for i in range(3)]
    hi = [max(v.co[i] for o in bpy.data.objects for v in o.data.vertices) for i in range(3)]
    print(f'TOTAL {total} tris; bbox m: {[round(v, 4) for v in lo]} .. {[round(v, 4) for v in hi]}')
    bpy.ops.export_scene.gltf(filepath=out, export_format='GLB', export_yup=True, export_apply=True)
    print('EXPORTED', out)


main()
