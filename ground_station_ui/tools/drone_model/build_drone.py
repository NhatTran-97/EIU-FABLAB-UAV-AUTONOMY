"""Dung mo hinh 3D drone EIU (quad X) bang Blender theo tham so -> .glb cho trang 3D cua GCS.

Chay (Blender >= 5.x):
    /snap/bin/blender --background --python build_drone.py -- <out.glb> [preview.png] [scene.blend] [top.png]
Hoac dung build.sh (xuat glb + chuyen sang QML bang pyside6-balsam).

He truc: X = mui, Y = trai, Z = len (FLU, base_link), m, goc = mach PX4. glTF xuat Y-len
-> (X mui, Y len, -Z trai) = khung node `drone` trong View3DPage.qml, khong can xoay them.

Mui = phia gimbal (mui ten GPS chi ve mui); GPS o phia sau hop.
Kich thuoc do (mm): cheo dong co 800, canh 260, hop 130 x 110 x 80 (dai x rong x cao), tay -> mat dat 260,
GPS cao 30 tren hop / sau tam 80, tam quet lidar cao hon PX4 115 (TF static).
Gia tri `# uoc luong`: uoc tu anh.
"""

import math
import os
import sys

import bmesh
import bpy

MM = 0.001
PARAMS = dict(
    wheelbase=800 * MM,           # cheo giua 2 dong co
    prop_d=260 * MM,              # duong kinh canh quat
    box_l=130 * MM, box_w=110 * MM, box_h=80 * MM,     # hop nau (dai theo mui x rong x cao)
    band_h=35 * MM,               # dai kem duoi hop nau                      # uoc luong
    plate_l=150 * MM, plate_w=130 * MM, plate_t=3 * MM,  # tam carbon trung tam # uoc luong
    arm_d=20 * MM,                # ong carbon tay                             # uoc luong
    leg_len=260 * MM,             # tay -> mat dat
    leg_d=16 * MM,                                                             # uoc luong
    motor_d=35 * MM,              # dong co SunnySky (chuong) -- tu anh          # uoc luong
    guard_gap=25 * MM, guard_h=14 * MM, guard_t=4 * MM, guard_arc_deg=250,     # uoc luong
    lidar_head_d=57 * MM, lidar_head_h=38 * MM, lidar_fwd=15 * MM,             # tu anh nhin tren
    lidar_scan_above_px4=115 * MM,  # = TF base_link -> laser_link (drone_bringup static_tf)
    gps_d=55 * MM, gps_h=16 * MM, gps_back=80 * MM, gps_above_box=30 * MM,
    gimbal_fwd=35 * MM,           # tam treo C12 lech ve mui so voi tam than        # uoc luong
)
P = PARAMS

COLORS = {
    'carbon': ((0.05, 0.05, 0.055, 1), 0.35, 0.35),   # (rgba, metallic, roughness)
    'brown': ((0.28, 0.15, 0.09, 1), 0.0, 0.7),
    'cream': ((0.86, 0.82, 0.66, 1), 0.0, 0.7),
    'white': ((0.92, 0.92, 0.9, 1), 0.0, 0.5),
    'motor_blue': ((0.08, 0.28, 0.75, 1), 0.8, 0.3),
    'black': ((0.02, 0.02, 0.02, 1), 0.0, 0.5),
    'red': ((0.75, 0.08, 0.06, 1), 0.0, 0.5),
    'mount_blue': ((0.20, 0.62, 0.88, 1), 0.0, 0.5),
    'silver': ((0.72, 0.72, 0.74, 1), 0.7, 0.35),
    'copper': ((0.72, 0.42, 0.20, 1), 0.8, 0.35),       # cuon day dong co
    'graphite': ((0.13, 0.13, 0.14, 1), 0.5, 0.4),    # vo gimbal C12
    'glass': ((0.02, 0.04, 0.09, 1), 0.9, 0.05),      # mat kinh ong kinh
    'c12_body': ((0.085, 0.085, 0.09, 1), 0.1, 0.6),  # vo Skydroid C12 (xam than mo)
    'c12_orange': ((0.95, 0.40, 0.04, 1), 0.0, 0.45), # vien cam C12
}
_mats = {}


def mat(name):
    if name not in _mats:
        rgba, metal, rough = COLORS[name]
        m = bpy.data.materials.new(name)
        m.use_nodes = True
        bsdf = m.node_tree.nodes.get('Principled BSDF')
        bsdf.inputs['Base Color'].default_value = rgba
        bsdf.inputs['Metallic'].default_value = metal
        bsdf.inputs['Roughness'].default_value = rough
        m.diffuse_color = rgba            # mau trong viewport / render Workbench
        _mats[name] = m
    return _mats[name]


def finish(obj, material, smooth=False):
    obj.data.materials.append(mat(material))
    if smooth:
        for poly in obj.data.polygons:
            poly.use_smooth = True
    return obj


def box(size, loc, material, rot=(0, 0, 0)):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc, rotation=rot)
    o = bpy.context.active_object
    o.scale = size
    return finish(o, material)


def cyl(r, depth, loc, material, rot=(0, 0, 0), verts=32):
    bpy.ops.mesh.primitive_cylinder_add(vertices=verts, radius=r, depth=depth, location=loc, rotation=rot)
    return finish(bpy.context.active_object, material, smooth=True)


def smooth_bevel(o, width, segments=3):
    """Bevel + smooth shading theo goc."""
    bpy.ops.object.select_all(action='DESELECT')
    o.select_set(True)
    bpy.context.view_layer.objects.active = o
    m = o.modifiers.new('bevel', 'BEVEL')
    m.width, m.segments, m.limit_method = width, segments, 'ANGLE'
    bpy.ops.object.modifier_apply(modifier=m.name)
    try:
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))
    except AttributeError:                       # Blender cu
        bpy.ops.object.shade_smooth()
    return o


def motor_simple(mx, my, z_base, yaw):
    """Dong co SunnySky dung tay (du phong khi thieu x2216.glb). Tra ve z mat kep canh."""
    r = P['motor_d'] / 2
    for k in (0, 1):                                            # de gan chu X
        box((0.046, 0.008, 0.003), (mx, my, z_base + 0.0015), 'motor_blue', rot=(0, 0, yaw + math.pi / 4 + k * math.pi / 2))
    z = z_base + 0.003
    smooth_bevel(cyl(r * 0.97, 0.004, (mx, my, z + 0.002), 'motor_blue', verts=48), 0.0008)   # de chuong
    z += 0.004
    cyl(r * 0.93, 0.002, (mx, my, z + 0.001), 'black', verts=48)                               # khe stator
    z += 0.002
    smooth_bevel(cyl(r, 0.018, (mx, my, z + 0.009), 'white', verts=48), 0.0008)               # chuong (tem SunnySky)
    z += 0.018
    smooth_bevel(cyl(r, 0.008, (mx, my, z + 0.004), 'motor_blue', verts=48), 0.0015)          # nap chuong
    z += 0.008
    for k in range(6):                                          # khe thong gio tren nap
        a = yaw + k * math.pi / 3
        box((0.008, 0.0035, 0.0012), (mx + 0.0105 * math.cos(a), my + 0.0105 * math.sin(a), z - 0.0004),
            'black', rot=(0, 0, a))
    cyl(0.005, 0.004, (mx, my, z + 0.002), 'silver', verts=24)                                 # co truc
    z += 0.004
    smooth_bevel(cyl(0.011, 0.004, (mx, my, z + 0.002), 'carbon', verts=32), 0.0006)          # dia kep canh
    z_prop = z + 0.002
    z += 0.004
    smooth_bevel(cyl(0.0065, 0.007, (mx, my, z + 0.0035), 'motor_blue', verts=24), 0.0006)    # oc
    bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=12, radius=0.0065, location=(mx, my, z + 0.007))
    o = bpy.context.active_object
    o.scale = (1, 1, 1.6)                                       # dau dan
    finish(o, 'motor_blue', smooth=True)
    cyl(0.0067, 0.0008, (mx, my, z + 0.0045), 'black', verts=24)                               # ranh tren oc
    return z_prop


def c12_gimbal(ox, oz):
    """Skydroid C12 dung tay (du phong khi thieu c12.glb), theo ban ve: 62 x 65 x 86 mm (rong x dai x cao).
    Goc = tam mat tren tam treo (ox, 0, oz); ong kinh huong +X."""
    m = MM
    box((48 * m, 40 * m, 3 * m), (ox, 0, oz - 1.5 * m), 'graphite')                       # tam treo tren
    for sx in (-1, 1):
        for sy in (-1, 1):
            bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=8, radius=5 * m,
                                                 location=(ox + sx * 17 * m, sy * 14 * m, oz - 7 * m))
            finish(bpy.context.active_object, 'black', smooth=True)                       # giam chan
    box((48 * m, 40 * m, 3 * m), (ox, 0, oz - 12.5 * m), 'graphite')                     # tam duoi
    smooth_bevel(cyl(13 * m, 10 * m, (ox, 0, oz - 19 * m), 'graphite', verts=40), 0.8 * m)  # dong co yaw
    # tay chu L: ngang ra sau roi xuong
    box((24 * m, 12 * m, 5 * m), (ox - 10 * m, 0, oz - 26 * m), 'graphite')
    box((6 * m, 12 * m, 34 * m), (ox - 22 * m, 0, oz - 43 * m), 'graphite')
    # dong co roll (truc X) o mat sau
    smooth_bevel(cyl(11 * m, 9 * m, (ox - 15 * m, 0, oz - 56 * m), 'graphite', rot=(0, math.pi / 2, 0), verts=40), 0.8 * m)
    # khung U om khoi camera
    pc_x, pc_z, pod_r = ox + 6 * m, oz - 62 * m, 20 * m
    box((10 * m, 56 * m, 6 * m), (ox - 7 * m, 0, pc_z), 'graphite')
    for sy in (-1, 1):
        box((14 * m, 4 * m, 8 * m), (ox, sy * 29 * m, pc_z), 'graphite')
    # khoi camera tru ngang + 2 nap dong co pitch
    smooth_bevel(cyl(pod_r, 44 * m, (pc_x, 0, pc_z), 'black', rot=(math.pi / 2, 0, 0), verts=48), 1.5 * m)
    for sy in (-1, 1):
        smooth_bevel(cyl(17 * m, 5 * m, (pc_x, sy * 25.5 * m, pc_z), 'silver', rot=(math.pi / 2, 0, 0), verts=40), 0.8 * m)
    # 2 ong kinh tren mat truoc (nhin +X)
    face_x = pc_x + pod_r - 2 * m
    for dz, rr in ((7 * m, 8 * m), (-9 * m, 6.5 * m)):
        smooth_bevel(cyl(rr + 1.5 * m, 6 * m, (face_x + 2 * m, 0, pc_z + dz), 'silver', rot=(0, math.pi / 2, 0), verts=40), 0.6 * m)
        cyl(rr, 1.5 * m, (face_x + 5 * m, 0, pc_z + dz), 'glass', rot=(0, math.pi / 2, 0), verts=40)


HERE = os.path.dirname(os.path.abspath(__file__))


def import_part(filename, loc, yaw=0.0):
    """Nap .glb (convert_part.py; he truc drone, m) tai loc, xoay yaw quanh Z. Vat lieu trung ten
    ('graphite.001') -> doi ve vat lieu chung de gop mesh. False neu thieu file."""
    from mathutils import Matrix
    path = os.path.join(HERE, filename)
    if not os.path.exists(path):
        return False
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=path)
    new = set(bpy.data.objects) - before
    place = Matrix.Translation(loc) @ Matrix.Rotation(yaw, 4, 'Z')
    for o in new:
        if o.type != 'MESH':
            continue
        world = o.matrix_world.copy()
        o.parent = None
        o.matrix_world = place @ world
        names = [m.name.split('.')[0] for m in o.data.materials if m]
        o.data.materials.clear()
        o.data.materials.append(mat(names[0] if names and names[0] in COLORS else 'graphite'))
    for o in new:
        if o.type != 'MESH':
            bpy.data.objects.remove(o)
    return True


def motor(mx, my, z_base, yaw):
    """SunnySky X2216 (x2216.glb, goc = tam day de, cao 36.5 mm) + dia kep canh + oc. Tra ve z mat kep canh.
    Thieu file -> motor_simple."""
    if not import_part('x2216.glb', (mx, my, z_base), yaw):
        return motor_simple(mx, my, z_base, yaw)
    z = z_base + 0.0365 - 0.0025                                  # dau truc nho 2.5 mm tren nap chuong
    smooth_bevel(cyl(0.011, 0.004, (mx, my, z + 0.002), 'carbon', verts=32), 0.0006)          # dia kep canh
    z_prop = z + 0.002
    z += 0.004
    smooth_bevel(cyl(0.0065, 0.007, (mx, my, z + 0.0035), 'motor_blue', verts=24), 0.0006)    # oc
    bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=12, radius=0.0065, location=(mx, my, z + 0.007))
    o = bpy.context.active_object
    o.scale = (1, 1, 1.6)                                        # dau dan
    finish(o, 'motor_blue', smooth=True)
    return z_prop


def import_c12(ox, oz):
    """Skydroid C12 (c12.glb): goc = mat tren tam treo, truoc = +X. Thieu file -> c12_gimbal."""
    if not import_part('c12.glb', (ox, 0, oz)):
        print('c12.glb khong co -> dung C12 tu dung')
        c12_gimbal(ox, oz)


def tube_between(p0, p1, r, material):
    """Ong tron noi 2 diem (tay, chan)."""
    dx, dy, dz = (p1[i] - p0[i] for i in range(3))
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = tuple((p0[i] + p1[i]) / 2 for i in range(3))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)       # truc Z cua cylinder -> huong p0->p1
    return cyl(r, length, mid, material, rot=(0, pitch, yaw))


def mesh_obj(name, verts, faces, material, smooth=False):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.update()
    o = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(o)
    return finish(o, material, smooth)


def arc_ring(center, r, t, h, start_deg, span_deg, material, steps=48):
    """Vong bao ve canh: cung tron tiet dien chu nhat t x h."""
    cx, cy, cz = center
    verts, faces = [], []
    for i in range(steps + 1):
        a = math.radians(start_deg + span_deg * i / steps)
        c, s = math.cos(a), math.sin(a)
        for rr, zz in ((r - t / 2, cz - h / 2), (r + t / 2, cz - h / 2), (r + t / 2, cz + h / 2), (r - t / 2, cz + h / 2)):
            verts.append((cx + rr * c, cy + rr * s, zz))
    for i in range(steps):
        b, n = 4 * i, 4 * (i + 1)
        for k in range(4):
            faces.append((b + k, b + (k + 1) % 4, n + (k + 1) % 4, n + k))
    faces.append((0, 3, 2, 1))
    e = 4 * steps
    faces.append((e, e + 1, e + 2, e + 3))
    return mesh_obj('guard', verts, faces, material)


def blade(center, angle, radius, material, twist_deg=12):
    """1 la canh: hinh thon dai (rong o goc, hep o ngon), xoan nhe, day 4 mm."""
    pts = []
    n = 14
    for i in range(n + 1):                         # canh truoc tu goc ra ngon
        u = i / n
        rr = 0.014 + (radius - 0.014) * u
        chord = 0.030 * (1 - 0.45 * u) * (math.sin(math.pi * min(1, (1 - u) * 6 + 0.15)) if u > 0.85 else 1)
        pts.append((rr, chord / 2))
    outline = pts + [(rr, -w) for rr, w in reversed(pts)]
    th = 0.002
    verts = []
    for z in (-th, th):
        for rr, w in outline:
            tw = math.radians(twist_deg) * (1 - rr / radius)
            verts.append((rr, w * math.cos(tw), z + w * math.sin(tw)))
    m = len(outline)
    faces = [tuple(range(m))[::-1], tuple(range(m, 2 * m))]
    for i in range(m):
        j = (i + 1) % m
        faces.append((i, j, m + j, m + i))
    o = mesh_obj('blade', verts, faces, material)
    o.location = center
    o.rotation_euler = (0, 0, angle)
    return o


def build():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    z_arm = 0.0                                           # dung tam tay, doi goc ve PX4 o cuoi
    # ---- than: tam carbon, dai kem, hop nau, nap co 4 tru goc ----
    box((P['plate_l'], P['plate_w'], P['plate_t']), (0, 0, z_arm + P['arm_d'] / 2 + P['plate_t'] / 2), 'carbon')
    box((P['plate_l'], P['plate_w'], P['plate_t']), (0, 0, z_arm - P['arm_d'] / 2 - P['plate_t'] / 2), 'carbon')
    z0 = z_arm + P['arm_d'] / 2 + P['plate_t']
    box((P['box_l'], P['box_w'], P['band_h']), (0, 0, z0 + P['band_h'] / 2), 'cream')
    z1 = z0 + P['band_h']
    box((P['box_l'], P['box_w'], P['box_h']), (0, 0, z1 + P['box_h'] / 2), 'brown')
    z_top = z1 + P['box_h']
    for sx in (-1, 1):
        for sy in (-1, 1):
            cyl(0.008, P['box_h'], (sx * (P['box_l'] / 2 - 0.002), sy * (P['box_w'] / 2 - 0.002), z1 + P['box_h'] / 2), 'brown')
    for sx in (-1, 1):                                    # kep tay mau do 2 ben
        box((0.03, P['plate_w'] * 0.9, P['arm_d'] + 0.008), (sx * (P['plate_l'] / 2 - 0.02), 0, z_arm), 'red')
    # ---- lidar tren nap: de + dau tron + tai huong ve mui ----
    lx = P['lidar_fwd']
    cyl(P['lidar_head_d'] / 2 + 0.002, 0.006, (lx, 0, z_top + 0.003), 'black')
    cyl(P['lidar_head_d'] / 2, P['lidar_head_h'], (lx, 0, z_top + 0.006 + P['lidar_head_h'] / 2), 'black')
    box((0.060, 0.034, 0.018), (lx + 0.040, 0, z_top + 0.009), 'black')
    cyl(0.017, 0.020, (lx + 0.062, 0, z_top + 0.010), 'black')
    z_scan = z_top + 0.006 + P['lidar_head_h'] * 0.6
    # ---- GPS phia sau ----
    gx = -P['gps_back']
    tube_between((gx, 0, z_top - 0.02), (gx, 0, z_top + P['gps_above_box']), 0.004, 'black')
    # gps_m9n.glb, mui ten ve +X; thieu file -> dia tron
    if not import_part('gps_m9n.glb', (gx, 0, z_top + P['gps_above_box'])):
        cyl(P['gps_d'] / 2, P['gps_h'], (gx, 0, z_top + P['gps_above_box'] + P['gps_h'] / 2), 'black', verts=48)
    # ---- gimbal duoi than, lech ve mui ----
    gz = z_arm - P['arm_d'] / 2 - P['plate_t']
    box((0.12, 0.07, 0.010), (0.01, 0, gz - 0.015), 'mount_blue')
    box((0.095, 0.045, 0.030), (0.0, 0, gz - 0.040), 'silver')
    import_c12(P['gimbal_fwd'], gz - 0.055)               # treo duoi hop
    # ---- 4 tay + dong co + canh + chan + vong bao ve ----
    R = P['wheelbase'] / 2
    for deg in (45, -45, 135, -135):
        a = math.radians(deg)
        c, s = math.cos(a), math.sin(a)
        mx, my = R * c, R * s
        tube_between((0.05 * c, 0.05 * s, z_arm), (mx - 0.02 * c, my - 0.02 * s, z_arm), P['arm_d'] / 2, 'carbon')
        box((0.055, 0.045, 0.030), (mx, my, z_arm), 'white', rot=(0, 0, a))                # de dong co
        z_p = motor(mx, my, z_arm + 0.015, a)
        for k in (0, 1):
            blade((mx, my, z_p), a + math.radians(30) + k * math.pi, P['prop_d'] / 2, 'carbon')
        # chan: ong dung duoi de dong co + de cao su
        tube_between((mx, my, z_arm), (mx, my, z_arm - P['leg_len'] + 0.025), P['leg_d'] / 2, 'carbon')
        cyl(0.013, 0.03, (mx, my, z_arm - P['leg_len'] + 0.015), 'black')
        # vong bao ve: cung phia ngoai, mo ve phia than; truoc = kem, sau = nau
        rg = P['prop_d'] / 2 + P['guard_gap']
        arc_ring((mx, my, z_p), rg, P['guard_t'], P['guard_h'], deg - P['guard_arc_deg'] / 2,
                 P['guard_arc_deg'], 'cream' if c > 0 else 'brown')
    # ---- doi goc ve mach PX4: tam quet lidar cao hon PX4 dung TF ----
    z_px4 = z_scan - P['lidar_scan_above_px4']
    for o in bpy.context.scene.objects:
        o.location.z -= z_px4
    return z_arm - z_px4


def join_by_material():
    """Gop vat cung vat lieu thanh 1 mesh (giam so node QML)."""
    groups = {}
    for o in list(bpy.context.scene.objects):
        if o.type == 'MESH' and o.data.materials:
            groups.setdefault(o.data.materials[0].name, []).append(o)
    for name, objs in groups.items():
        bpy.ops.object.select_all(action='DESELECT')
        for o in objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = objs[0]
        bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
        if len(objs) > 1:
            bpy.ops.object.join()
        bpy.context.active_object.name = name


def render_preview(path, cam_loc=(0.95, -0.85, 0.75), look_at=(0, 0, -0.08), lens=40):
    scene = bpy.context.scene
    scene.render.engine = 'BLENDER_WORKBENCH'
    scene.display.shading.color_type = 'MATERIAL'
    scene.display.shading.light = 'STUDIO'
    scene.display.shading.show_shadows = True
    scene.render.resolution_x, scene.render.resolution_y = 1400, 1000
    scene.render.filepath = path
    if scene.world is None:
        scene.world = bpy.data.worlds.new('w')
    scene.world.color = (0.75, 0.77, 0.8)
    from mathutils import Vector
    cam = bpy.data.objects.new('cam', bpy.data.cameras.new('cam'))
    scene.collection.objects.link(cam)
    cam.location = cam_loc
    # dat rotation truc tiep (Track To khong cap nhat giua 2 lan render lien tiep)
    cam.rotation_mode = 'QUATERNION'
    cam.rotation_quaternion = (Vector(look_at) - Vector(cam_loc)).to_track_quat('-Z', 'Y')
    cam.data.lens = lens
    cam.data.clip_start = 0.002                   # mac dinh 0.1 m cat mat vat khi chup can canh
    scene.camera = cam
    bpy.context.view_layer.update()
    bpy.ops.render.render(write_still=True)
    bpy.data.objects.remove(cam)


def render_top(path, size=256):
    """Anh nhin tu tren, nen trong suot, mui huong len (icon ban do; rotation = heading)."""
    scene = bpy.context.scene
    scene.render.engine = 'BLENDER_WORKBENCH'
    scene.display.shading.color_type = 'MATERIAL'
    scene.display.shading.light = 'STUDIO'
    scene.display.shading.show_shadows = False
    scene.display.shading.show_cavity = True
    scene.render.film_transparent = True
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGBA'
    scene.render.resolution_x = scene.render.resolution_y = size
    scene.render.filepath = path
    cam = bpy.data.objects.new('top', bpy.data.cameras.new('top'))
    scene.collection.objects.link(cam)
    cam.data.type = 'ORTHO'
    # vua khit: tam dong co + ban kinh vong bao ve canh
    span = P['wheelbase'] / 2 / math.sqrt(2) + P['prop_d'] / 2 + P['guard_gap'] + 0.01
    cam.data.ortho_scale = 2 * span
    cam.data.clip_start, cam.data.clip_end = 0.01, 5.0
    cam.location = (0, 0, 2.0)
    cam.rotation_euler = (0, 0, -math.pi / 2)        # nhin xuong; tren anh = +X (mui)
    scene.camera = cam
    bpy.context.view_layer.update()
    bpy.ops.render.render(write_still=True)
    bpy.data.objects.remove(cam)
    scene.render.film_transparent = False


def main():
    args = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    out = args[0] if args else 'eiu_drone.glb'
    z_arm = build()
    join_by_material()
    print(f'base_link = mach PX4; tay o z = {z_arm * 1000:.0f} mm, mat dat o z = {(z_arm - P["leg_len"]) * 1000:.0f} mm')
    if len(args) > 2:
        bpy.ops.wm.save_as_mainfile(filepath=args[2])
    bpy.ops.export_scene.gltf(filepath=out, export_format='GLB', export_yup=True, export_apply=True)
    print('EXPORTED', out)
    if len(args) > 3:
        render_top(args[3])
        print('TOPVIEW', args[3])
    if len(args) > 1:
        render_preview(args[1])
        print('PREVIEW', args[1])
        # can canh dong co truoc-phai (+x, -y) de so voi anh that
        R = P['wheelbase'] / 2
        mx, my = R * math.cos(math.radians(-45)), R * math.sin(math.radians(-45))
        z_m = -0.005                               # giua dong co (tay o -42 mm so voi PX4)
        close = args[1].replace('.png', '_motor.png')
        render_preview(close, cam_loc=(mx + 0.11, my - 0.13, z_m + 0.08), look_at=(mx, my, z_m), lens=55)
        print('PREVIEW', close)
        gim = args[1].replace('.png', '_gimbal.png')
        render_preview(gim, cam_loc=(0.30, -0.20, -0.10), look_at=(P['gimbal_fwd'], 0, -0.15), lens=55)
        print('PREVIEW', gim)
        gps = args[1].replace('.png', '_gps.png')      # GPS phia sau hop (nap hop ~ +86 mm so voi PX4)
        render_preview(gps, cam_loc=(0.06, -0.12, 0.24), look_at=(-P['gps_back'], 0, 0.115), lens=50)
        print('PREVIEW', gps)


main()
