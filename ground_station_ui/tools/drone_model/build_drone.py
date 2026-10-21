"""Parametric Blender model of the EIU drone (quad X) -> .glb for the GCS 3D page.

Run (Blender >= 5.x):
    /snap/bin/blender --background --python build_drone.py -- <out.glb> [preview.png] [scene.blend] [top.png]
or use build.sh (exports the glb and converts it to QML with pyside6-balsam).

Axes: X = nose, Y = left, Z = up (FLU, base_link), metres, origin = PX4 board. glTF export is Y-up
-> (X nose, Y up, -Z left), which matches the `drone` node frame in View3DPage.qml without extra rotation.

Nose = gimbal side (the GPS arrow points to it); the GPS sits behind the box.
Measured dimensions (mm): motor diagonal 800, prop 260, box 130 x 110 x 80 (L x W x H), arm to ground 260,
GPS 30 above the box / 80 behind centre, lidar scan plane 115 above the PX4 (static TF).
Values marked `# estimate` are estimated from photos.
"""

import math
import os
import sys

import bmesh
import bpy

MM = 0.001
PARAMS = dict(
    wheelbase=800 * MM,           # diagonal motor-to-motor distance
    prop_d=260 * MM,              # propeller diameter
    box_l=130 * MM, box_w=110 * MM, box_h=80 * MM,     # brown box (length along the nose x width x height)
    band_h=35 * MM,               # silver band under the brown box             # estimate
    plate_l=150 * MM, plate_w=130 * MM, plate_t=3 * MM,  # central carbon plate # estimate
    arm_d=20 * MM,                # carbon arm tube                              # estimate
    leg_len=260 * MM,             # arm to ground
    leg_d=16 * MM,                                                             # estimate
    motor_d=35 * MM,              # SunnySky motor (bell), from photos           # estimate
    guard_gap=25 * MM, guard_h=14 * MM, guard_t=4 * MM, guard_arc_deg=250,     # estimate
    lidar_head_d=57 * MM, lidar_head_h=38 * MM, lidar_fwd=15 * MM,             # from the top-view photo
    lidar_scan_above_px4=115 * MM,  # = TF base_link -> laser_link (drone_bringup static_tf)
    gps_d=55 * MM, gps_h=16 * MM, gps_back=80 * MM, gps_above_box=30 * MM,
    gimbal_fwd=35 * MM,           # C12 mount offset towards the nose from body centre   # estimate
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
    'copper': ((0.72, 0.42, 0.20, 1), 0.8, 0.35),       # motor windings
    'graphite': ((0.13, 0.13, 0.14, 1), 0.5, 0.4),    # C12 gimbal housing
    'glass': ((0.02, 0.04, 0.09, 1), 0.9, 0.05),      # lens glass
    'c12_body': ((0.085, 0.085, 0.09, 1), 0.1, 0.6),  # Skydroid C12 housing (matte charcoal)
    'c12_orange': ((0.95, 0.40, 0.04, 1), 0.0, 0.45), # C12 orange trim
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
        m.diffuse_color = rgba            # colour in the viewport / Workbench render
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
    """Bevel + smooth shading by angle."""
    bpy.ops.object.select_all(action='DESELECT')
    o.select_set(True)
    bpy.context.view_layer.objects.active = o
    m = o.modifiers.new('bevel', 'BEVEL')
    m.width, m.segments, m.limit_method = width, segments, 'ANGLE'
    bpy.ops.object.modifier_apply(modifier=m.name)
    try:
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))
    except AttributeError:                       # older Blender
        bpy.ops.object.shade_smooth()
    return o


def motor_simple(mx, my, z_base, yaw):
    """Hand-built SunnySky motor (fallback when x2216.glb is missing). Returns the z of the prop mount."""
    r = P['motor_d'] / 2
    for k in (0, 1):                                            # X-shaped base
        box((0.046, 0.008, 0.003), (mx, my, z_base + 0.0015), 'motor_blue', rot=(0, 0, yaw + math.pi / 4 + k * math.pi / 2))
    z = z_base + 0.003
    smooth_bevel(cyl(r * 0.97, 0.004, (mx, my, z + 0.002), 'motor_blue', verts=48), 0.0008)   # bell base
    z += 0.004
    cyl(r * 0.93, 0.002, (mx, my, z + 0.001), 'black', verts=48)                               # stator gap
    z += 0.002
    smooth_bevel(cyl(r, 0.018, (mx, my, z + 0.009), 'white', verts=48), 0.0008)               # bell (SunnySky label)
    z += 0.018
    smooth_bevel(cyl(r, 0.008, (mx, my, z + 0.004), 'motor_blue', verts=48), 0.0015)          # bell cap
    z += 0.008
    for k in range(6):                                          # cooling slots in the cap
        a = yaw + k * math.pi / 3
        box((0.008, 0.0035, 0.0012), (mx + 0.0105 * math.cos(a), my + 0.0105 * math.sin(a), z - 0.0004),
            'black', rot=(0, 0, a))
    cyl(0.005, 0.004, (mx, my, z + 0.002), 'silver', verts=24)                                 # shaft collar
    z += 0.004
    smooth_bevel(cyl(0.011, 0.004, (mx, my, z + 0.002), 'carbon', verts=32), 0.0006)          # prop clamp disc
    z_prop = z + 0.002
    z += 0.004
    smooth_bevel(cyl(0.0065, 0.007, (mx, my, z + 0.0035), 'motor_blue', verts=24), 0.0006)    # nut
    bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=12, radius=0.0065, location=(mx, my, z + 0.007))
    o = bpy.context.active_object
    o.scale = (1, 1, 1.6)                                       # bullet tip
    finish(o, 'motor_blue', smooth=True)
    cyl(0.0067, 0.0008, (mx, my, z + 0.0045), 'black', verts=24)                               # groove on the nut
    return z_prop


def c12_gimbal(ox, oz):
    """Hand-built Skydroid C12 (fallback when c12.glb is missing), from the drawing: 62 x 65 x 86 mm (W x L x H).
    Origin = centre of the top face of the mount plate (ox, 0, oz); lenses face +X."""
    m = MM
    box((48 * m, 40 * m, 3 * m), (ox, 0, oz - 1.5 * m), 'graphite')                       # top mount plate
    for sx in (-1, 1):
        for sy in (-1, 1):
            bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=8, radius=5 * m,
                                                 location=(ox + sx * 17 * m, sy * 14 * m, oz - 7 * m))
            finish(bpy.context.active_object, 'black', smooth=True)                       # dampers
    box((48 * m, 40 * m, 3 * m), (ox, 0, oz - 12.5 * m), 'graphite')                     # bottom plate
    smooth_bevel(cyl(13 * m, 10 * m, (ox, 0, oz - 19 * m), 'graphite', verts=40), 0.8 * m)  # yaw motor
    # L arm: back, then down
    box((24 * m, 12 * m, 5 * m), (ox - 10 * m, 0, oz - 26 * m), 'graphite')
    box((6 * m, 12 * m, 34 * m), (ox - 22 * m, 0, oz - 43 * m), 'graphite')
    # roll motor (X axis) at the back
    smooth_bevel(cyl(11 * m, 9 * m, (ox - 15 * m, 0, oz - 56 * m), 'graphite', rot=(0, math.pi / 2, 0), verts=40), 0.8 * m)
    # U frame around the camera block
    pc_x, pc_z, pod_r = ox + 6 * m, oz - 62 * m, 20 * m
    box((10 * m, 56 * m, 6 * m), (ox - 7 * m, 0, pc_z), 'graphite')
    for sy in (-1, 1):
        box((14 * m, 4 * m, 8 * m), (ox, sy * 29 * m, pc_z), 'graphite')
    # horizontal camera cylinder + two pitch motor caps
    smooth_bevel(cyl(pod_r, 44 * m, (pc_x, 0, pc_z), 'black', rot=(math.pi / 2, 0, 0), verts=48), 1.5 * m)
    for sy in (-1, 1):
        smooth_bevel(cyl(17 * m, 5 * m, (pc_x, sy * 25.5 * m, pc_z), 'silver', rot=(math.pi / 2, 0, 0), verts=40), 0.8 * m)
    # two lenses on the front face (facing +X)
    face_x = pc_x + pod_r - 2 * m
    for dz, rr in ((7 * m, 8 * m), (-9 * m, 6.5 * m)):
        smooth_bevel(cyl(rr + 1.5 * m, 6 * m, (face_x + 2 * m, 0, pc_z + dz), 'silver', rot=(0, math.pi / 2, 0), verts=40), 0.6 * m)
        cyl(rr, 1.5 * m, (face_x + 5 * m, 0, pc_z + dz), 'glass', rot=(0, math.pi / 2, 0), verts=40)


HERE = os.path.dirname(os.path.abspath(__file__))


def import_part(filename, loc, yaw=0.0):
    """Import a .glb (from convert_part.py; drone axes, metres) at loc, rotated by yaw about Z. Materials with
    the same base name ('graphite.001') are mapped to the shared material so meshes can be merged.
    False if the file is missing."""
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
    """SunnySky X2216 (x2216.glb, origin = centre of the base, 36.5 mm tall) + prop clamp + nut. Returns the z of
    the prop mount. Falls back to motor_simple if the file is missing."""
    if not import_part('x2216.glb', (mx, my, z_base), yaw):
        return motor_simple(mx, my, z_base, yaw)
    z = z_base + 0.0365 - 0.0025                                  # shaft tip 2.5 mm above the bell cap
    smooth_bevel(cyl(0.011, 0.004, (mx, my, z + 0.002), 'carbon', verts=32), 0.0006)          # prop clamp disc
    z_prop = z + 0.002
    z += 0.004
    smooth_bevel(cyl(0.0065, 0.007, (mx, my, z + 0.0035), 'motor_blue', verts=24), 0.0006)    # nut
    bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=12, radius=0.0065, location=(mx, my, z + 0.007))
    o = bpy.context.active_object
    o.scale = (1, 1, 1.6)                                        # bullet tip
    finish(o, 'motor_blue', smooth=True)
    return z_prop


def import_c12(ox, oz):
    """Skydroid C12 (c12.glb): origin = top face of the mount plate, front = +X. Falls back to c12_gimbal."""
    if not import_part('c12.glb', (ox, 0, oz)):
        print('c12.glb missing -> using the hand-built C12')
        c12_gimbal(ox, oz)


def tube_between(p0, p1, r, material):
    """Round tube between two points (arms, legs)."""
    dx, dy, dz = (p1[i] - p0[i] for i in range(3))
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = tuple((p0[i] + p1[i]) / 2 for i in range(3))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)       # cylinder Z axis -> direction p0 -> p1
    return cyl(r, length, mid, material, rot=(0, pitch, yaw))


def mesh_obj(name, verts, faces, material, smooth=False):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.update()
    o = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(o)
    return finish(o, material, smooth)


def arc_ring(center, r, t, h, start_deg, span_deg, material, steps=48):
    """Prop guard: arc with a t x h rectangular cross-section."""
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
    """One blade: long tapered shape (wide at the root, narrow at the tip), slight twist, 4 mm thick."""
    pts = []
    n = 14
    for i in range(n + 1):                         # leading edge from root to tip
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
    z_arm = 0.0                                           # built at arm height; the origin is moved to the PX4 at the end
    # ---- body: carbon plate, band, brown box, lid with four corner posts ----
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
    for sx in (-1, 1):                                    # red arm clamps on both sides
        box((0.03, P['plate_w'] * 0.9, P['arm_d'] + 0.008), (sx * (P['plate_l'] / 2 - 0.02), 0, z_arm), 'red')
    # ---- lidar on the lid: base + round head + tab pointing to the nose ----
    lx = P['lidar_fwd']
    cyl(P['lidar_head_d'] / 2 + 0.002, 0.006, (lx, 0, z_top + 0.003), 'black')
    cyl(P['lidar_head_d'] / 2, P['lidar_head_h'], (lx, 0, z_top + 0.006 + P['lidar_head_h'] / 2), 'black')
    box((0.060, 0.034, 0.018), (lx + 0.040, 0, z_top + 0.009), 'black')
    cyl(0.017, 0.020, (lx + 0.062, 0, z_top + 0.010), 'black')
    z_scan = z_top + 0.006 + P['lidar_head_h'] * 0.6
    # ---- GPS at the back ----
    gx = -P['gps_back']
    tube_between((gx, 0, z_top - 0.02), (gx, 0, z_top + P['gps_above_box']), 0.004, 'black')
    # gps_m9n.glb, arrow towards +X; fallback disc if the file is missing
    if not import_part('gps_m9n.glb', (gx, 0, z_top + P['gps_above_box'])):
        cyl(P['gps_d'] / 2, P['gps_h'], (gx, 0, z_top + P['gps_above_box'] + P['gps_h'] / 2), 'black', verts=48)
    # ---- gimbal under the body, offset towards the nose ----
    gz = z_arm - P['arm_d'] / 2 - P['plate_t']
    box((0.12, 0.07, 0.010), (0.01, 0, gz - 0.015), 'mount_blue')
    box((0.095, 0.045, 0.030), (0.0, 0, gz - 0.040), 'silver')
    import_c12(P['gimbal_fwd'], gz - 0.055)               # hangs under the box
    # ---- 4 arms + motors + props + legs + guards ----
    R = P['wheelbase'] / 2
    for deg in (45, -45, 135, -135):
        a = math.radians(deg)
        c, s = math.cos(a), math.sin(a)
        mx, my = R * c, R * s
        tube_between((0.05 * c, 0.05 * s, z_arm), (mx - 0.02 * c, my - 0.02 * s, z_arm), P['arm_d'] / 2, 'carbon')
        box((0.055, 0.045, 0.030), (mx, my, z_arm), 'white', rot=(0, 0, a))                # motor mount
        z_p = motor(mx, my, z_arm + 0.015, a)
        for k in (0, 1):
            blade((mx, my, z_p), a + math.radians(30) + k * math.pi, P['prop_d'] / 2, 'carbon')
        # leg: vertical tube under the motor mount + rubber foot
        tube_between((mx, my, z_arm), (mx, my, z_arm - P['leg_len'] + 0.025), P['leg_d'] / 2, 'carbon')
        cyl(0.013, 0.03, (mx, my, z_arm - P['leg_len'] + 0.015), 'black')
        # guard: outer arc, open towards the body; front = silver, back = brown
        rg = P['prop_d'] / 2 + P['guard_gap']
        arc_ring((mx, my, z_p), rg, P['guard_t'], P['guard_h'], deg - P['guard_arc_deg'] / 2,
                 P['guard_arc_deg'], 'cream' if c > 0 else 'brown')
    # ---- move the origin to the PX4 board: lidar scan plane above the PX4 per the TF ----
    z_px4 = z_scan - P['lidar_scan_above_px4']
    for o in bpy.context.scene.objects:
        o.location.z -= z_px4
    return z_arm - z_px4


def join_by_material():
    """Join objects with the same material into one mesh (fewer QML nodes)."""
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
    # set the rotation directly (Track To does not update between consecutive renders)
    cam.rotation_mode = 'QUATERNION'
    cam.rotation_quaternion = (Vector(look_at) - Vector(cam_loc)).to_track_quat('-Z', 'Y')
    cam.data.lens = lens
    cam.data.clip_start = 0.002                   # the default 0.1 m clips close-up shots
    scene.camera = cam
    bpy.context.view_layer.update()
    bpy.ops.render.render(write_still=True)
    bpy.data.objects.remove(cam)


def render_top(path, size=256):
    """Top-down image, transparent background, nose up (map icon; rotation = heading)."""
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
    # tight fit: motor centres + prop guard radius
    span = P['wheelbase'] / 2 / math.sqrt(2) + P['prop_d'] / 2 + P['guard_gap'] + 0.01
    cam.data.ortho_scale = 2 * span
    cam.data.clip_start, cam.data.clip_end = 0.01, 5.0
    cam.location = (0, 0, 2.0)
    cam.rotation_euler = (0, 0, -math.pi / 2)        # looking down; image up = +X (nose)
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
    print(f'base_link = PX4 board; arms at z = {z_arm * 1000:.0f} mm, ground at z = {(z_arm - P["leg_len"]) * 1000:.0f} mm')
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
        # close-up of the front-right motor (+x, -y) for comparison with photos
        R = P['wheelbase'] / 2
        mx, my = R * math.cos(math.radians(-45)), R * math.sin(math.radians(-45))
        z_m = -0.005                               # motor middle (arm at -42 mm relative to the PX4)
        close = args[1].replace('.png', '_motor.png')
        render_preview(close, cam_loc=(mx + 0.11, my - 0.13, z_m + 0.08), look_at=(mx, my, z_m), lens=55)
        print('PREVIEW', close)
        gim = args[1].replace('.png', '_gimbal.png')
        render_preview(gim, cam_loc=(0.30, -0.20, -0.10), look_at=(P['gimbal_fwd'], 0, -0.15), lens=55)
        print('PREVIEW', gim)
        gps = args[1].replace('.png', '_gps.png')      # GPS behind the box (box lid ~ +86 mm relative to the PX4)
        render_preview(gps, cam_loc=(0.06, -0.12, 0.24), look_at=(-P['gps_back'], 0, 0.115), lens=50)
        print('PREVIEW', gps)


main()
