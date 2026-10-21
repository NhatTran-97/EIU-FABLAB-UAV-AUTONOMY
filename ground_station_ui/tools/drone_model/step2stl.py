"""STEP -> STL, one file per volume (so convert_part.py can assign materials). Uses libgmsh from an unpacked wheel.

    python3 step2stl.py <wheel_dir> <file.stp> <out_dir>
"""
import os, sys, ctypes
W = sys.argv[1]
import glob
ctypes.CDLL(glob.glob(os.path.join(W, 'gmsh-*.data/data/lib/libgmsh.so*'))[0], mode=ctypes.RTLD_GLOBAL)
sys.path.insert(0, W)
import gmsh
gmsh.initialize(); gmsh.option.setNumber('General.Terminal', 0)
gmsh.model.occ.importShapes(sys.argv[2]); gmsh.model.occ.synchronize()
vols = gmsh.model.getEntities(3)
print('volumes:', len(vols), ' bbox(mm):', [round(v, 1) for v in gmsh.model.getBoundingBox(-1, -1)])
gmsh.option.setNumber('Mesh.MeshSizeFromCurvature', 16)
gmsh.option.setNumber('Mesh.MeshSizeMax', 3.0)
gmsh.option.setNumber('Mesh.MeshSizeMin', 0.3)
gmsh.model.mesh.generate(2)
out = sys.argv[3]; os.makedirs(out, exist_ok=True)
for dim, tag in vols:                                  # one STL per volume (its bounding surface)
    surf = [s for _, s in gmsh.model.getBoundary([(dim, tag)], oriented=False)]
    pg = gmsh.model.addPhysicalGroup(2, surf)
    gmsh.option.setNumber('Mesh.SaveAll', 0)
    gmsh.write(os.path.join(out, f'part_{tag:03d}.stl'))
    gmsh.model.removePhysicalGroups([(2, pg)])
    bb = gmsh.model.getBoundingBox(dim, tag)
    print(f'part_{tag:03d}  bbox x[{bb[0]:.1f},{bb[3]:.1f}] y[{bb[1]:.1f},{bb[4]:.1f}] z[{bb[2]:.1f},{bb[5]:.1f}]')
gmsh.finalize()
