import struct
import os
import glob
import numpy as np

MECH_DIR = r"c:/ClaudeAsistan/projeler/6 DOF robot arm/proje dosyalari/DIY_Robotics_EducativeCell_V1_0/Mechanical"

def read_stl_binary(path):
    with open(path, 'rb') as f:
        f.read(80)
        n = struct.unpack('<I', f.read(4))[0]
        data = f.read(n * 50)
    arr = np.frombuffer(data, dtype=np.uint8).reshape(n, 50)
    floats = np.frombuffer(arr[:, :48].tobytes(), dtype='<f4').reshape(n, 12)
    verts = floats[:, 3:].reshape(n * 3, 3)
    return verts

def part_stats(path):
    v = read_stl_binary(path)
    mn = v.min(axis=0)
    mx = v.max(axis=0)
    size = mx - mn
    centroid = v.mean(axis=0)
    return mn, mx, size, centroid, len(v) // 3

files = sorted(glob.glob(os.path.join(MECH_DIR, "*.stl")))
print(f"{'Part':<32} {'Xsize':>8} {'Ysize':>8} {'Zsize':>8}    bbox min                bbox max                centroid")
print("-" * 140)
for p in files:
    name = os.path.basename(p).replace("Robot - ", "").replace(".stl", "")
    mn, mx, sz, cn, nt = part_stats(p)
    print(f"{name:<32} {sz[0]:8.2f} {sz[1]:8.2f} {sz[2]:8.2f}    "
          f"({mn[0]:7.2f},{mn[1]:7.2f},{mn[2]:7.2f})  "
          f"({mx[0]:7.2f},{mx[1]:7.2f},{mx[2]:7.2f})  "
          f"({cn[0]:7.2f},{cn[1]:7.2f},{cn[2]:7.2f})")
