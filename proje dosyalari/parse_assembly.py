"""
DIY Robotics 6-axis arm — General Assembly STP parser.
Goal: extract translation vectors of each major part to deduce joint axis positions.
"""
import re
from collections import defaultdict

ASM_PATH = r"c:/ClaudeAsistan/projeler/6 DOF robot arm/proje dosyalari/DIY_Robotics_EducativeCell_V1_0/Mechanical/Robot - General Assembly (V1 - May 2020).stp"

with open(ASM_PATH, 'r', encoding='utf-8', errors='ignore') as f:
    lines = f.readlines()

text = ''.join(lines)

entities = {}
re_entity = re.compile(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;", re.DOTALL)

for m in re_entity.finditer(text):
    eid = int(m.group(1))
    etype = m.group(2)
    body = m.group(3)
    entities[eid] = (etype, body)

print(f"Total entities: {len(entities)}")

products = {}
for eid, (etype, body) in entities.items():
    if etype == 'PRODUCT':
        m = re.match(r"\s*'([^']*)'", body)
        if m:
            products[eid] = m.group(1)

print(f"Products: {len(products)}")
for eid, name in products.items():
    print(f"  #{eid}: {name}")

cart_points = {}
for eid, (etype, body) in entities.items():
    if etype == 'CARTESIAN_POINT':
        m = re.search(r"\(\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*\)", body)
        if m:
            cart_points[eid] = tuple(float(v) for v in m.groups())

axis_pl = {}
for eid, (etype, body) in entities.items():
    if etype == 'AXIS2_PLACEMENT_3D':
        ids = re.findall(r"#(\d+)", body)
        if len(ids) >= 1:
            loc = int(ids[0])
            if loc in cart_points:
                axis_pl[eid] = cart_points[loc]

next_asm = {}
for eid, (etype, body) in entities.items():
    if etype == 'NEXT_ASSEMBLY_USAGE_OCCURRENCE':
        m = re.match(r"\s*'([^']*)'", body)
        if m:
            next_asm[eid] = (m.group(1), body)

print(f"\nAxis placements: {len(axis_pl)}")
print(f"Cartesian points: {len(cart_points)}")
print(f"Next assembly occurrences: {len(next_asm)}")

print("\n--- Robot part instances ---")
robot_parts = [(eid, name) for eid, (name, _) in next_asm.items()
               if 'Robot -' in name or 'MG996R' in name or 'Micro' in name or 'SG90' in name]
for eid, name in robot_parts[:30]:
    print(f"  #{eid}: {name}")

print(f"\nTotal robot/servo instances: {len(robot_parts)}")
