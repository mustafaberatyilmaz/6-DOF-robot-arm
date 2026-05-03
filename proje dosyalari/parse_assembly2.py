"""
Find sub-assembly translation positions in the master assembly file.
Strategy: trace AXIS2_PLACEMENT_3D referenced by major sub-assemblies, extract translation.
"""
import re
import os

ASM_PATH = r"c:/ClaudeAsistan/projeler/6 DOF robot arm/proje dosyalari/DIY_Robotics_EducativeCell_V1_0/Mechanical/Robot - General Assembly (V1 - May 2020).stp"

with open(ASM_PATH, 'r', encoding='utf-8', errors='ignore') as f:
    text = f.read()

re_entity = re.compile(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;", re.DOTALL)
entities = {}
for m in re_entity.finditer(text):
    entities[int(m.group(1))] = (m.group(2), m.group(3))

cart = {}
for eid, (et, body) in entities.items():
    if et == 'CARTESIAN_POINT':
        m = re.search(r"\(\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*\)", body)
        if m:
            cart[eid] = tuple(float(v) for v in m.groups())

direction = {}
for eid, (et, body) in entities.items():
    if et == 'DIRECTION':
        m = re.search(r"\(\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*,\s*([-\d.E+]+)\s*\)", body)
        if m:
            direction[eid] = tuple(float(v) for v in m.groups())

axis_pl = {}
for eid, (et, body) in entities.items():
    if et == 'AXIS2_PLACEMENT_3D':
        ids = re.findall(r"#(\d+)", body)
        if len(ids) >= 1:
            loc = int(ids[0])
            z_axis = direction.get(int(ids[1]), None) if len(ids) >= 2 else None
            x_axis = direction.get(int(ids[2]), None) if len(ids) >= 3 else None
            axis_pl[eid] = {
                'origin': cart.get(loc),
                'z': z_axis,
                'x': x_axis,
            }

print("=" * 70)
print("SUB-ASSEMBLY POSITIONS (top-level)")
print("=" * 70)

target_names = [
    "Robot - Base (Assemblage):1",
    "Robot - J1 (Assemblage):1",
    "Robot - J2 (Bras):1",
    "Robot - J3 (Assemblage):1",
    "Robot - J4 & J5 (Assemblage):1",
    "Robot - J6 (Assemblage):1",
]

re_naoc = re.compile(r"#(\d+)\s*=\s*NEXT_ASSEMBLY_USAGE_OCCURRENCE\s*\(\s*'([^']*)'", re.MULTILINE)
naoc_ids = {}
for m in re_naoc.finditer(text):
    naoc_ids[m.group(2)] = int(m.group(1))

re_cdsr = re.compile(r"#(\d+)\s*=\s*CONTEXT_DEPENDENT_SHAPE_REPRESENTATION\s*\(\s*#(\d+)\s*,\s*#(\d+)\s*\)")
cdsr = {}
for m in re_cdsr.finditer(text):
    cdsr[int(m.group(1))] = (int(m.group(2)), int(m.group(3)))

re_pdsr = re.compile(r"#(\d+)\s*=\s*PRODUCT_DEFINITION_SHAPE\s*\(\s*([^)]*)\)\s*;")
pds = {}
for m in re_pdsr.finditer(text):
    refs = re.findall(r"#(\d+)", m.group(2))
    pds[int(m.group(1))] = [int(r) for r in refs]

re_idt = re.compile(r"#(\d+)\s*=\s*ITEM_DEFINED_TRANSFORMATION\s*\(\s*[^,]*,\s*[^,]*,\s*#(\d+)\s*,\s*#(\d+)\s*\)\s*;")
idt = {}
for m in re_idt.finditer(text):
    idt[int(m.group(1))] = (int(m.group(2)), int(m.group(3)))

re_rrwt = re.compile(r"#(\d+)\s*=\s*\(\s*REPRESENTATION_RELATIONSHIP\s*\(\s*[^)]*\)\s*REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*#(\d+)\s*\)\s*SHAPE_REPRESENTATION_RELATIONSHIP\s*\(\s*\)\s*\)\s*;")
rrwt = {}
for m in re_rrwt.finditer(text):
    rrwt[int(m.group(1))] = int(m.group(2))

print(f"\nNAOC entries: {len(naoc_ids)}")
print(f"CDSR entries: {len(cdsr)}")
print(f"IDT entries: {len(idt)}")
print(f"PDS entries: {len(pds)}")

print("\n--- All NAOC names with their CDSR transformation ---")

for name in target_names:
    if name not in naoc_ids:
        for k in naoc_ids:
            if name.split(":")[0] in k:
                print(f"  Approx match: {k} -> #{naoc_ids[k]}")
        continue
    eid = naoc_ids[name]
    print(f"\n{name} (NAOC #{eid})")
    for cdsr_id, (pds_ref, sr_ref) in cdsr.items():
        if pds_ref == eid:
            print(f"  CDSR #{cdsr_id} -> SR #{sr_ref}")
            for sr in [sr_ref]:
                if sr in entities:
                    et, body = entities[sr]
                    refs = re.findall(r"#(\d+)", body)
                    print(f"    SR type: {et}, refs: {refs[:5]}")

print("\n--- Looking for top-level assembly translations ---")
for name in target_names:
    if name in naoc_ids:
        eid = naoc_ids[name]
        body = entities[eid][1]
        refs = re.findall(r"#(\d+)", body)
        print(f"\n{name}:")
        print(f"  Body refs: {refs}")
