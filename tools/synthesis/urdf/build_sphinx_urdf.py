"""Build a Synthesis-ready URDF of Sphinx (FRC 8567, 2026) from Onshape plus the robot code.

Geometry comes from the Onshape "Rebuilt Robot Final" Final Assembly (read only; see
export_gltf.py). Kinematics that the CAD does not carry come from the robot code:

  * The CAD drivetrain is welded solid, so the four swerve modules are generated from
    DriveConstants (module positions, wheel radius) and the CAD modules are left out.
  * The intake arm and hood are split out of their subassemblies by following the Onshape mate
    graph: cut the mates on the pivot axis, and whatever falls away from the frame moves.

Joint names starting with ``dof_`` are the ones Synthesis is patched to keep on import.
Output: out/sphinx_urdf.zip (URDF + STL meshes). Run: uv run --with ... python build_sphinx_urdf.py
"""

import io
import json
import re
import zipfile
from collections import defaultdict
from pathlib import Path

import fast_simplification
import numpy as np
import trimesh
from pygltflib import GLTF2

import onshape_api as api

HERE = Path(__file__).parent
DID, WID, EID = "efe6d43f19b9a04e4fbd6feb", "f4632416d558477d4613d12f", "65536059d04cede773134438"

# ------------------------------------------------------------------ robot code constants
# Mirrors src/main/java/frc/robot/subsystems/drive/DriveConstants.java; read from the source so a
# change there flows into the model.
DRIVE_SRC = HERE.parents[2] / "src/main/java/frc/robot/subsystems/drive/DriveConstants.java"


def _java_inches(name: str) -> float:
    m = re.search(rf"{name}\s*=\s*Units\.inchesToMeters\(([\d.]+)\)", DRIVE_SRC.read_text())
    return float(m.group(1)) * 0.0254


TRACK = _java_inches("kTrackWidthMeters")
BASE = _java_inches("kWheelBaseMeters")
_wd = re.search(r"kWheelDiameterMeters\s*=\s*Units\.inchesToMeters\(([\d.]+)\)\s*\*\s*([\d.]+)\s*/\s*([\d.]+)",
                DRIVE_SRC.read_text())
WHEEL_R = float(_wd.group(1)) * 0.0254 * float(_wd.group(2)) / float(_wd.group(3)) / 2
WHEEL_W = 0.038

# Module order matches the robot code: 0 FL, 1 FR, 2 BL, 3 BR (x forward, y left).
MODULES = [("fl", +BASE / 2, +TRACK / 2), ("fr", +BASE / 2, -TRACK / 2),
           ("bl", -BASE / 2, +TRACK / 2), ("br", -BASE / 2, -TRACK / 2)]

# CAD frame: x lateral (+x = robot left), y fore-aft (-y = forward), z up. URDF: x fwd, y left.
CAD_TO_URDF = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)

# Triangle budgets per link after decimation; parts smaller than MIN_PART_M are dropped.
BUDGET = {"base_link": 90_000, "intake_arm": 20_000, "hood": 12_000}
MIN_PART_M = 0.02
DROP = re.compile(r"screw|washer|nut\b|rivet|shaft collar|spacer|bearing|encoder magnet", re.I)


# ------------------------------------------------------------------ Onshape assembly
asm = api.get(f"/api/v6/assemblies/d/{DID}/w/{WID}/e/{EID}",
              {"includeMateFeatures": "true", "includeMateConnectors": "true"})
occ_T = {tuple(o["path"]): np.array(o["transform"]).reshape(4, 4) for o in asm["rootAssembly"]["occurrences"]}
key = lambda x: (x["documentId"], x.get("documentMicroversion"), x["elementId"], x.get("configuration"))
subs = {key(s): s for s in asm["subAssemblies"]}
top = {i["name"]: i for i in asm["rootAssembly"]["instances"]}

# Every occurrence path -> list of instance names along it.
path_names: dict[tuple, list[str]] = {}
part_paths: list[tuple] = []


def walk(instances, prefix, names):
    for inst in instances:
        if inst.get("suppressed"):
            continue
        p, n = prefix + (inst["id"],), names + [inst["name"]]
        path_names[p] = n
        if inst["type"] == "Assembly":
            walk(subs[key(inst)]["instances"], p, n)
        else:
            part_paths.append(p)


walk(asm["rootAssembly"]["instances"], (), [])


def sub_mates(top_name):
    inst = top[top_name]
    s = subs[key(inst)]
    mates = []
    for f in s["features"]:
        if f["featureType"] != "mate" or f.get("suppressed"):
            continue
        d = f["featureData"]
        e = d.get("matedEntities", [])
        if len(e) != 2 or not e[0]["matedOccurrence"] or not e[1]["matedOccurrence"]:
            continue
        path = (inst["id"],) + tuple(e[0]["matedOccurrence"])
        while path not in occ_T and len(path) > 1:
            path = path[:-1]
        T = occ_T.get(path, np.eye(4))
        cs = e[0]["matedCS"]
        o = T[:3, :3] @ np.array(cs["origin"]) + T[:3, 3]
        z = T[:3, :3] @ np.array(cs["zAxis"])
        mates.append(dict(id=f["id"], name=d["name"], type=d["mateType"], o=o, z=z / np.linalg.norm(z),
                          a=e[0]["matedOccurrence"][0], b=e[1]["matedOccurrence"][0]))
    return inst, s, mates


def components(s, mates, cut=frozenset()):
    adj = defaultdict(set)
    for m in mates:
        if m["id"] not in cut:
            adj[m["a"]].add(m["b"])
            adj[m["b"]].add(m["a"])
    seen, comps = set(), []
    for n in (i["id"] for i in s["instances"]):
        if n in seen:
            continue
        comp, st = {n}, [n]
        seen.add(n)
        while st:
            for k in adj[st.pop()]:
                if k not in seen:
                    seen.add(k); comp.add(k); st.append(k)
        comps.append(comp)
    return comps


def coaxial(mates, ref):
    return [m for m in mates if m["type"] != "FASTENED" and abs(abs(m["z"] @ ref["z"]) - 1) < 1e-3
            and np.linalg.norm((m["o"] - ref["o"]) - ((m["o"] - ref["o"]) @ ref["z"]) * ref["z"]) < 2e-3]


# --- intake arm: cut every mate on the pivot shaft's axis; everything not with the frame moves.
intake_inst, intake_s, intake_mates = sub_mates("Main Assembly <2>")
pivot_ref = next(m for m in intake_mates if m["name"] == "Cylindrical 1")
pivot_line = coaxial(intake_mates, pivot_ref)
names_in = {i["id"]: i["name"] for i in intake_s["instances"]}
frame_id = next(i for i, n in names_in.items() if n.startswith("Robot Frame"))
arm_core, intake_unknown = set(), set()
for c in components(intake_s, intake_mates, frozenset(m["id"] for m in pivot_line)):
    if frame_id in c:
        continue
    (arm_core if len(c) >= 2 else intake_unknown).update(c)

# --- hood: the rigid group holding the hood plates; it is not mated to the shooter frame.
shooter_inst, shooter_s, shooter_mates = sub_mates("Main Assembly <1>")
names_sh = {i["id"]: i["name"] for i in shooter_s["instances"]}
# Several instances are called "Hood Plate"; only the moving hood carries the gear bracket.
hood_seed = next(i for i, n in names_sh.items() if n.startswith("Hood Gear Bracket"))
comps_sh = components(shooter_s, shooter_mates)
hood_core = next(c for c in comps_sh if hood_seed in c)
shooter_big = max((c for c in comps_sh if c is not hood_core), key=len)
shooter_unknown = set().union(*[c for c in comps_sh if c is not hood_core and c is not shooter_big])

# Hood pivot = flywheel axis (line through the 4" Stealth wheels).
fly = np.array([occ_T[p][:3, 3] for p in part_paths if path_names[p][-1].startswith('4" Stealth Wheel')])
hood_axis_o = fly.mean(0)
hood_axis_z = np.array([1.0, 0, 0])


def classify(p: tuple) -> str | None:
    names = path_names[p]
    if names[0].startswith("Assembly 1") and any(n.startswith("SDS MK5") for n in names):
        return "module"                      # replaced by generated modules
    if p[0] == intake_inst["id"]:
        if p[1] in arm_core:
            return "intake_arm"
        if p[1] in intake_unknown:
            return "?intake"
    if p[0] == shooter_inst["id"]:
        if p[1] in hood_core:
            return "hood"
        if p[1] in shooter_unknown:
            return "?shooter"
    return "base_link"


# ------------------------------------------------------------------ glTF meshes
g = GLTF2().load(str(HERE / ".cache/sphinx.gltf"))
blob = g.binary_blob() or g.get_data_from_buffer_uri(g.buffers[0].uri)


def accessor(i):
    a = g.accessors[i]
    bv = g.bufferViews[a.bufferView]
    comp = {5126: np.float32, 5125: np.uint32, 5123: np.uint16, 5121: np.uint8}[a.componentType]
    n = {"SCALAR": 1, "VEC3": 3, "VEC2": 2, "VEC4": 4}[a.type]
    start = (bv.byteOffset or 0) + (a.byteOffset or 0)
    return np.frombuffer(blob, comp, a.count * n, start).reshape(a.count, n) if n > 1 else \
        np.frombuffer(blob, comp, a.count, start)


mesh_cache = {}


def mesh_of(mi):
    if mi not in mesh_cache:
        vs, fs, off = [], [], 0
        for prim in g.meshes[mi].primitives:
            v = accessor(prim.attributes.POSITION).astype(float)
            f = accessor(prim.indices).reshape(-1, 3).astype(np.int64) if prim.indices is not None \
                else np.arange(len(v)).reshape(-1, 3)
            vs.append(v); fs.append(f + off); off += len(v)
        mesh_cache[mi] = (np.vstack(vs), np.vstack(fs))
    return mesh_cache[mi]


def node_matrix(n):
    if n.matrix:
        return np.array(n.matrix).reshape(4, 4).T
    M = np.eye(4)
    if n.scale:
        M = np.diag(list(n.scale) + [1]) @ M
    if n.rotation:
        M = trimesh.transformations.quaternion_matrix([n.rotation[3], *n.rotation[:3]]) @ M
    if n.translation:
        M[:3, 3] += n.translation
    return M


leaves = []  # (name, world matrix, mesh index)


def gwalk(i, parent, pname):
    n = g.nodes[i]
    M = parent @ node_matrix(n)
    name = (n.name or pname or "").removeprefix("occurrence of ")
    if n.mesh is not None:
        leaves.append((name, M, n.mesh))
    for c in n.children or []:
        gwalk(c, M, name)


for r in g.scenes[0].nodes:
    gwalk(r, np.eye(4), "")

# Match glTF leaves to assembly part occurrences by part name and world placement.
by_name = defaultdict(list)
for p in part_paths:
    by_name[path_names[p][-1].split(" <")[0]].append(p)
matched, unmatched = {}, 0
for li, (name, M, mi) in enumerate(leaves):
    best, bd = None, 1e9
    for p in by_name.get(name, []):
        d = np.linalg.norm(occ_T[p][:3, 3] - M[:3, 3]) + np.linalg.norm(occ_T[p][:3, :3] - M[:3, :3])
        if d < bd:
            best, bd = p, d
    if best is not None and bd < 1e-3:
        matched[li] = best
    else:
        unmatched += 1

# World-space part meshes, grouped by link.
parts = []  # dict(link, name, V, F, centroid, diag)
for li, (name, M, mi) in enumerate(leaves):
    link = classify(matched[li]) if li in matched else "base_link"
    V, F = mesh_of(mi)
    W = (M[:3, :3] @ V.T).T + M[:3, 3]
    lo, hi = W.min(0), W.max(0)
    parts.append(dict(link=link, name=name, V=W, F=F, c=(lo + hi) / 2, diag=np.linalg.norm(hi - lo)))

# Unmated parts ride with a moving link only if they sit against it (within NEAR_M of one of its
# parts); anything else stays on the frame.
NEAR_M = 0.05
for kind, moving in (("?intake", "intake_arm"), ("?shooter", "hood")):
    core = [p for p in parts if p["link"] == moving]
    for p in parts:
        if p["link"] == kind:
            near = any(np.all(p["c"] >= q["V"].min(0) - NEAR_M) and np.all(p["c"] <= q["V"].max(0) + NEAR_M)
                       for q in core)
            p["link"] = moving if near else "base_link"

# ------------------------------------------------------------------ robot frame from the CAD
mods = [p for p in parts if p["link"] == "module"]
floor_z = min(p["V"][:, 2].min() for p in mods)
mod_pts = np.vstack([p["V"] for p in mods])
center = (mod_pts.min(0) + mod_pts.max(0)) / 2
# CAD module spacing vs DriveConstants (reported, not enforced)
cad_span = (CAD_TO_URDF @ (mod_pts.max(0) - mod_pts.min(0)))

def to_urdf(P):
    return (CAD_TO_URDF @ (P - np.array([center[0], center[1], floor_z])).T).T


# ------------------------------------------------------------------ write meshes + URDF
def link_mesh(link, origin_urdf):
    ps = [p for p in parts if p["link"] == link and p["diag"] >= MIN_PART_M and not DROP.search(p["name"])]
    V, F, off = [], [], 0
    for p in ps:
        V.append(to_urdf(p["V"]) - origin_urdf); F.append(p["F"] + off); off += len(p["V"])
    V, F = np.vstack(V), np.vstack(F)
    target = BUDGET[link]
    if len(F) > target:
        V, F = fast_simplification.simplify(V.astype(np.float32), F.astype(np.int32), 1 - target / len(F))
    return trimesh.Trimesh(V, F, process=True), len(ps)


out = HERE / "out"
out.mkdir(exist_ok=True)
files = {}
report = {"matched_leaves": len(matched), "unmatched_leaves": unmatched,
          "drive_constants": {"track_m": round(TRACK, 4), "wheelbase_m": round(BASE, 4), "wheel_r_m": round(WHEEL_R, 4)},
          "cad_module_extent_xy_m": [round(float(cad_span[0]), 3), round(float(cad_span[1]), 3)]}

intake_o = to_urdf(pivot_ref["o"][None])[0]
intake_axis = CAD_TO_URDF @ pivot_ref["z"]
hood_o = to_urdf(hood_axis_o[None])[0]
hood_axis = CAD_TO_URDF @ hood_axis_z
# Choose the intake axis direction so that stowing (negative joint angle) lifts the arm: the
# mate's own z axis can point either way along the shaft.
arm_c = to_urdf(np.vstack([p["V"] for p in parts if p["link"] == "intake_arm"])).mean(0) - intake_o
lift = trimesh.transformations.rotation_matrix(-0.3, intake_axis)[:3, :3] @ arm_c
if lift[2] < arm_c[2]:
    intake_axis = -intake_axis
report["intake_axis_urdf"] = [round(float(v), 3) for v in intake_axis]
origins = {"base_link": np.zeros(3), "intake_arm": intake_o, "hood": hood_o}
# The robot starts every match with the intake stowed, and the pivot's absolute encoder turns 2.5x
# per pivot turn, so only the stowed start reads unambiguously. Bake the arm into the stowed pose
# (the CAD shows it deployed, 1.9 rad away) so joint 0 is stowed, like the real robot at boot.
STOW_FROM_CAD = -1.9
pose = {"intake_arm": trimesh.transformations.rotation_matrix(STOW_FROM_CAD, intake_axis)}
for link, o in origins.items():
    m, n = link_mesh(link, o)
    if link in pose:
        m.apply_transform(pose[link])
    buf = io.BytesIO(); m.export(buf, file_type="stl"); files[f"meshes/{link}.stl"] = buf.getvalue()
    report[link] = {"parts": n, "triangles": len(m.faces), "origin_m": [round(float(x), 3) for x in o]}

# Game-piece handling points for Synthesis, in each link's own frame (URDF axes, metres):
#  * intake: the pickup zone at the arm's leading roller, located on the deployed CAD arm and
#    carried into the stowed link frame the joint uses;
#  * launcher: where fuel leaves the shooter and which way, in the hood frame (origin on the
#    flywheel axis). The ball wraps under the flywheel and leaves forward and up past the hood.
arm_pts = to_urdf(np.vstack([p["V"] for p in parts if p["link"] == "intake_arm"]))
lead = arm_pts[arm_pts[:, 0] > arm_pts[:, 0].max() - 0.08]          # the front-most 8 cm
pick_deployed = np.array([lead[:, 0].mean(), 0.0, lead[:, 2].mean()]) - intake_o
unstow = trimesh.transformations.rotation_matrix(STOW_FROM_CAD, intake_axis)[:3, :3]
pick_link = unstow @ pick_deployed
LAUNCH_ELEVATION_DEG = 62.0
launch = {"point": [0.06, 0.0, 0.20],
          "direction": [float(np.cos(np.radians(LAUNCH_ELEVATION_DEG))), 0.0,
                        float(np.sin(np.radians(LAUNCH_ELEVATION_DEG)))]}
# Hopper: the space between the hopper side plates, from just ahead of the shooter to the slider's
# front plate. Held fuel is drawn there (Synthesis itself parks every held piece at the exit).
report["hopper_parts"] = {}
hop = []
for p in parts:
    if re.search(r"hopper", p["name"], re.I):
        P = to_urdf(p["V"])
        hop.append(P)
        report["hopper_parts"].setdefault(p["name"], []).append(
            [[round(float(v), 3) for v in P.min(0)], [round(float(v), 3) for v in P.max(0)]])
hop = np.vstack(hop)
side = [to_urdf(p["V"]) for p in parts if re.search(r"hopper static", p["name"], re.I)]
inner_y = min(abs(S[:, 1]).min() for S in side)                  # inside face of the side plates
hopper_box = {"min": [round(float(hop[:, 0].min()) + 0.01, 3), round(-inner_y, 3), 0.10],
              "max": [round(float(hop[:, 0].max()) - 0.01, 3), round(inner_y, 3), round(float(hop[:, 2].max()), 3)]}
sim_meta = {
    "hopper": hopper_box,
    "intake": {"link": "intake_arm", "point": [round(float(v), 4) for v in pick_link], "diameter": 0.45,
               "maxPieces": 40},
    # Exit speed = efficiency(rpm) x flywheel surface speed. Calibrated in Synthesis on
    # 2026-09-25 by firing the robot code's own ranged shots and bisecting on where the ball comes
    # down through rim height (1.83 m): 2.4 m -> 0.40 at 3604 rpm, 3.0 m -> 0.352 at 3833,
    # 3.6 m -> 0.337 at 4130, 4.4 m -> 0.329 at 4234 (7.2-7.6 m/s; Synthesis damps game pieces
    # heavily, so range is flat in speed). The ends hold that exit speed. The team's real shot
    # table was never well tuned, so this makes the simulated shooter consistent with it rather
    # than claiming the real robot scores this way.
    "launcher": {"link": "hood", **launch, "flywheelRadius": 0.0508,
                 "efficiencyByRpm": [[2927, 0.49], [3604, 0.40], [3833, 0.352], [4130, 0.337],
                                     [4234, 0.329], [5290, 0.263]]},
    "fieldFrame": {"note": "x_code = 8.27 - X, y_code = 4.041 + Z, heading_code = heading + pi"},
}
files["sim.json"] = json.dumps(sim_meta, indent=2).encode()
report["sim"] = sim_meta

# Generated module meshes: a thin steering housing and the wheel.
steer = trimesh.creation.cylinder(radius=0.055, height=0.05)
wheel = trimesh.creation.cylinder(radius=WHEEL_R, height=WHEEL_W, sections=48)
wheel.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))  # axle along y
for nm, m in (("steer", steer), ("wheel", wheel)):
    buf = io.BytesIO(); m.export(buf, file_type="stl"); files[f"meshes/{nm}.stl"] = buf.getvalue()


def xyz(v):
    return " ".join(f"{x:.5f}" for x in v)


def axis(v):
    v = np.round(v / np.linalg.norm(v), 6)
    return " ".join(f"{x:g}" for x in v)


def link_xml(name, mesh, mass, rgba="0.45 0.2 0.6 1"):
    return f"""  <link name="{name}">
    <inertial><origin xyz="0 0 0"/><mass value="{mass}"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><geometry><mesh filename="meshes/{mesh}.stl"/></geometry><material name="{name}_mat"><color rgba="{rgba}"/></material></visual>
    <collision><geometry><mesh filename="meshes/{mesh}.stl"/></geometry></collision>
  </link>
"""


# Intake: joint 0 is stowed (robot code 0.1 rad) and deploying is +1.9 about the axis. Synthesis
# has imported this joint's sense and limits both ways on different loads, so the limits are
# symmetric and the adapter calibrates direction at attach time.
xml = ['<?xml version="1.0"?>\n<robot name="sphinx_8567">\n']
xml.append(link_xml("base_link", "base_link", 45.0))
xml.append(link_xml("intake_arm", "intake_arm", 3.0, "0.3 0.3 0.3 1"))
xml.append(link_xml("hood", "hood", 1.5, "0.3 0.3 0.3 1"))
xml.append(f"""  <joint name="dof_intake_pivot" type="revolute">
    <parent link="base_link"/><child link="intake_arm"/>
    <origin xyz="{xyz(intake_o)}"/><axis xyz="{axis(intake_axis)}"/>
    <limit lower="-2.0" upper="2.0" effort="100" velocity="6"/>
  </joint>
  <joint name="dof_hood" type="revolute">
    <parent link="base_link"/><child link="hood"/>
    <origin xyz="{xyz(hood_o)}"/><axis xyz="{axis(hood_axis)}"/>
    <limit lower="-0.35" upper="0.35" effort="50" velocity="4"/>
  </joint>
""")
for tag, x, y in MODULES:
    xml.append(link_xml(f"{tag}_steer", "steer", 0.8, "0.15 0.15 0.15 1"))
    xml.append(link_xml(f"{tag}_wheel", "wheel", 0.4, "0.05 0.05 0.05 1"))
    xml.append(f"""  <joint name="dof_{tag}_steer" type="continuous">
    <parent link="base_link"/><child link="{tag}_steer"/>
    <origin xyz="{x:.5f} {y:.5f} {WHEEL_R:.5f}"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="dof_{tag}_wheel" type="continuous">
    <parent link="{tag}_steer"/><child link="{tag}_wheel"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/>
  </joint>
""")
xml.append("</robot>\n")
files["sphinx.urdf"] = "".join(xml).encode()

with zipfile.ZipFile(out / "sphinx_urdf.zip", "w", zipfile.ZIP_DEFLATED) as z:
    for name, data in files.items():
        z.writestr(f"sphinx/{name}", data)
(out / "report.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
print("API calls this run:", api.calls)
