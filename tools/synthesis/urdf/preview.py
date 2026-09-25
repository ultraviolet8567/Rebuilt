"""Side and top views of out/sphinx_urdf.zip, with the intake shown deployed (0) and stowed (-1.9)."""
import io, sys, zipfile, xml.etree.ElementTree as ET
import numpy as np, trimesh, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt

z = zipfile.ZipFile("out/sphinx_urdf.zip")
root = ET.fromstring(z.read("sphinx/sphinx.urdf"))
mesh = lambda n: trimesh.load(io.BytesIO(z.read(f"sphinx/meshes/{n}.stl")), file_type="stl")
joints = {j.get("name"): j for j in root.findall("joint")}
def origin(j): return np.array([float(v) for v in j.find("origin").get("xyz").split()])
def axis(j): return np.array([float(v) for v in j.find("axis").get("xyz").split()])
def rot(ax, q): return trimesh.transformations.rotation_matrix(q, ax)[:3, :3]

fig, axs = plt.subplots(1, 2, figsize=(16, 7))
for ax, (i, j, lab) in zip(axs, [(0, 2, "side (x fwd, z up)"), (0, 1, "top (x fwd, y left)")]):
    b = mesh("base_link").vertices[::7]
    ax.scatter(b[:, i], b[:, j], s=0.2, c="#7a4fb0", alpha=.3)
    for name, link, q, col in [("dof_intake_pivot", "intake_arm", 1.9, "#1f77b4"), ("dof_intake_pivot", "intake_arm", 0.0, "#ff7f0e"),
                               ("dof_hood", "hood", 0.0, "#2ca02c")]:
        J = joints[name]; o = origin(J); V = (rot(axis(J), q) @ mesh(link).vertices[::3].T).T + o
        ax.scatter(V[:, i], V[:, j], s=0.3, c=col, alpha=.4)
        ax.plot(o[i], o[j], "k+", ms=14, mew=2)
    for t in ("fl", "fr", "bl", "br"):
        o = origin(joints[f"dof_{t}_steer"]); W = mesh("wheel").vertices + o
        ax.scatter(W[:, i], W[:, j], s=1, c="k")
    ax.set_aspect("equal"); ax.set_title(lab); ax.grid(alpha=.3)
axs[0].axhline(0, c="brown", lw=1)
plt.suptitle("Sphinx URDF: base (purple), intake stowed 0 (orange) / deployed +1.9 (blue), hood (green), generated wheels (black)")
plt.tight_layout(); plt.savefig(sys.argv[1], dpi=90)
