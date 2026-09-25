"""Export the Sphinx Final Assembly as one glTF (a few API calls) into sphinx.gltf.zip / .glb."""
import sys, time
from pathlib import Path
import onshape_api as api

DID, WID, EID = "efe6d43f19b9a04e4fbd6feb", "f4632416d558477d4613d12f", "65536059d04cede773134438"
out = Path(__file__).parent / ".cache" / "sphinx.gltf"
if out.exists():
    print("already exported:", out, out.stat().st_size); sys.exit()
t = api.post(f"/api/v6/assemblies/d/{DID}/w/{WID}/e/{EID}/translations", {
    "formatName": "GLTF", "storeInDocument": False, "flattenAssemblies": False,
    "yAxisIsUp": False, "resolution": "medium", "binaryExport": True,
})
tid = t["id"]
for _ in range(90):
    time.sleep(10)
    st = api.get_uncached(f"/api/v6/translations/{tid}")
    print("state", st["requestState"], flush=True)
    if st["requestState"] != "ACTIVE":
        break
if st["requestState"] != "DONE":
    raise SystemExit(f"translation failed: {st.get('failureReason')}")
fid = st["resultExternalDataIds"][0]
data = api.get_uncached(f"/api/v6/documents/d/{DID}/externaldata/{fid}", accept="application/octet-stream", binary=True)
out.write_bytes(data)
print("saved", out, len(data), "bytes; API calls:", api.calls)
