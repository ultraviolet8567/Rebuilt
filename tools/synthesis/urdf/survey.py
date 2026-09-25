"""List every moving mate in the Sphinx assembly with the parts each side carries."""

import sys
from collections import defaultdict

import onshape_api as api

DID, WID, EID = "efe6d43f19b9a04e4fbd6feb", "f4632416d558477d4613d12f", "65536059d04cede773134438"

asm = api.get(f"/api/v6/assemblies/d/{DID}/w/{WID}/e/{EID}",
              {"includeMateFeatures": "true", "includeMateConnectors": "true"})

def key(x):
    return (x["documentId"], x.get("documentMicroversion"), x["elementId"], x.get("configuration"))

subs = {key(s): s for s in asm["subAssemblies"]}

def walk(instances, sub_path_name):
    """Yield (assembly-name, assembly-dict) for every (sub)assembly occurrence."""
    for inst in instances:
        if inst.get("type") == "Assembly" and not inst.get("suppressed"):
            s = subs.get(key(inst))
            if s:
                yield inst["name"], s
                yield from walk(s["instances"], inst["name"])

seen = set()
for name, s in walk(asm["rootAssembly"]["instances"], "root"):
    if id(s) in seen:
        continue
    seen.add(id(s))
    inst_name = {i["id"]: i["name"] for i in s["instances"]}
    fastened = defaultdict(set)
    moving = []
    for f in s.get("features", []):
        if f["featureType"] != "mate" or f.get("suppressed"):
            continue
        d = f["featureData"]
        ents = d.get("matedEntities", [])
        if len(ents) != 2 or not ents[0]["matedOccurrence"] or not ents[1]["matedOccurrence"]:
            continue
        a, b = ents[0]["matedOccurrence"][0], ents[1]["matedOccurrence"][0]
        if d["mateType"] == "FASTENED":
            fastened[a].add(b); fastened[b].add(a)
        else:
            moving.append((d["mateType"], d["name"], a, b))

    def group(start, blocked):
        out, stack = {start}, [start]
        while stack:
            for n in fastened[stack.pop()]:
                if n not in out and n != blocked:
                    out.add(n); stack.append(n)
        return out

    print(f"\n== {name}: {len(s['instances'])} instances, {len(moving)} moving mates")
    for t, n, a, b in moving:
        ga, gb = group(a, b), group(b, a)
        small = ga if len(ga) <= len(gb) else gb
        names = sorted({inst_name.get(i, "?").split(" <")[0] for i in small})
        print(f"  {t:11s} {n:18s} smaller side {len(small):3d} parts: {', '.join(names)[:150]}")
print(f"\nAPI calls made: {api.calls}", file=sys.stderr)
