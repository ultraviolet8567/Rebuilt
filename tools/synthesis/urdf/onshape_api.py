"""Minimal Onshape REST client for the Sphinx URDF exporter.

Credentials come from URDF_ONSHAPE_ACCESS_KEY / URDF_ONSHAPE_SECRET_KEY, read from the environment
or from the file named by ONSHAPE_KEYS_FILE. Values are never printed. Every call counts against
the account's Onshape API allocation, so responses are cached on disk by URL.
"""

import hashlib
import json
import os
import re
from pathlib import Path

import requests

BASE = "https://cad.onshape.com"
CACHE = Path(os.environ.get("ONSHAPE_CACHE", Path(__file__).parent / ".cache"))


def _keys() -> tuple[str, str]:
    access = os.environ.get("URDF_ONSHAPE_ACCESS_KEY")
    secret = os.environ.get("URDF_ONSHAPE_SECRET_KEY")
    keys_file = os.environ.get("ONSHAPE_KEYS_FILE")
    if (not access or not secret) and keys_file:
        for line in Path(keys_file).read_text().splitlines():
            m = re.match(r"\s*(URDF_ONSHAPE_(?:ACCESS|SECRET)_KEY)\s*=\s*(.*)", line)
            if m:
                value = m.group(2).strip().strip("'\"")
                if m.group(1).endswith("ACCESS_KEY"):
                    access = access or value
                else:
                    secret = secret or value
    if not access or not secret:
        raise SystemExit("Set URDF_ONSHAPE_ACCESS_KEY/SECRET_KEY or ONSHAPE_KEYS_FILE")
    return access, secret


_session = requests.Session()
_session.auth = _keys()
calls = 0


def get(path: str, params: dict | None = None, accept: str = "application/json", binary=False):
    """GET with an on-disk cache; `binary` returns bytes."""
    global calls
    key = hashlib.sha1((path + json.dumps(params or {}, sort_keys=True) + accept).encode()).hexdigest()
    CACHE.mkdir(parents=True, exist_ok=True)
    hit = CACHE / key
    if hit.exists():
        data = hit.read_bytes()
    else:
        r = _session.get(BASE + path, params=params, headers={"Accept": accept}, timeout=300)
        calls += 1
        if r.status_code != 200:
            raise RuntimeError(f"{r.status_code} {path}: {r.text[:300]}")
        data = r.content
        hit.write_bytes(data)
    return data if binary else json.loads(data)


def post(path: str, body: dict):
    """Uncached POST; used only to start exports (translations)."""
    global calls
    r = _session.post(BASE + path, json=body, headers={"Accept": "application/json"}, timeout=300)
    calls += 1
    if r.status_code not in (200, 201):
        raise RuntimeError(f"{r.status_code} {path}: {r.text[:300]}")
    return r.json()


def get_uncached(path: str, accept="application/json", binary=False):
    global calls
    r = _session.get(BASE + path, headers={"Accept": accept}, timeout=600)
    calls += 1
    if r.status_code != 200:
        raise RuntimeError(f"{r.status_code} {path}: {r.text[:300]}")
    return r.content if binary else r.json()
