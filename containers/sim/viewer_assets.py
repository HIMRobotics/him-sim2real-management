"""Cache the pinned Three.js r160 viewer dependencies."""

import hashlib
import sys
import tempfile
from pathlib import Path
from urllib.request import urlopen

CACHE = Path(__file__).resolve().parents[2] / ".cache/three-r160"
BASE_URL = "https://raw.githubusercontent.com/mrdoob/three.js/r160/"
ASSETS = {
    "build/three.module.js": "76dea8151bc9352aef3528b4262e249b2604f62543828328db978d060d61a495",
    "examples/jsm/controls/OrbitControls.js": "5a44a9e86a2a0fb11933eed69bc2cd33c76a496854c1aed6ed776efa87d7b064",
    "examples/jsm/loaders/STLLoader.js": "896d006a48b8f125385a485ccae154dadee801a953f0b45ceffe7ddd8a29ca93",
    "LICENSE": "852e0e8699169bf9f6fdc6bda3e682d078dcbc738b5d33e74df594721bff271d",
}


def install():
    """Return verified local assets, downloading missing or corrupt files."""
    CACHE.mkdir(parents=True, exist_ok=True)
    paths = []
    for source, checksum in ASSETS.items():
        path = CACHE / Path(source).name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != checksum:
            print(f"Downloading Three.js r160: {path.name}", file=sys.stderr)
            try:
                with urlopen(BASE_URL + source, timeout=30) as response:
                    data = response.read()
                if hashlib.sha256(data).hexdigest() != checksum:
                    raise ValueError(f"checksum mismatch for {path.name}")
            except (OSError, ValueError) as exc:
                raise RuntimeError(
                    "Viewer assets unavailable. Connect to the internet and run "
                    "python3 containers/sim/viewer_assets.py before viewing offline."
                ) from exc
            # Atomic replacement keeps simultaneous exports from seeing partial files.
            with tempfile.NamedTemporaryFile(dir=CACHE, delete=False) as temp:
                pending = Path(temp.name)
                try:
                    temp.write(data)
                    temp.close()
                    pending.replace(path)
                finally:
                    pending.unlink(missing_ok=True)
        paths.append(path)
    return paths


if __name__ == "__main__":
    install()
    print(CACHE)
