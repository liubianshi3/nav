from __future__ import annotations

import sys
from pathlib import Path


def _ensure_package_tree(root: Path, relative_parts: list[str]) -> None:
    current = root
    current.mkdir(parents=True, exist_ok=True)
    init_file = current / "__init__.py"
    if not init_file.exists():
        init_file.write_text("", encoding="utf-8")
    for part in relative_parts:
        current = current / part
        current.mkdir(parents=True, exist_ok=True)
        init_file = current / "__init__.py"
        if not init_file.exists():
            init_file.write_text("", encoding="utf-8")


def generate_proto_stubs(*, repo_root: Path, out_root: Path) -> Path:
    try:
        from grpc_tools import protoc
    except Exception as exc:  # pragma: no cover
        raise RuntimeError(f"grpc_tools is unavailable: {exc}") from exc

    proto_root = repo_root / "proto"
    if not proto_root.exists():
        raise RuntimeError(f"proto directory not found: {proto_root}")

    proto_files = sorted(proto_root.rglob("*.proto"))
    if not proto_files:
        raise RuntimeError(f"no proto files found under: {proto_root}")

    out_root.mkdir(parents=True, exist_ok=True)
    args_base = [
        "protoc",
        f"-I{proto_root}",
        f"--python_out={out_root}",
        f"--grpc_python_out={out_root}",
    ]
    for proto_file in proto_files:
        result = protoc.main([*args_base, str(proto_file)])
        if result != 0:
            raise RuntimeError(f"protoc failed for {proto_file} (exit={result})")

    _ensure_package_tree(out_root, ["common"])
    _ensure_package_tree(out_root, ["device"])
    _ensure_package_tree(out_root, ["physical"])
    _ensure_package_tree(out_root, ["physical", "common"])
    _ensure_package_tree(out_root, ["physical", "device"])

    if str(out_root) not in sys.path:
        sys.path.insert(0, str(out_root))
    return out_root


def ensure_grpc_generated() -> Path:
    here = Path(__file__).resolve()
    repo_root = here.parents[2]
    out_root = here.parent / "grpc_gen"
    marker = out_root / ".generated"
    proto_root = repo_root / "proto"
    proto_files = sorted(proto_root.rglob("*.proto")) if proto_root.exists() else []
    latest_proto_mtime = max((path.stat().st_mtime for path in proto_files), default=0.0)
    expected = [
        out_root / "common" / "alarm_pb2.py",
        out_root / "common" / "registry_pb2.py",
        out_root / "common" / "light_pb2.py",
    ]
    regenerate = False
    if marker.exists():
        try:
            generated_mtime = float(marker.read_text(encoding="utf-8").strip() or "0")
        except Exception:
            generated_mtime = 0.0
        if latest_proto_mtime > generated_mtime:
            regenerate = True
        if not regenerate:
            for path in expected:
                if not path.exists():
                    regenerate = True
                    break
    else:
        regenerate = True

    if not regenerate:
        if str(out_root) not in sys.path:
            sys.path.insert(0, str(out_root))
        return out_root

    generated_root = generate_proto_stubs(repo_root=repo_root, out_root=out_root)
    marker.write_text(str(latest_proto_mtime), encoding="utf-8")
    return generated_root
