from __future__ import annotations

import importlib.util
from pathlib import Path


def load_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "config_schema_check.py"
    spec = importlib.util.spec_from_file_location("config_schema_check_under_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


schema = load_module()


def test_unique_loader_rejects_duplicate_keys(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text("root:\n  key: 1\n  key: 2\n", encoding="utf-8")

    try:
        schema.load_yaml_unique(path)
    except ValueError as exc:
        assert "duplicate YAML key" in str(exc)
    else:
        raise AssertionError("duplicate YAML key was not rejected")


def test_current_config_audit_passes():
    config_dir = Path(__file__).resolve().parents[1] / "config"
    result = schema.audit_all(config_dir)

    assert result.errors == []
