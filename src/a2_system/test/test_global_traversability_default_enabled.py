#!/usr/bin/env python3
"""Verify global traversability layer defaults and switches in launch/config files.

Run without ROS:
  python3 -m pytest src/a2_system/test/test_global_traversability_default_enabled.py -q
"""

import os
import unittest
from pathlib import Path


_WS = Path(__file__).resolve().parents[3]


class TestLaunchDefaults(unittest.TestCase):
    def test_jt128_3d_navigation_default_disabled(self):
        path = _WS / "src" / "a2_bringup" / "launch" / "jt128_3d_navigation.launch.py"
        self.assertTrue(path.exists(), f"file not found: {path}")
        text = path.read_text()
        self.assertIn(
            'enable_global_traversability_layer", default_value="false"',
            text,
            "jt128_3d_navigation.launch.py must default enable_global_traversability_layer to false",
        )

    def test_nav2_3d_default_disabled(self):
        path = _WS / "src" / "a2_bringup" / "launch" / "nav2_3d.launch.py"
        self.assertTrue(path.exists(), f"file not found: {path}")
        text = path.read_text()
        self.assertIn(
            'enable_global_traversability_layer",\n            default_value="false"',
            text,
            "nav2_3d.launch.py must default enable_global_traversability_layer to false",
        )

    def test_nav2_3d_config_path_not_empty_default(self):
        path = _WS / "src" / "a2_bringup" / "launch" / "nav2_3d.launch.py"
        text = path.read_text()
        self.assertIn(
            "global_traversability_integrator.yaml",
            text,
            "nav2_3d.launch.py global_traversability_config must point to integrator YAML",
        )


class TestStackScript(unittest.TestCase):
    def test_stack_script_has_env_default(self):
        path = _WS / "src" / "a2_system" / "tools" / "start_jt128_3d_stack.sh"
        self.assertTrue(path.exists(), f"file not found: {path}")
        text = path.read_text()
        self.assertIn(
            'ENABLE_GLOBAL_TRAVERSABILITY_LAYER="${A2_ENABLE_GLOBAL_TRAVERSABILITY_LAYER:-false}"',
            text,
            "start_jt128_3d_stack.sh must default ENABLE_GLOBAL_TRAVERSABILITY_LAYER to false",
        )

    def test_stack_script_has_no_flag(self):
        path = _WS / "src" / "a2_system" / "tools" / "start_jt128_3d_stack.sh"
        text = path.read_text()
        self.assertIn(
            "--no-global-traversability-layer",
            text,
            "start_jt128_3d_stack.sh must support --no-global-traversability-layer",
        )

    def test_stack_script_passes_to_launch(self):
        path = _WS / "src" / "a2_system" / "tools" / "start_jt128_3d_stack.sh"
        text = path.read_text()
        self.assertIn(
            "enable_global_traversability_layer:=${ENABLE_GLOBAL_TRAVERSABILITY_LAYER}",
            text,
            "start_jt128_3d_stack.sh must pass enable_global_traversability_layer to ros2 launch",
        )

    def test_stack_script_writes_state(self):
        path = _WS / "src" / "a2_system" / "tools" / "start_jt128_3d_stack.sh"
        text = path.read_text()
        self.assertIn(
            "enable_global_traversability_layer",
            text,
            "NAV_STATE_FILE must record enable_global_traversability_layer",
        )


class TestWebBackend(unittest.TestCase):
    def test_start_navigation_request_has_default(self):
        path = _WS / "web_console" / "backend" / "models.py"
        self.assertTrue(path.exists(), f"file not found: {path}")
        text = path.read_text()
        self.assertIn(
            "enable_global_traversability_layer: bool = False",
            text,
            "StartNavigationRequest must default enable_global_traversability_layer to False",
        )

    def test_docker_compose_has_env(self):
        path = _WS / "docker-compose.a2.yml"
        self.assertTrue(path.exists(), f"file not found: {path}")
        text = path.read_text()
        self.assertIn(
            "A2_ENABLE_GLOBAL_TRAVERSABILITY_LAYER",
            text,
            "docker-compose.a2.yml must set A2_ENABLE_GLOBAL_TRAVERSABILITY_LAYER env var",
        )

    def test_existing_integrator_tests_still_import(self):
        import sys
        sys.path.insert(0, str(_WS / "src" / "a2_system" / "scripts"))
        from global_traversability_integrator import (
            GlobalTraversabilityMemory,
            validate_frame,
            should_update_with_tf,
        )
        self.assertTrue(callable(validate_frame))
        self.assertTrue(callable(should_update_with_tf))
        mem = GlobalTraversabilityMemory()
        self.assertIsNotNone(mem)


if __name__ == "__main__":
    unittest.main()
