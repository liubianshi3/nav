#!/usr/bin/env bash
set -euo pipefail
# Verify require_a2_system_executable fails when global_traversability_integrator.py is missing
# and succeeds when present.

TMPDIR=$(mktemp -d)
trap 'rm -rf "$TMPDIR"' EXIT

WORKSPACE="$TMPDIR/workspace"
mkdir -p "$WORKSPACE/install/a2_system/lib/a2_system"
mkdir -p "$WORKSPACE/src/a2_system/scripts"

export WORKSPACE

require_a2_system_executable() {
  local name="$1"
  local install_path="${WORKSPACE}/install/a2_system/lib/a2_system/${name}"
  local source_path="${WORKSPACE}/src/a2_system/scripts/${name}"
  if [[ -x "$install_path" ]]; then
    return 0
  fi
  if [[ -x "$source_path" ]]; then
    echo "[WARN] install executable missing for ${name}; launch will fall back to source path ${source_path}" >&2
    return 0
  fi
  echo "[ERROR] required a2_system executable is unavailable: ${name} (checked ${install_path} and ${source_path})" >&2
  return 1
}

# Test 1: missing => must fail
echo "=== Test 1: missing executable must fail ==="
if require_a2_system_executable "global_traversability_integrator.py" 2>/dev/null; then
  echo "FAIL: should have failed for missing executable"
  exit 1
else
  echo "PASS: correctly failed for missing executable"
fi

# Test 2: present in source path => succeed with warning
echo "=== Test 2: executable in source path must succeed ==="
touch "$WORKSPACE/src/a2_system/scripts/global_traversability_integrator.py"
chmod +x "$WORKSPACE/src/a2_system/scripts/global_traversability_integrator.py"
if require_a2_system_executable "global_traversability_integrator.py"; then
  echo "PASS: succeeded for present executable"
else
  echo "FAIL: should succeed when executable exists"
  exit 1
fi

# Test 3: present in install path => succeed
echo "=== Test 3: executable in install path must succeed ==="
touch "$WORKSPACE/install/a2_system/lib/a2_system/global_traversability_integrator.py"
chmod +x "$WORKSPACE/install/a2_system/lib/a2_system/global_traversability_integrator.py"
if require_a2_system_executable "global_traversability_integrator.py"; then
  echo "PASS: succeeded for install-path executable"
else
  echo "FAIL: should succeed when install executable exists"
  exit 1
fi

# Test 4: verify the exact check is in the start script
echo "=== Test 4: check present in start_jt128_3d_stack.sh ==="
START_SCRIPT="/home/unitree/ws/device-navigation/src/a2_system/tools/start_jt128_3d_stack.sh"
if grep -q 'require_a2_system_executable "global_traversability_integrator.py"' "$START_SCRIPT"; then
  echo "PASS: pre-flight check found in start script"
else
  echo "FAIL: pre-flight check not found in start script"
  exit 1
fi

echo ""
echo "All pre-flight tests passed."
