#!/usr/bin/env bash
set -eo pipefail   # NOTE: no -u globally; we'll re-enable around our code

# --- Config (you can leave this blank; we auto-detect if missing) ---
ROS_SETUP="/opt/ros/jazzy/setup.bash"   # Fallback if auto-detect fails
REQS_FILE="requirements.txt"
VENV_DIR="venv"
PY_SCRIPT="hydra_node_cli.py"
# -------------------------------------------------------------------

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

safe_source() {
  # Temporarily disable nounset and source a file
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

echo "==> 1) Locating ROS 2 environment"
if [[ ! -f "$ROS_SETUP" ]]; then
  # Try common ROS distros in order
  for d in jazzy humble iron rolling; do
    if [[ -f "/opt/ros/$d/setup.bash" ]]; then
      ROS_SETUP="/opt/ros/$d/setup.bash"
      break
    fi
  done
fi

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: Could not find a ROS setup.bash in /opt/ros/{jazzy,humble,iron,rolling}."
  echo "       Install ROS 2 or set ROS_SETUP to the correct path inside this script."
  exit 1
fi

echo "==> 1a) Sourcing ROS from: $ROS_SETUP"
# Ensure nounset is on for *our* script, but off while sourcing ROS
set -u
safe_source "$ROS_SETUP"

# Optionally source your workspace if present
if [[ -f "$PROJECT_DIR/install/setup.bash" ]]; then
  echo "==> 1b) Sourcing local workspace"
  safe_source "$PROJECT_DIR/install/setup.bash"
elif [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  echo "==> 1b) Sourcing $HOME/ros2_ws"
  safe_source "$HOME/ros2_ws/install/setup.bash"
fi

echo "==> 2) Verifying system packages (venv + common ROS python deps)"
PKGS=(python3-venv python3-yaml python3-numpy python3-packaging python3-argcomplete)
MISSING=()
for p in "${PKGS[@]}"; do
  if ! dpkg -s "$p" >/dev/null 2>&1; then
    MISSING+=("$p")
  fi
done
if (( ${#MISSING[@]} )); then
  echo "Installing: ${MISSING[*]}"
  sudo apt update
  sudo apt install -y "${MISSING[@]}"
else
  echo "All required system packages already present."
fi

echo "==> 3) Creating/activating virtualenv"
if [[ ! -d "$VENV_DIR" ]]; then
  python3 -m venv "$VENV_DIR"
fi
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

echo "==> 4) Ensuring requirements.txt exists"
if [[ ! -f "$REQS_FILE" ]]; then
  cat > "$REQS_FILE" <<'EOF'
# Core ROS Python deps (keep versions flexible for ROS compatibility)
numpy
pyyaml
packaging
argcomplete
EOF
  echo "Created $REQS_FILE"
fi

echo "==> 5) Installing Python requirements into venv"
pip install --upgrade pip
pip install -r "$REQS_FILE"

echo "==> 6) Sanity-check: Python can import rclpy"
python - <<'PY'
import sys
try:
    import rclpy
except Exception as e:
    print("ERROR: Failed to import rclpy. Make sure ROS 2 is sourced (setup.bash) in THIS shell.")
    print("Detail:", e)
    sys.exit(2)
print("OK: rclpy import succeeded.")
PY

if [[ "${XDG_SESSION_TYPE:-}" != "x11" ]]; then
  echo "NOTE: XDG_SESSION_TYPE=${XDG_SESSION_TYPE:-unknown}. Gazebo/keyboard UI typically works best on X11."
fi

echo "==> 7) Running controller: $PY_SCRIPT"
if [[ ! -f "$PY_SCRIPT" ]]; then
  echo "ERROR: $PY_SCRIPT not found next to this script."
  echo "Make sure the Fix B node is saved as: $PY_SCRIPT"
  exit 1
fi

python "$PY_SCRIPT"
