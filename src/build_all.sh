#!/usr/bin/env bash
# =============================================================================
# AIR26 ROS 2 Workshop — one-shot build for the WHOLE workspace
# =============================================================================
# Resolves every package's ROS dependencies with rosdep, then colcon-builds all
# projects (01–07) at once. Meant as a "does everything compile on my machine?"
# smoke test for students who just cloned the repo.
#
#   cd ~/air26-ros2-ws
#   ./src/build_all.sh                 # rosdep + build everything
#   ./src/build_all.sh --no-rosdep     # skip rosdep (deps already installed)
#   ./src/build_all.sh --pip-extras    # ALSO pip-install the sim runtime deps
#   ./src/build_all.sh --packages "basics_py microbot_sim"   # build a subset
#   ./src/build_all.sh -h              # help
#
# NOTE: this script handles the ROS/colcon side. Simulator + LLM/VLA runtimes
# (MuJoCo, Ollama, SmolVLA venv, PlatformIO firmware) are NOT ROS packages —
# see src/INSTALL.md. `--pip-extras` covers the easy pip ones (MuJoCo + Stretch
# sim + Ollama client); Ollama, the SmolVLA venv, Webots and PlatformIO stay
# manual (documented in INSTALL.md) because they are large / need sudo / a GPU.
# =============================================================================
set -euo pipefail

# --- locate the workspace root (this script lives in <ws>/src/) --------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# --- options -----------------------------------------------------------------
RUN_ROSDEP=1
PIP_EXTRAS=0
PACKAGES=""
COLCON_EXTRA=(--symlink-install)

# print the leading comment header (skip shebang, stop at first non-comment line)
usage() { awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"; exit 0; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rosdep)   RUN_ROSDEP=0 ;;
    --pip-extras)  PIP_EXTRAS=1 ;;
    --packages)    PACKAGES="$2"; shift ;;
    -h|--help)     usage ;;
    *) echo "Unknown option: $1 (try -h)"; exit 1 ;;
  esac
  shift
done

say() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }

cd "${WS_ROOT}"

# --- 1. source ROS -----------------------------------------------------------
say "Sourcing ROS 2 ${ROS_DISTRO}"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# --- 2. rosdep: install all declared ROS dependencies ------------------------
if [[ "${RUN_ROSDEP}" == "1" ]]; then
  say "Resolving ROS dependencies with rosdep (from src/)"
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init || true
  fi
  rosdep update
  # -r = keep going past packages rosdep can't resolve (e.g. venv-only 07 bits)
  rosdep install --from-paths src --ignore-src -r -y
else
  say "Skipping rosdep (--no-rosdep)"
fi

# --- 3. optional: pip runtime deps for the simulators ------------------------
if [[ "${PIP_EXTRAS}" == "1" ]]; then
  say "Installing pip sim/runtime extras (MuJoCo, Stretch sim, Ollama client)"
  # Pinned to protect ROS Humble's numpy 1.24 (see src/INSTALL.md for the why).
  python3 -m pip install \
    "numpy==1.24.2" "opencv-python==4.10.0.84" "mujoco==3.2.6" "ros2-numpy==0.0.5" \
    "ollama"
  # Stretch MuJoCo sim is an editable install driven by uv (its build backend).
  python3 -m pip install uv
  python3 -m uv pip install --system -e \
    "${WS_ROOT}/src/04_stretch-hr-se3/upstream/stretch_mujoco"
fi

# --- 4. colcon build ---------------------------------------------------------
if [[ -n "${PACKAGES}" ]]; then
  say "Building selected packages: ${PACKAGES}"
  # shellcheck disable=SC2086
  colcon build "${COLCON_EXTRA[@]}" --packages-select ${PACKAGES}
else
  say "Building ALL packages in src/ (this can take several minutes)"
  colcon build "${COLCON_EXTRA[@]}"
fi

# --- 5. done -----------------------------------------------------------------
say "Build complete"
cat <<EOF

Next:
  source ${WS_ROOT}/install/setup.bash
  ros2 pkg list | grep -E 'basics|microbot|multibot|stretch_se3|perceptbot|llm_integration|vla_so101'

Runtime extras that are NOT built by colcon (see src/INSTALL.md):
  * MuJoCo / Stretch sim ...... rerun with --pip-extras, or follow INSTALL.md §3
  * Ollama server + models .... project 06  (systemd service + qwen3 models)
  * SmolVLA venv (torch) ...... project 07  (isolated /home/lsp/vla_venv)
  * Webots R2025a ............. project 05  (2 GB .deb)
  * PlatformIO firmware ....... projects 02 & 05  (ESP32, official installer)
EOF
