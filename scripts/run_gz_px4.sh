#!/usr/bin/env bash
set -euo pipefail

# --- Paths --- 
PX4_DIR="$HOME/PX4-Autopilot"
SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
ENV_DIR_DEFAULT="$(cd -- "${SCRIPT_DIR}/.." >/dev/null 2>&1 && pwd)"
ENV_DIR="${ENV_DIR:-$ENV_DIR_DEFAULT}" # Allows override via env (not used atm)
PX4_SIM_PATH="${PX4_DIR}/Tools/simulation/gz"

WORLD_NAME="walls"
PX4_MODEL="x500_lidar"
HEADLESS=0

# --- Argument Parsing --- 
while [[ $# -gt 0 ]]; do
  case "$1" in
    -w|--world)
      WORLD_NAME="$2"
      shift 2
      ;;
    --headless)
      HEADLESS=1
      shift
      ;;
    -h|--help)
      echo "Usage: $0 [--world NAME] [--headless]"
      echo "  --world NAME   Use worlds/NAME.sdf (default: default)"
      echo "  --headless     Run gz sim without GUI"
      exit 0
      ;;
    *)
      echo "Unknown argument: $1"
      exit 1
      ;;
  esac
done

if [[ ! -x "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]]; then
  echo "[run_gz_px4] ERROR: PX4 SITL binary not found."
  echo "Expected at: ${PX4_DIR}/build/px4_sitl_default/bin/px4"
  echo "Did you run:  cd ${PX4_DIR} && make px4_sitl_default ?"
  exit 1
fi

# --- GZ Resource Path
export GZ_SIM_RESOURCE_PATH="${ENV_DIR}/worlds:${ENV_DIR}/models:${PX4_SIM_PATH}/models:${PX4_SIM_PATH}/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SERVER_CONFIG_PATH="${PX4_SIM_PATH}/server.config"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_SIM_PATH}/plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

# --- PIDs to track for cleanup
GZ_PID=""
AGENT_PID=""
BRIDGE_PID=""
TF_PID=""

# Clean-up function
cleanup() {
  echo
  echo "[run_gz_px4] Cleanup: stopping Gazebo, bridge, agent, and PX4..."

  # Stop process function
  stop_proc() { 
    local name=$1
    local pid_var=$2
    local pid=${!pid_var-}   # value of the variable whose name is in pid_var (or empty if unset)

    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      echo "[run_gz_px4] Stopping ${name} (PID=${pid})..."
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  }

  stop_proc "ROS 2 bridge"        BRIDGE_PID
  stop_proc "TF Publisher"        TF_PID
  stop_proc "Micro XRCE-DDS Agent" AGENT_PID
  stop_proc "Gazebo"              GZ_PID

  echo "[run_gz_px4] Cleanup done."
}

# Define a trap: Run cleanup if CTRL+C (SIGINT)
trap 'cleanup; exit 0' INT TERM

# --- Start GZ SIM
# Prefer local env world, fall back to PX4's worlds
if [[ -f "${ENV_DIR}/worlds/${WORLD_NAME}.sdf" ]]; then
  WORLD_FILE="${ENV_DIR}/worlds/${WORLD_NAME}.sdf"
elif [[ -f "${PX4_SIM_PATH}/worlds/${WORLD_NAME}.sdf" ]]; then
  WORLD_FILE="${PX4_SIM_PATH}/worlds/${WORLD_NAME}.sdf"
else
  echo "[run_gz_px4] ERROR: world file not found in ENV_DIR or PX4: ${WORLD_NAME}.sdf" >&2
  exit 1
fi

GZ_CMD=(gz sim -r "${WORLD_FILE}")
if [[ "${HEADLESS}" -eq 1 ]]; then
  GZ_CMD+=(-s)
fi

echo "[run_gz_px4] Starting Gazebo: ${GZ_CMD[*]}"
setsid "${GZ_CMD[@]}" &
GZ_PID=$!
sleep 5

# --- Start ROS 2 bridge and TF publisher
echo "[run_gz_px4] Starting ROS 2 bridge..."
if [[ ! -f "${ENV_DIR}/install/setup.bash" ]]; then
  echo "[run_gz_px4] ERROR: ${ENV_DIR}/install/setup.bash not found. Did you colcon build?" >&2
  exit 1
fi
cd "${ENV_DIR}"
# Relax strict mode while sourcing ROS2 env
set +e
set +u
source install/setup.bash
set -e
set -u
ros2 launch gz_px4 bridge.launch.py &
BRIDGE_PID=$!
ros2 launch gz_px4 tf.launch.py &
TF_PID=$!

# --- Start Micro XRCE-DDS Agent (For uORB->ROS2 topics)
echo "[run_gz_px4] Starting Micro XRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
sleep 5

# --- Start PX4 in Standalone (Will identify the GZ SIM instance)
cd "${PX4_DIR}"
echo "[run_gz_px4] Starting PX4 SITL with model: ${PX4_MODEL}"
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL="${PX4_MODEL}" \
PX4_GZ_WORLD="${WORLD_NAME}" \
./build/px4_sitl_default/bin/px4

# If PX4 exits normally, still cleanup Gazebo
cleanup

echo "EOF!"
