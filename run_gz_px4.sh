#!/usr/bin/env bash
set -euo pipefail

# export ROS_LOCALHOST_ONLY=1

# --- Paths --- 
PX4_DIR="$HOME/PX4-Autopilot"
ENV_DIR="$HOME/Gz-PX4-Environment"

WORLD_NAME="forest"
PX4_MODEL="test"
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
PX4_GZ_PATH="${PX4_DIR}/Tools/simulation/gz"

export GZ_SIM_RESOURCE_PATH="${ENV_DIR}/worlds:${ENV_DIR}/models:${PX4_GZ_PATH}/models:${PX4_GZ_PATH}/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SERVER_CONFIG_PATH="${PX4_GZ_PATH}/server.config"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_GZ_PATH}/plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

GZ_PID=""

# Clean-up function
cleanup() {
  echo
  echo "[run_gz_px4] Cleanup: stopping Gazebo and PX4..."
  if [[ -n "${GZ_PID}" ]] && kill -0 "${GZ_PID}" 2>/dev/null; then
    echo "[run_gz_px4] Stopping Gazebo (PID=${GZ_PID})..."
    kill "${GZ_PID}" 2>/dev/null || true
    # Optional: wait for gz to exit
    wait "${GZ_PID}" 2>/dev/null || true
  fi
  echo "[run_gz_px4] Cleanup done."
}

# Define a trap: Run cleanup if CTRL+C (SIGINT)
trap 'cleanup; exit 0' INT TERM

# --- Start GZ SIM
# Prefer local env world, fall back to PX4's worlds
if [[ -f "${ENV_DIR}/worlds/${WORLD_NAME}.sdf" ]]; then
  WORLD_FILE="${ENV_DIR}/worlds/${WORLD_NAME}.sdf"
elif [[ -f "${PX4_GZ_PATH}/worlds/${WORLD_NAME}.sdf" ]]; then
  WORLD_FILE="${PX4_GZ_PATH}/worlds/${WORLD_NAME}.sdf"
else
  echo "[run_gz_px4] ERROR: world file not found in ENV_DIR or PX4: ${WORLD_NAME}.sdf" >&2
  exit 1
fi

GZ_CMD=(gz sim -r "${WORLD_FILE}")
if [[ "${HEADLESS}" -eq 1 ]]; then
  GZ_CMD+=(-s)
fi

echo "[run_gz_px4] Using PX4_DIR=${PX4_DIR}"
echo "[run_gz_px4] Using ENV_DIR=${ENV_DIR}"
echo "[run_gz_px4] Using GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"
echo "[run_gz_px4] World file: ${WORLD_FILE}"

echo "[run_gz_px4] Starting Gazebo: ${GZ_CMD[*]}"
setsid "${GZ_CMD[@]}" &
GZ_PID=$!

sleep 5

# --- Start Micro XRCE-DDS Agent (For uORB->ROS2 topics)
MicroXRCEAgent udp4 -p 8888 &

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
