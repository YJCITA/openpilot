#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="${SP_DOCKER_IMAGE:-openpilot:latest}"
WORKTREE="/work/openpilot"
IMAGE_VENV="/home/batman/openpilot/.venv"
HOME_DIR="${SP_DOCKER_HOME_DIR:-${ROOT}/.docker-cache/home}"
SCONS_CACHE_DIR="${SP_SCONS_CACHE_DIR:-${ROOT}/.docker-cache/scons-cache}"
UI_TIMEOUT="${SP_UI_TIMEOUT:-10}"

HEADLESS=0
SMOKE_TEST=0

mkdir -p "$HOME_DIR" "$SCONS_CACHE_DIR"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)
      HEADLESS=1
      ;;
    --smoke-test)
      SMOKE_TEST=1
      ;;
    *)
      echo "Unknown option: $1" >&2
      echo "Usage: bash tools/docker/run_ui.sh [--headless] [--smoke-test]" >&2
      exit 1
      ;;
  esac
  shift
done

docker_args=(
  run --rm -t
  --security-opt apparmor=unconfined
  --env HOME=/tmp/codex-home
  --env VIRTUAL_ENV="${IMAGE_VENV}"
  --env PATH="/home/batman/.local/bin:${IMAGE_VENV}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
  --env PYTHONPATH="${WORKTREE}"
  --env QT_X11_NO_MITSHM=1
  --volume "${ROOT}:${WORKTREE}"
  --volume "${HOME_DIR}:/tmp/codex-home"
  --volume "${SCONS_CACHE_DIR}:/tmp/scons_cache"
  --workdir "${WORKTREE}"
)

if [[ -S /run/dbus/system_bus_socket ]]; then
  docker_args+=(--volume /run/dbus/system_bus_socket:/run/dbus/system_bus_socket)
fi

if [[ -d /dev/dri ]]; then
  docker_args+=(--device /dev/dri:/dev/dri)
fi

run_cmd="python selfdrive/ui/ui.py"

if [[ "$HEADLESS" -eq 1 ]]; then
  run_cmd="xvfb-run -a -s '-screen 0 2160x1080x24' ${run_cmd}"
else
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "DISPLAY is not set. Use --headless when running outside a graphical desktop." >&2
    exit 1
  fi

  docker_args+=(--env "DISPLAY=${DISPLAY}")

  if [[ -S /tmp/.X11-unix/X0 || -d /tmp/.X11-unix ]]; then
    docker_args+=(--volume /tmp/.X11-unix:/tmp/.X11-unix)
  fi

  if [[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY}" ]]; then
    docker_args+=(--env XAUTHORITY=/tmp/.Xauthority)
    docker_args+=(--volume "${XAUTHORITY}:/tmp/.Xauthority:ro")
  fi
fi

if [[ "$SMOKE_TEST" -eq 1 ]]; then
  run_cmd="timeout ${UI_TIMEOUT}s ${run_cmd}"
fi

cmd="${run_cmd}"

echo "[docker] repo: $ROOT"
echo "[docker] image: $IMAGE"
echo "[docker] command: $cmd"

docker "${docker_args[@]}" "$IMAGE" bash -c "$cmd"
