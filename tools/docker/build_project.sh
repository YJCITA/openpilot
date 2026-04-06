#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="${SP_DOCKER_IMAGE:-openpilot:latest}"
WORKTREE="/work/openpilot"
IMAGE_VENV="/home/batman/openpilot/.venv"
HOME_DIR="${SP_DOCKER_HOME_DIR:-${ROOT}/.docker-cache/home}"
SCONS_CACHE_DIR="${SP_SCONS_CACHE_DIR:-${ROOT}/.docker-cache/scons-cache}"
JOBS="${JOBS:-$(nproc)}"

mkdir -p "$HOME_DIR" "$SCONS_CACHE_DIR"

quoted_args=()
for arg in "$@"; do
  printf -v quoted '%q' "$arg"
  quoted_args+=("$quoted")
done

cmd="scons -j${JOBS}"
if [[ ${#quoted_args[@]} -gt 0 ]]; then
  cmd+=" ${quoted_args[*]}"
fi

if [[ "$cmd" != *"cache_dir="* ]]; then
  cmd+=" cache_dir=/tmp/scons_cache"
fi

echo "[docker] repo: $ROOT"
echo "[docker] image: $IMAGE"
echo "[docker] command: $cmd"

docker run --rm -t \
  --user "$(id -u):$(id -g)" \
  --env HOME=/tmp/codex-home \
  --env VIRTUAL_ENV="${IMAGE_VENV}" \
  --env PATH="/home/batman/.local/bin:${IMAGE_VENV}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  --env PYTHONPATH="${WORKTREE}" \
  --volume "${ROOT}:${WORKTREE}" \
  --volume "${HOME_DIR}:/tmp/codex-home" \
  --volume "${SCONS_CACHE_DIR}:/tmp/scons_cache" \
  --workdir "${WORKTREE}" \
  "$IMAGE" \
  bash -c "$cmd"
