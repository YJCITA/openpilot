#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE_NAME="${SP_DOCKER_IMAGE:-openpilot}"
UV_TIMEOUT="${SP_UV_HTTP_TIMEOUT:-300}"
UV_SYNC_ARGS="${SP_DOCKER_UV_SYNC_ARGS:---frozen}"
CACHE_DIR="${ROOT}/.docker-cache"
DEPS_MIRROR="${CACHE_DIR}/dependencies.git"
WHEEL_DIR="${CACHE_DIR}/wheels"
GCC_WHEEL="${WHEEL_DIR}/gcc_arm_none_eabi-13.2.1-py3-none-linux_x86_64.whl"
GCC_WHEEL_URL="https://github.com/commaai/dependencies/releases/download/gcc-arm-none-eabi/v13.2.1/gcc_arm_none_eabi-13.2.1-py3-none-linux_x86_64.whl"
FFMPEG_WHEEL="${WHEEL_DIR}/ffmpeg-7.1.0-py3-none-linux_x86_64.whl"
FFMPEG_WHEEL_URL="https://github.com/commaai/dependencies/releases/download/ffmpeg/v7.1.0/ffmpeg-7.1.0-py3-none-linux_x86_64.whl"
DEPS_REPO_URL="https://github.com/commaai/dependencies.git"
DEPS_REPO_COMMIT="9777ee38aa5ca9439843125392af38ed1262e500"

retry() {
  local attempts=$1
  shift
  local try=1
  while (( try <= attempts )); do
    if "$@"; then
      return 0
    fi
    if (( try < attempts )); then
      echo "[cache] attempt ${try}/${attempts} failed, retrying in 5s"
      sleep 5
    fi
    ((try++))
  done
  return 1
}

ensure_dependencies_mirror() {
  mkdir -p "$CACHE_DIR"
  if [[ -d "$DEPS_MIRROR/objects" ]]; then
    echo "[cache] refreshing dependencies mirror"
    if ! retry 5 git -c http.version=HTTP/1.1 --git-dir "$DEPS_MIRROR" fetch --prune origin '+refs/*:refs/*'; then
      echo "[cache] warning: could not refresh dependencies mirror, using existing local mirror"
    fi
  else
    rm -rf "$DEPS_MIRROR"
    echo "[cache] cloning dependencies mirror"
    retry 5 git -c http.version=HTTP/1.1 clone --mirror "$DEPS_REPO_URL" "$DEPS_MIRROR"
  fi

  if git --git-dir "$DEPS_MIRROR" cat-file -e "${DEPS_REPO_COMMIT}^{commit}" 2>/dev/null; then
    echo "[cache] pinned dependencies commit already cached"
  else
    echo "[cache] fetching pinned dependencies commit ${DEPS_REPO_COMMIT}"
    retry 5 git -c http.version=HTTP/1.1 --git-dir "$DEPS_MIRROR" fetch origin "+${DEPS_REPO_COMMIT}:refs/commit/${DEPS_REPO_COMMIT}"
  fi
}

remote_size() {
  curl --http1.1 -fsIL "$1" | awk 'tolower($1) == "content-length:" {print $2}' | tr -d '\r' | tail -n1
}

ensure_cached_wheel() {
  local wheel_path="$1"
  local wheel_url="$2"
  local wheel_name="$3"
  local expected_size=""

  mkdir -p "$WHEEL_DIR"
  expected_size="$(remote_size "$wheel_url" || true)"

  if [[ -n "$expected_size" && -f "$wheel_path" ]] && (( $(stat -c %s "$wheel_path") >= expected_size )); then
    echo "[cache] ${wheel_name} wheel already cached"
    return 0
  fi

  echo "[cache] downloading ${wheel_name} wheel"
  curl --http1.1 --retry 20 --retry-delay 5 -L -C - "$wheel_url" -o "$wheel_path"
}

ensure_dependencies_mirror
ensure_cached_wheel "$GCC_WHEEL" "$GCC_WHEEL_URL" "gcc-arm-none-eabi"
ensure_cached_wheel "$FFMPEG_WHEEL" "$FFMPEG_WHEEL_URL" "ffmpeg"

echo "[docker] repo: $ROOT"
echo "[docker] image: ${IMAGE_NAME}:latest"
echo "[docker] uv timeout: ${UV_TIMEOUT}s"
echo "[docker] uv sync args: ${UV_SYNC_ARGS}"
echo "[docker] cache dir: ${CACHE_DIR}"

DOCKER_BUILDKIT=0 docker build \
  --pull \
  --build-arg UV_HTTP_TIMEOUT="${UV_TIMEOUT}" \
  --build-arg UV_REQUEST_TIMEOUT="${UV_TIMEOUT}" \
  --build-arg UV_SYNC_ARGS="${UV_SYNC_ARGS}" \
  -t "${IMAGE_NAME}:latest" \
  -f "${ROOT}/Dockerfile.openpilot" \
  "${ROOT}"
