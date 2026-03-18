#!/usr/bin/env bash

set -euo pipefail

ACTION="${1:-mount}"
DEVICE="${2:-/dev/sdg1}"
MOUNT_POINT="/mnt/external_realdata"

log() {
  echo "[external_realdata] $*"
}

usage() {
  cat <<EOF
Usage:
  $0 mount [device]
  $0 status
  $0 umount

Defaults:
  device: /dev/sdg1
  mount point: ${MOUNT_POINT}
EOF
}

is_mounted() {
  awk -v target="${MOUNT_POINT}" '$2 == target { found = 1 } END { exit(found ? 0 : 1) }' /proc/mounts
}

show_status() {
  if is_mounted; then
    awk -v target="${MOUNT_POINT}" '$2 == target { printf("mounted: source=%s target=%s fstype=%s options=%s\n", $1, $2, $3, $4) }' /proc/mounts
    if [ -w "${MOUNT_POINT}" ]; then
      log "writable: yes"
    else
      log "writable: no"
      return 1
    fi
  else
    log "not mounted: ${MOUNT_POINT}"
    return 1
  fi
}

ensure_mount_point() {
  if [ -d "${MOUNT_POINT}" ]; then
    return 0
  fi

  log "creating ${MOUNT_POINT}"
  if sudo mkdir -p "${MOUNT_POINT}" 2>/dev/null; then
    return 0
  fi

  log "/ is read-only, remounting / as rw temporarily"
  sudo mount -o rw,remount /
  trap 'sudo mount -o ro,remount /' EXIT
  sudo mkdir -p "${MOUNT_POINT}"
  sudo mount -o ro,remount /
  trap - EXIT
}

do_mount() {
  if [ ! -b "${DEVICE}" ]; then
    echo "block device not found: ${DEVICE}" >&2
    exit 1
  fi

  ensure_mount_point

  if is_mounted; then
    log "${MOUNT_POINT} is already mounted"
    show_status
    return 0
  fi

  log "mounting ${DEVICE} -> ${MOUNT_POINT}"
  sudo mount "${DEVICE}" "${MOUNT_POINT}"
  show_status
}

do_umount() {
  if is_mounted; then
    log "unmounting ${MOUNT_POINT}"
    sudo umount "${MOUNT_POINT}"
  else
    log "${MOUNT_POINT} is not mounted"
  fi
}

case "${ACTION}" in
  mount)
    do_mount
    ;;
  status)
    show_status
    ;;
  umount)
    do_umount
    ;;
  *)
    usage
    exit 1
    ;;
esac
