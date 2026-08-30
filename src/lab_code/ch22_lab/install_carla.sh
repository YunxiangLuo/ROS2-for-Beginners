#!/usr/bin/env bash
# Install the course CARLA server, Python API, and pinned ROS 2 bridge.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COURSE_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

if [[ ! -f "${COURSE_ROOT}/setup_course.sh" ]]; then
  printf '[FAIL] Course installer not found: %s/setup_course.sh\n' "${COURSE_ROOT}" >&2
  exit 1
fi

exec bash "${COURSE_ROOT}/setup_course.sh" --with-carla "$@"
