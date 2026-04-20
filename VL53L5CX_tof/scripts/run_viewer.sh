#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="${REPO_ROOT}/.venv"

if [ ! -x "${VENV_DIR}/bin/python" ]; then
  echo "Viewer virtual environment is missing." >&2
  echo "Run ${REPO_ROOT}/scripts/setup_viewer_env.sh first." >&2
  exit 1
fi

cd "${REPO_ROOT}"
exec "${VENV_DIR}/bin/python" -m viewer "$@"
