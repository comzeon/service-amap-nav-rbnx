#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# amap_nav build: uv venv + sync + rbnx codegen (--mcp, 包内契约合并).
set -euo pipefail
: "${UV_INDEX_URL:=https://pypi.tuna.tsinghua.edu.cn/simple}"
: "${PIP_INDEX_URL:=https://pypi.tuna.tsinghua.edu.cn/simple}"
export UV_INDEX_URL PIP_INDEX_URL
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
BUILD="rbnx-build"
VENV="$BUILD/venv"
mkdir -p "$BUILD/data"
command -v uv >/dev/null 2>&1 || { echo "error: uv not found" >&2; exit 1; }
[[ -d "$VENV" ]] || uv venv "$VENV"
VIRTUAL_ENV="$PKG/$VENV" uv sync --active --no-managed-python
rbnx codegen -p "$PKG" --mcp
echo "[build] done."
