#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p runs
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-$PWD/runs/cache}"
export MESA_SHADER_CACHE_DIR="${MESA_SHADER_CACHE_DIR:-$PWD/runs/mesa-cache}"
export LIBGL_ALWAYS_SOFTWARE=true
exec xvfb-run -a -s '-screen 0 1440x1000x24 -nolisten tcp' \
    "${MVN:-mvn}" -B -C -Dprism.order=es2 -Dprism.forceGPU=true \
    "-Dapp.args=--smoke --smoke-output=$PWD/runs/ui-smoke.png" javafx:run
