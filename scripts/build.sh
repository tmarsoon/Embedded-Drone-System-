#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
make -C native -j4 all
"${MVN:-mvn}" -B -C package
