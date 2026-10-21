#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
exec "${MVN:-mvn}" -C javafx:run "$@"
