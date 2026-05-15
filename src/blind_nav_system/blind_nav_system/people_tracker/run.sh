#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/people_tracker/bin/activate"

echo "[run.sh] 경로: $SCRIPT_DIR"
echo "[run.sh] Python: $(which python)"
echo "[run.sh] 실행: python main.py $@"
echo ""

python main.py "$@"
