#!/usr/bin/env bash
# Run the GeGi driver unit tests.
#
# These are pure-logic tests: NO detector, NO roscore, NO hardware. They only
# need the ROS python packages importable (the node modules do `import rospy`),
# so they are safe to run offline and in CI.
#
#   bash test/run_tests.sh          # all tests
#   bash test/run_tests.sh -v       # verbose
#
# Environment (all auto-detected, override if needed):
#   GEGI_WS  path to the catkin workspace holding this package (for the custom
#            radiation_detector_msgs / phds_gegi_driver.srv imports)
#   PYTHON   python interpreter to use (default: python2 if present, else python3)
set -e

cd "$(dirname "$0")/.."

# --- ROS: source whatever distro is installed (melodic, noetic, ...) ---------
if [ -z "$ROS_DISTRO" ]; then
    for setup in /opt/ros/*/setup.bash; do
        if [ -f "$setup" ]; then
            # shellcheck disable=SC1090
            source "$setup"
            break
        fi
    done
fi
if [ -z "$ROS_DISTRO" ]; then
    echo "WARNING: no ROS installation found under /opt/ros." >&2
    echo "         The node modules import rospy, so most tests will not load." >&2
fi

# --- Catkin workspace: needed for radiation_detector_msgs + the custom srv ---
for ws in "$GEGI_WS" "$HOME/gegi_ws" "$HOME/catkin_ws" "$(pwd)/../.."; do
    if [ -n "$ws" ] && [ -f "$ws/devel/setup.bash" ]; then
        # shellcheck disable=SC1090
        source "$ws/devel/setup.bash"
        echo "Using catkin workspace: $ws"
        break
    fi
done

# --- Interpreter ------------------------------------------------------------
PY="${PYTHON:-}"
if [ -z "$PY" ]; then
    if command -v python2 >/dev/null 2>&1; then PY=python2; else PY=python3; fi
fi

echo "Running GeGi driver unit tests (no hardware required) using $PY ..."
"$PY" -m unittest discover -s test -p "test_*.py" "$@"
