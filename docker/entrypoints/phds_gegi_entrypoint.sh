#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/melodic/setup.bash"
source "/opt/phds_gegi_driver/devel/setup.bash"

exec "$@"
