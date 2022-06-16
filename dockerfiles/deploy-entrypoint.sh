#!/bin/bash
set -e

# setup openspot environment
source "/openspot/install/setup.bash"

exec "$@"
