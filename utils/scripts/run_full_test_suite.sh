#!/bin/bash

set -e

./fetch_and_upload_firmware.sh

docker exec -it open-mower /openmower_entrypoint.sh /bin/bash -c "rosrun mower_logic hardware_test"
docker exec -it open-mower /openmower_entrypoint.sh /bin/bash -c "python3 utils/scripts/integration_test.py"
