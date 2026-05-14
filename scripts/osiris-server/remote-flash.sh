#!/bin/bash
#
# These are the commands for flashing remotely build firmware onto your computer's usb port
#

set -Eeuo pipefail

HOST="${1:-${FLASH_HOST:-}}"
if [ -z "$HOST" ]; then
	echo "Usage: $0 <host> (or set FLASH_HOST)"
	exit 1
fi

while IFS= read -r line; do
    case "$line" in
        SYSTEM_PATH=*) ;;
        PATH=*)        export "PATH=${line#PATH=}:$PATH" ;;
        *=*)           export "$line" ;;
    esac
done < <(/opt/esp/tools/.espressif/tools/activate_idf_v6.0.1.sh -e)

IDF="$IDF_PATH/tools/idf.py"

"$IDF" -p "rfc2217://${HOST}:4000?ign_set_control" -b 460800 flash
"$IDF" -p "rfc2217://${HOST}:4000?ign_set_control" monitor
