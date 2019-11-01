#!/bin/bash
# cp setting files
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR/../src/system_monitor/utils"
if [ ! -e "config.py" ]; then
    cp config-template.py config.py
fi
