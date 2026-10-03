#!/bin/sh
# Start the GroundSpeed dashboard. Extra args are passed through, e.g.
#   dashboard/run.sh --phone 172.20.10.1
#   dashboard/run.sh --sim            # also start the fake phone (local testing)
cd "$(dirname "$0")" || exit 1
PY=${PYTHON:-python3}
if [ "$1" = "--sim" ]; then
  shift
  $PY -m gsdash.sim --host 127.0.0.1 &
  SIM=$!
  trap 'kill $SIM 2>/dev/null' EXIT INT TERM
fi
$PY -m gsdash "$@"
