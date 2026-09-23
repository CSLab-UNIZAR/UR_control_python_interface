#!/usr/bin/env bash
# Starts the web control panel and opens it in the browser.
# Extra options are passed on, e.g.  ./run_webui.sh --host 192.168.0.210
cd "$(dirname "$0")" || exit 1
if [ ! -x .venv/bin/python ]; then
  echo "The virtual environment is missing: run ./setup.sh first." >&2
  exit 1
fi
exec .venv/bin/python -m webui "$@"
