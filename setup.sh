#!/usr/bin/env bash
# Creates the project virtual environment (.venv) and installs requirements.txt.
#
#   ./setup.sh                      use Python 3.12, 3.13, 3.11, 3.10 or 3.14 (first found)
#   ./setup.sh --python python3.11  force an interpreter
#   ./setup.sh --recreate           delete .venv and build it again
#
# Needs python3 with its venv module (Ubuntu: sudo apt install python3-venv).
# For the Leap Motion and SpaceMouse system packages, run install.sh instead.
set -euo pipefail
cd "$(dirname "$0")"

SUPPORTED="3.12 3.13 3.11 3.10 3.14"   # in order of preference
PYTHON=""
RECREATE=0

fail() { printf '\nERROR: %s\n' "$1" >&2; exit 1; }
version_of() { "$1" -c 'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null; }
supported() { case " $SUPPORTED " in *" $1 "*) return 0 ;; *) return 1 ;; esac; }

while [ $# -gt 0 ]; do
  case "$1" in
    --python) PYTHON="${2:?--python needs an interpreter, e.g. python3.12}"; shift 2 ;;
    --recreate) RECREATE=1; shift ;;
    -h|--help) sed -n '2,9s/^# \{0,1\}//p' "$0"; exit 0 ;;
    *) fail "unknown option '$1' (see ./setup.sh --help)" ;;
  esac
done

# --- 1. Find a supported Python -------------------------------------------------------
if [ -z "$PYTHON" ]; then
  for v in $SUPPORTED; do
    if command -v "python$v" >/dev/null 2>&1; then PYTHON="python$v"; break; fi
  done
  if [ -z "$PYTHON" ] && command -v python3 >/dev/null 2>&1 && supported "$(version_of python3)"; then
    PYTHON=python3
  fi
fi
[ -n "$PYTHON" ] || fail "No supported Python found (need one of: $SUPPORTED).
  Ubuntu 22.04 / 24.04 / 26.04:  sudo apt install python3 python3-venv"
VERSION="$(version_of "$PYTHON")" || fail "'$PYTHON' does not run."
supported "$VERSION" || fail "'$PYTHON' is Python $VERSION; need one of: $SUPPORTED."
echo "Using Python $VERSION ($PYTHON)"

# --- 2. Create (or reuse) .venv -------------------------------------------------------------
VENV_PY=.venv/bin/python
if [ "$RECREATE" = 1 ] && [ -d .venv ]; then
  echo "Removing the existing .venv ..."
  rm -rf .venv
fi
if [ -x "$VENV_PY" ]; then
  VENV_VERSION="$(version_of "$VENV_PY")" || VENV_VERSION="?"
  supported "$VENV_VERSION" || fail "The existing .venv uses Python $VENV_VERSION. Run: ./setup.sh --recreate"
  echo "Reusing .venv (Python $VENV_VERSION)"
elif [ -d .venv ]; then
  fail "The existing .venv was not created on this system (e.g. it comes from Windows). Run: ./setup.sh --recreate"
else
  echo "Creating .venv ..."
  if ! "$PYTHON" -m venv .venv; then
    rm -rf .venv
    fail "Could not create the virtual environment. On Ubuntu/Debian install the venv module:
  sudo apt install python$VERSION-venv"
  fi
fi

# --- 3. Install the requirements --------------------------------------------------------
echo "Installing requirements (this can take a few minutes the first time) ..."
"$VENV_PY" -m pip install --upgrade pip --disable-pip-version-check -q \
  || fail "Upgrading pip failed (check the internet connection / proxy)."
"$VENV_PY" -m pip install -r requirements.txt --disable-pip-version-check \
  || fail "Installing requirements.txt failed."

# --- 4. Check that the framework and the panel import ---------------------------------------
"$VENV_PY" -c "import core.ur_control, webui.robot, webui.server; print('Import check: OK')" \
  || fail "The packages installed, but UR_CONTROL does not import (see the error above)."

cat <<'EOF'

Setup complete.
  Web panel:     ./run_webui.sh   (or  .venv/bin/python -m webui)
  Activate venv: source .venv/bin/activate
EOF
