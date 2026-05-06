#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="${1:-$HOME/catkin_ws}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$WS_ROOT/src"
PKG_LINK="$SRC_DIR/MPC-D-CBF"

mkdir -p "$SRC_DIR"

if [ -L "$PKG_LINK" ]; then
  rm "$PKG_LINK"
elif [ -e "$PKG_LINK" ]; then
  echo "Path already exists and is not a symlink: $PKG_LINK" >&2
  echo "Remove or rename it manually, then rerun this script." >&2
  exit 1
fi

ln -s "$REPO_ROOT" "$PKG_LINK"

echo "Workspace prepared at: $WS_ROOT"
echo "Next steps:"
echo "  1. source /opt/ros/noetic/setup.bash"
echo "  2. cd $WS_ROOT"
echo "  3. catkin_make"
echo "  4. source $WS_ROOT/devel/setup.bash"
