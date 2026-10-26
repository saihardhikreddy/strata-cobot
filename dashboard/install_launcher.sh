#!/usr/bin/env bash
# Add "Strata Cobot" to the Ubuntu app menu, so the dashboard starts with one click.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HOME/.local/share/applications"
cat > "$HOME/.local/share/applications/strata-cobot-dashboard.desktop" <<DESKTOP
[Desktop Entry]
Type=Application
Name=Strata Cobot
Comment=Run and watch the ore-sorting arm and the mobile robot
Exec=$HERE/run_dashboard.sh
Icon=$HERE/static/icon.svg
Terminal=true
Categories=Development;Science;
DESKTOP
echo "Added 'Strata Cobot' to the app menu (it opens a terminal for the log and the dashboard in your browser)."
