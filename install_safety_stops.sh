#!/usr/bin/env bash
# Install or remove the soft-reset and hard-damp gate for this clone.
set -eo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
unit_name=g1-safety-stops.service
unit_dir="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
unit_path="$unit_dir/$unit_name"
launcher="$repo_dir/tests/manual/run_safety_stops.sh"

installed() {
    [[ -f "$unit_path" ]] && return 0
    systemctl --user is-enabled --quiet "$unit_name" 2>/dev/null
}

install_stops() {
    if [[ ! -x "$launcher" ]]; then
        chmod +x "$launcher"
    fi
    mkdir -p "$unit_dir"
    cat > "$unit_path" << EOF
[Unit]
Description=G1 soft reset and hard damp
After=network.target
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Type=notify
NotifyAccess=all
WorkingDirectory=$repo_dir
ExecStart=$launcher
TimeoutStartSec=180
TimeoutStopSec=15
KillSignal=SIGTERM
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
EOF
    systemctl --user daemon-reload
    systemctl --user enable "$unit_name"
    systemctl --user restart "$unit_name"
    if ! loginctl show-user "$USER" -p Linger | grep -q '=yes'; then
        if ! loginctl enable-linger "$USER"; then
            echo "The service is running, but boot start still needs: sudo loginctl enable-linger $USER" >&2
            exit 1
        fi
    fi
    systemctl --user --no-pager --full status "$unit_name"
}

uninstall_stops() {
    systemctl --user disable --now "$unit_name" 2>/dev/null || true
    rm -f "$unit_path"
    systemctl --user daemon-reload
    systemctl --user reset-failed "$unit_name" 2>/dev/null || true
    echo "Safety stops are stopped and will not start on boot."
}

choose() {
    if installed; then
        echo "Safety stops are installed."
        echo "1) uninstall and stop them"
        echo "2) leave them installed"
    else
        echo "Safety stops are not installed."
        echo "1) install, start them, and start them on every boot"
        echo "2) leave them uninstalled"
    fi
    read -r -p "Choose 1 or 2: " choice
    case "$choice" in
        1)
            if installed; then
                uninstall_stops
            else
                install_stops
            fi
            ;;
        2) ;;
        *)
            echo "Choose 1 or 2." >&2
            exit 1
            ;;
    esac
}

case "${1:-}" in
    install) install_stops ;;
    uninstall) uninstall_stops ;;
    "") choose ;;
    *)
        echo "Usage: $0 [install|uninstall]" >&2
        exit 1
        ;;
esac
