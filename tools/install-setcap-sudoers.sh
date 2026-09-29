#!/bin/sh
# Copyright (C) 2023-2026 Ola Gatner
# SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
# One-time setup per machine: lets the build re-apply AmyCore's file capabilities after every link without a password.
# Relinking build-robot/AmyCore drops its capabilities; without cap_net_raw it can't open the raw Ethernet socket
# ("Raw Ethsocket: Operation not permitted", "SIOCGIFINDEX: Bad file descriptor"). CMakeLists.txt runs, after each
# link, exactly the command allowed here: sudo -n /usr/sbin/setcap <CAPS> <repo>/build-robot/AmyCore
# The rule allows only that one command with those exact arguments. Remove it with: sudo rm /etc/sudoers.d/amycore-setcap
set -e
REPO=$(cd "$(dirname "$0")/.." && pwd)
BIN="$REPO/build-robot/AmyCore"
USER_NAME=$(id -un)
RULE_FILE=/etc/sudoers.d/amycore-setcap
TMP=$(mktemp)
# commas inside a sudoers command argument must be escaped
printf '%s ALL=(root) NOPASSWD: /usr/sbin/setcap cap_net_raw\\,cap_net_admin\\,cap_sys_nice\\,cap_ipc_lock+ep %s\n' "$USER_NAME" "$BIN" > "$TMP"
echo "rule: $(cat "$TMP")"
sudo visudo -cf "$TMP"
sudo install -m 0440 -o root -g root "$TMP" "$RULE_FILE"
rm -f "$TMP"
echo "installed $RULE_FILE"
[ -f "$BIN" ] && sudo -n /usr/sbin/setcap cap_net_raw,cap_net_admin,cap_sys_nice,cap_ipc_lock+ep "$BIN" && getcap "$BIN" || true
