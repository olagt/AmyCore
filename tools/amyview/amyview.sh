#!/bin/sh
# Copyright (C) 2023-2026 Ola Gatner
# SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
# AmyView launcher: Pinocchio from robotpkg (/opt/openrobots), meshcat from the viewer's own venv (setup: see README)
HERE=$(dirname "$(readlink -f "$0")")
export PYTHONPATH=/opt/openrobots/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}
export LD_LIBRARY_PATH=/opt/openrobots/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
exec "$HERE/venv/bin/python" "$HERE/amyview.py" "$@"
