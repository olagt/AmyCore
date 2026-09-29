// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
#pragma once
// Version and build info; the definitions are generated at build time (cmake/build_info.cmake -> build_info.cpp).
// The version number itself is set in CMakeLists.txt (project(AmyCore VERSION ...)).
extern const char *amyVersion;       // "0.1.0"
extern const char *amyGitRev;        // git describe --always --dirty, e.g. "c448919-dirty"
extern const char *amyBuildDate;     // "2026-09-26 14:03" (local time of the build)
extern const char *amyVersionLine;   // "AmyCore v0.1.0 (c448919-dirty, built 2026-09-26 14:03)"
