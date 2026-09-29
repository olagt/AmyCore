// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
#pragma once
// Self-collision and table check with Pinocchio + coal (collision.cpp), for coordinated moves ("Go to home posture").
// Geometry: the real link meshes of the official sawyer_description package (the URDF's visual meshes); Intera's own
// collision primitives are too coarse (their 18 cm head sphere flags Intera's neutral and shipping poses as colliding).
// Joint order as in gravity.h: head_pan, j0 .. j6. Plain arrays only across this interface.
#include "gravity.h"

// urdf: the robot model, packageDir: folder that contains sawyer_description/ (meshes), margin: minimum distance [m]
// between checked links; table: add the plane z = tableZ [m] (arm base frame, z up) against the moving links
bool collisionInit(const char *urdf, const char *packageDir, double margin, bool table, double tableZ, char *err, int errLen);
// add a fixed obstacle (arm base frame, metres), checked against every moving link (j0 .. j6 and the head):
// type 0 box (dims = size x y z), 1 cylinder with vertical axis (dims = radius, length), 2 sphere (dims = radius).
// Call after collisionInit and before collisionSetPairMargin / collisionDisableLink.
bool collisionAddObstacle(const char *name, int type, const double center[3], const double dims[3], char *err, int errLen);
// set the margin [m] of one link pair (e.g. parts built to pass close to each other); false if the pair isn't checked
bool collisionSetPairMargin(const char *link1, const char *link2, double margin);
// stop checking every pair that involves this link (e.g. a joint whose angle is unknown); false if no pair has it
bool collisionDisableLink(const char *link);
// true if the pose is collision-free (with the margin); otherwise false and the colliding pairs in info.
// Not thread-safe: call from one thread only.
bool collisionFree(const double q[GRAVITY_JOINTS], char *info, int infoLen);
