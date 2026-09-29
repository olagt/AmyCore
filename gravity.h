// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
#pragma once
// Gravity torques from the URDF with Pinocchio (gravity.cpp). Plain arrays only, so the rest of AmyCore doesn't need
// Pinocchio's (heavy) headers.

// Joint order used here and by AmyCore's board/side numbering: head_pan, j0, j1, j2, j3, j4, j5, j6
const int GRAVITY_JOINTS = 8;

// load the robot model; returns false and writes a message into err on failure
bool gravityInit(const char *urdfPath, char *err, int errLen);
// q: joint angles [rad], tau: gravity torque each joint must produce to hold the pose [Nm] (both in the order above)
bool gravityCompute(const double q[GRAVITY_JOINTS], double tau[GRAVITY_JOINTS]);

// ---- hand (tool) kinematics, frame "right_hand" (BODY), arm base frame: x forward, y left, z up ----
// (own Pinocchio data: may run in another thread than gravityCompute, but not in two threads at once)
// position [m] and rotation matrix (row-major 3x3) of the hand for the pose q
bool handPose(const double q[GRAVITY_JOINTS], double pos[3], double rot[9]);
// joint velocities dq [rad/s] (head_pan entry always 0) that move the hand with linear velocity v [m/s] (base axes);
// keepOrientation: also hold the hand's orientation (6D task), else position only (3D task). Damped least squares
// with damping lambda; returns the smallest singular value of the task Jacobian in sigmaMin (near 0 = singular pose)
bool handVelocityToJoints(const double q[GRAVITY_JOINTS], const double v[3], bool keepOrientation, double lambda,
                          double dq[GRAVITY_JOINTS], double *sigmaMin);
