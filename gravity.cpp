// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
// Gravity torques with Pinocchio (see gravity.h).
#include "gravity.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <Eigen/SVD>
#include <cstdio>
#include <exception>
#include <memory>

static_assert(_GLIBCXX_USE_CXX11_ABI == 1, "Pinocchio from robotpkg is built with the default (new) C++ string ABI");

namespace {
std::unique_ptr<pinocchio::Model> model;
std::unique_ptr<pinocchio::Data>  data;      // gravityCompute (comm thread)
std::unique_ptr<pinocchio::Data>  handData;  // handPose / handVelocityToJoints (Cartesian jog thread): separate, no race
int qIdx[GRAVITY_JOINTS], vIdx[GRAVITY_JOINTS];     // position of each of our joints in Pinocchio's q / v vectors
pinocchio::FrameIndex handFrame = 0;
const char *urdfNames[GRAVITY_JOINTS] = {"head_pan", "right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};
}

bool gravityInit(const char *urdfPath, char *err, int errLen)
{
    try
    {
        auto m = std::make_unique<pinocchio::Model>();
        pinocchio::urdf::buildModel(urdfPath, *m);
        for (int i = 0; i < GRAVITY_JOINTS; i++)
        {
            if (!m->existJointName(urdfNames[i])) { snprintf(err, errLen, "joint %s not in %s", urdfNames[i], urdfPath); return false; }
            auto id = m->getJointId(urdfNames[i]);
            qIdx[i] = m->joints[id].idx_q();
            vIdx[i] = m->joints[id].idx_v();
        }
        if (!m->existFrame("right_hand", pinocchio::BODY)) { snprintf(err, errLen, "frame right_hand not in %s", urdfPath); return false; }
        handFrame = m->getFrameId("right_hand", pinocchio::BODY);
        data = std::make_unique<pinocchio::Data>(*m);
        handData = std::make_unique<pinocchio::Data>(*m);
        model = std::move(m);
        return true;
    }
    catch (const std::exception &e) { snprintf(err, errLen, "%s", e.what()); return false; }
}

bool gravityCompute(const double q[GRAVITY_JOINTS], double tau[GRAVITY_JOINTS])
{
    if (!model) return false;
    Eigen::VectorXd qv = Eigen::VectorXd::Zero(model->nq);
    for (int i = 0; i < GRAVITY_JOINTS; i++) qv[qIdx[i]] = q[i];
    const Eigen::VectorXd &g = pinocchio::computeGeneralizedGravity(*model, *data, qv);
    for (int i = 0; i < GRAVITY_JOINTS; i++) tau[i] = g[vIdx[i]];
    return true;
}

static Eigen::VectorXd toQ(const double q[GRAVITY_JOINTS])
{
    Eigen::VectorXd qv = Eigen::VectorXd::Zero(model->nq);
    for (int i = 0; i < GRAVITY_JOINTS; i++) qv[qIdx[i]] = q[i];
    return qv;
}

bool handPose(const double q[GRAVITY_JOINTS], double pos[3], double rot[9])
{
    if (!model) return false;
    pinocchio::framesForwardKinematics(*model, *handData, toQ(q));
    const auto &M = handData->oMf[handFrame];
    for (int i = 0; i < 3; i++) { pos[i] = M.translation()[i]; for (int k = 0; k < 3; k++) rot[3*i+k] = M.rotation()(i, k); }
    return true;
}

bool handVelocityToJoints(const double q[GRAVITY_JOINTS], const double v[3], bool keepOrientation, double lambda,
                          double dq[GRAVITY_JOINTS], double *sigmaMin)
{
    if (!model) return false;
    Eigen::VectorXd qv = toQ(q);
    pinocchio::computeJointJacobians(*model, *handData, qv);
    pinocchio::updateFramePlacements(*model, *handData);
    pinocchio::Data::Matrix6x Jfull(6, model->nv); Jfull.setZero();
    pinocchio::getFrameJacobian(*model, *handData, handFrame, pinocchio::LOCAL_WORLD_ALIGNED, Jfull);   // rows: vx vy vz wx wy wz
    // task Jacobian over the 7 arm joints only (the head is not in the hand's chain anyway)
    int rows = keepOrientation ? 6 : 3;
    Eigen::MatrixXd J(rows, GRAVITY_JOINTS - 1);
    for (int i = 1; i < GRAVITY_JOINTS; i++) J.col(i - 1) = Jfull.block(0, vIdx[i], rows, 1);
    Eigen::VectorXd xd = Eigen::VectorXd::Zero(rows);
    xd << v[0], v[1], v[2], Eigen::VectorXd::Zero(rows - 3);            // angular velocity 0 = keep orientation
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    if (sigmaMin) *sigmaMin = svd.singularValues().minCoeff();
    // damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 xd
    Eigen::MatrixXd JJt = J * J.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(rows, rows);
    Eigen::VectorXd d = J.transpose() * JJt.ldlt().solve(xd);
    dq[0] = 0;
    for (int i = 1; i < GRAVITY_JOINTS; i++) dq[i] = d[i - 1];
    return true;
}
