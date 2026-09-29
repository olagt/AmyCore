// Copyright (C) 2023-2026 Ola Gatner
// SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
// Self-collision and table check (see collision.h).
#include "collision.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <coal/shape/geometric_shapes.h>
#include <coal/BVH/BVH_model.h>
#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace {
std::unique_ptr<pinocchio::Model> model;
std::unique_ptr<pinocchio::Data>  data;
std::unique_ptr<pinocchio::GeometryModel> geom;
std::unique_ptr<pinocchio::GeometryData>  geomData;
int qIdx[GRAVITY_JOINTS];
const char *urdfNames[GRAVITY_JOINTS] = {"head_pan", "right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"};

// link name of a geometry; obstacles are attached to the world, so they are named by their own name
std::string linkOf(const pinocchio::GeometryObject &o) { return o.parentJoint == 0 && o.parentFrame == 0 ? o.name : model->frames[o.parentFrame].name; }
struct TableLink { pinocchio::GeomIndex geom; std::vector<coal::Vec3s> verts; };
std::vector<TableLink> tableLinks;
bool tableOn = false; double tableHeight = 0, tableMargin = 0;
double generalMargin = 0.02;
}

bool collisionInit(const char *urdf, const char *packageDir, double margin, bool table, double tableZ, char *err, int errLen)
{
    try
    {
        auto m = std::make_unique<pinocchio::Model>();
        pinocchio::urdf::buildModel(urdf, *m);
        for (int i = 0; i < GRAVITY_JOINTS; i++)
        {
            if (!m->existJointName(urdfNames[i])) { snprintf(err, errLen, "joint %s not in %s", urdfNames[i], urdf); return false; }
            qIdx[i] = m->joints[m->getJointId(urdfNames[i])].idx_q();
        }
        auto g = std::make_unique<pinocchio::GeometryModel>();
        std::vector<std::string> dirs{packageDir};
        pinocchio::urdf::buildGeom(*m, urdf, pinocchio::VISUAL, *g, dirs);    // real meshes as collision geometry

        // every pair of links except those on the same joint or on neighbouring joints (they touch at the joint)
        g->addAllCollisionPairs();
        std::vector<pinocchio::CollisionPair> keep;
        for (auto &cp : g->collisionPairs)
        {
            auto j1 = g->geometryObjects[cp.first].parentJoint, j2 = g->geometryObjects[cp.second].parentJoint;
            if (j1 == j2 || m->parents[j1] == j2 || m->parents[j2] == j1) continue;
            keep.push_back(cp);
        }
        g->removeAllCollisionPairs();
        for (auto &cp : keep) g->addCollisionPair(cp);

        // table plane z = tableZ against the links of j1 .. j6 (the base, pedestal and head can't reach it). A coal
        // plane vs a full mesh tests every triangle (~30 ms per pose) and this coal has no convex hulls (built without
        // qhull), so the lowest mesh vertex is computed directly (collisionFree): exact and well under a millisecond
        tableOn = table; tableHeight = tableZ; tableMargin = margin; tableLinks.clear();
        if (table)
        {
            auto jFirst = (pinocchio::JointIndex)m->getJointId("right_j1");
            for (pinocchio::GeomIndex k = 0; k < g->geometryObjects.size(); k++)
            {
                auto &o = g->geometryObjects[k];
                if (o.parentJoint < jFirst) continue;
                auto bvh = std::dynamic_pointer_cast<coal::BVHModelBase>(o.geometry);
                if (!bvh || !bvh->vertices) continue;
                TableLink tl; tl.geom = k;
                tl.verts.assign(bvh->vertices->begin(), bvh->vertices->end());
                tableLinks.push_back(std::move(tl));
            }
        }
        data = std::make_unique<pinocchio::Data>(*m);
        geomData = std::make_unique<pinocchio::GeometryData>(*g);
        for (auto &r : geomData->collisionRequests) r.security_margin = margin;
        generalMargin = margin;
        model = std::move(m); geom = std::move(g);
        return true;
    }
    catch (const std::exception &e) { snprintf(err, errLen, "%s", e.what()); return false; }
}

bool collisionAddObstacle(const char *name, int type, const double center[3], const double dims[3], char *err, int errLen)
{
    if (!model) { snprintf(err, errLen, "collision model not loaded"); return false; }
    std::shared_ptr<coal::CollisionGeometry> shape;
    if      (type == 0) shape = std::make_shared<coal::Box>(dims[0], dims[1], dims[2]);
    else if (type == 1) shape = std::make_shared<coal::Cylinder>(dims[0], dims[1]);
    else if (type == 2) shape = std::make_shared<coal::Sphere>(dims[0]);
    else { snprintf(err, errLen, "unknown obstacle type %d", type); return false; }
    pinocchio::SE3 at(Eigen::Matrix3d::Identity(), Eigen::Vector3d(center[0], center[1], center[2]));
    auto k = geom->addGeometryObject(pinocchio::GeometryObject(name, 0, 0, at, shape));
    for (pinocchio::GeomIndex i = 0; i < geom->geometryObjects.size(); i++)
        if (i != k && geom->geometryObjects[i].parentJoint != 0) geom->addCollisionPair(pinocchio::CollisionPair(i, k));
    // the pair list changed: new GeometryData with the general margin (pair margins / disabled links are set after this)
    geomData = std::make_unique<pinocchio::GeometryData>(*geom);
    for (auto &r : geomData->collisionRequests) r.security_margin = generalMargin;
    return true;
}

bool collisionSetPairMargin(const char *link1, const char *link2, double margin)
{
    if (!model) return false;
    bool found = false;
    for (size_t k = 0; k < geom->collisionPairs.size(); k++)
    {
        auto a = linkOf(geom->geometryObjects[geom->collisionPairs[k].first]), b = linkOf(geom->geometryObjects[geom->collisionPairs[k].second]);
        if ((a == link1 && b == link2) || (a == link2 && b == link1)) { geomData->collisionRequests[k].security_margin = margin; found = true; }
    }
    return found;
}

bool collisionDisableLink(const char *link)
{
    if (!model) return false;
    bool found = false;
    for (size_t k = 0; k < geom->collisionPairs.size(); k++)
        if (linkOf(geom->geometryObjects[geom->collisionPairs[k].first]) == link || linkOf(geom->geometryObjects[geom->collisionPairs[k].second]) == link)
        { geomData->activeCollisionPairs[k] = false; found = true; }
    return found;
}

bool collisionFree(const double q[GRAVITY_JOINTS], char *info, int infoLen)
{
    if (!model) { snprintf(info, infoLen, "collision model not loaded"); return false; }
    Eigen::VectorXd qv = Eigen::VectorXd::Zero(model->nq);
    for (int i = 0; i < GRAVITY_JOINTS; i++) qv[qIdx[i]] = q[i];
    bool hit = pinocchio::computeCollisions(*model, *data, *geom, *geomData, qv, false);   // also updates oMg
    std::string s;
    for (auto &tl : tableLinks)
    {
        const auto &M = geomData->oMg[tl.geom];
        Eigen::Vector3d r = M.rotation().row(2).transpose();
        double lowest = 1e9;
        for (auto &v : tl.verts) lowest = std::min(lowest, r.dot(v.cast<double>()));
        lowest += M.translation().z();
        if (lowest < tableHeight + tableMargin)
        {
            if (!s.empty()) s += ", ";
            char b[96]; snprintf(b, sizeof b, "%s - table (lowest point %.3f m)", linkOf(geom->geometryObjects[tl.geom]).c_str(), lowest);
            s += b; hit = true;
        }
    }
    if (!hit) return true;
    for (size_t k = 0; k < geom->collisionPairs.size(); k++)
        if (geomData->collisionResults[k].isCollision())
        {
            auto &cp = geom->collisionPairs[k];
            if (!s.empty()) s += ", ";
            s += linkOf(geom->geometryObjects[cp.first]) + " - " + linkOf(geom->geometryObjects[cp.second]);
        }
    snprintf(info, infoLen, "%s", s.c_str());
    return false;
}
