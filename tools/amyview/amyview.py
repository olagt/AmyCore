#!/usr/bin/env python3
# Copyright (C) 2023-2026 Ola Gatner
# SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
"""AmyView: 3D view of the Sawyer arm in its current pose, in a web browser (MeshCat).

Reads what AmyCore publishes and never talks to the robot:
  /tmp/amycore.pose    (~30 Hz) per joint: angle [rad], held target [rad], status flags, error; goal of a running move
  /tmp/amycore.status  (5 Hz, fallback) "side <name> ... pos <counts>" lines
Shows:
  solid robot         current pose (real link meshes of thirdparty/sawyer_robot/sawyer_description)
  green ghost         held targets (where AmyCore holds each joint; shows lag and sag)
  blue ghost          goal of a running coordinated move ("Go to home posture", "Go to")
  grey plane          table plane from AmyConfig/collision.txt (table_z_m, arm base frame)
  orange shapes       obstacles from AmyConfig/collision.txt (re-read every 2 s: place them while watching)
Joints whose angle is unknown (not homed) keep their last known angle (0 at first) and are listed in the terminal.

usage: tools/amyview/amyview.sh [--demo] [--no-ghosts]
       then open one of the printed URLs: http://127.0.0.1:7000/static/ only works in a browser on the NUC itself;
       from another computer use the NUC's address, e.g. http://<robot-pc>:7000/static/, or
       ssh -L 7000:localhost:7000 <robot-pc> and open http://localhost:7000/static/ there
"""
import argparse, atexit, math, os, signal, subprocess, sys, time
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as mg
import meshcat.transformations as mt

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
URDF = os.path.join(REPO, 'AmyConfig/model/sawyer_intera2023.urdf')
PACKAGES = os.path.join(REPO, 'thirdparty/sawyer_robot')
COLLISION_CFG = os.path.join(REPO, 'AmyConfig/collision.txt')
POSE_FILE, STATUS_FILE = '/tmp/amycore.pose', '/tmp/amycore.status'
# AmyCore side names -> URDF joint names (order of the pose file)
SIDES = ['head_pan', 'j0', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6']
URDF_NAMES = ['head_pan'] + [f'right_j{i}' for i in range(7)]
COUNTS_PER_RAD = 1024.0 * 1000.0


def read_scene():
    """(table_z or None, [(name, type, centre, dims)]) from AmyConfig/collision.txt"""
    tz, obs = None, []
    try:
        for line in open(COLLISION_CFG):
            f = line.split('#')[0].split()
            if len(f) >= 2 and f[0] == 'table_z_m':
                tz = None if f[1] == 'none' else float(f[1])
            elif len(f) >= 7 and f[0] == 'obstacle':
                try:
                    obs.append((f[1], f[2], [float(x) for x in f[3:6]], [float(x) for x in f[6:]]))
                except ValueError:
                    pass
    except OSError:
        pass
    return tz, obs


def draw_scene(viewer, scene, old):
    """(re)draw table plane and obstacles when the file changed"""
    if scene == old:
        return
    tz, obs = scene
    viewer['amy/table'].delete(); viewer['amy/obstacles'].delete()
    if tz is not None:
        viewer['amy/table'].set_object(mg.Box([1.6, 1.6, 0.01]), mg.MeshLambertMaterial(color=0x888888, opacity=0.3, transparent=True))
        viewer['amy/table'].set_transform(mt.translation_matrix([0.5, 0.0, tz - 0.005]))
    mat = mg.MeshLambertMaterial(color=0xff8800, opacity=0.45, transparent=True)
    for name, typ, c, d in obs:
        if typ == 'box' and len(d) >= 3: geom, tf = mg.Box(d[:3]), mt.translation_matrix(c)
        elif typ == 'cylinder' and len(d) >= 2:   # meshcat cylinders are along y: turn to vertical
            geom, tf = mg.Cylinder(d[1], d[0]), mt.translation_matrix(c) @ mt.rotation_matrix(math.pi / 2, [1, 0, 0])
        elif typ == 'sphere' and len(d) >= 1: geom, tf = mg.Sphere(d[0]), mt.translation_matrix(c)
        else: continue
        viewer['amy/obstacles/' + name].set_object(geom, mat)
        viewer['amy/obstacles/' + name].set_transform(tf)
    print(f"scene: table {'none' if tz is None else f'{tz:+.3f} m'}, obstacles: {', '.join(o[0] for o in obs) or 'none'}", flush=True)


def read_pose():
    """(angles{side: rad or None}, targets{side: rad or None}, goal[8] or None, move state, age [s]) or None"""
    try:
        age = time.time() - os.path.getmtime(POSE_FILE)
        lines = open(POSE_FILE).read().split('\n')
    except OSError:
        return None
    ang, tgt, goal, move = {}, {}, None, 'idle'
    for l in lines:
        f = l.split()
        if len(f) >= 3 and f[0] in SIDES:
            ang[f[0]] = None if f[1] == 'nan' else float(f[1])
            tgt[f[0]] = None if f[2] == 'nan' else float(f[2])
        elif f and f[0] == 'goal' and len(f) == 9:
            goal = [float(x) for x in f[1:]]
        elif f and f[0] == 'move':
            move = ' '.join(f[1:])
    return ang, tgt, goal, move, age


def read_status():
    """fallback: angles from the 5 Hz status file (counts), no targets"""
    try:
        age = time.time() - os.path.getmtime(STATUS_FILE)
        lines = open(STATUS_FILE).read().split('\n')
    except OSError:
        return None
    ang = {}
    for l in lines:
        f = l.split()
        if len(f) > 3 and f[0] == 'side' and f[1] in SIDES and 'pos' in f:
            homed = 'HOMED' in f
            ang[f[1]] = int(f[f.index('pos') + 1]) / COUNTS_PER_RAD if homed else None
    return ang, {}, None, 'unknown (status file)', age


def make_viz(model, cmodel, vmodel, viewer, root, color=None):
    viz = MeshcatVisualizer(model, cmodel, vmodel)
    viz.initViewer(viewer=viewer)
    if color is None:
        viz.loadViewerModel(rootNodeName=root)
    else:
        viz.loadViewerModel(rootNodeName=root, visual_color=color)
    return viz


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--demo', action='store_true', help='animate without AmyCore (checks the viewer)')
    ap.add_argument('--no-ghosts', action='store_true', help='only the current pose')
    ap.add_argument('--rate', type=float, default=30.0, help='updates per second')
    args = ap.parse_args()

    model, cmodel, vmodel = pin.buildModelsFromUrdf(URDF, package_dirs=[PACKAGES])
    idx = [model.joints[model.getJointId(n)].idx_q for n in URDF_NAMES]
    viewer = None
    actual = make_viz(model, cmodel, vmodel, viewer, 'amy/actual')
    viewer = actual.viewer
    # meshcat runs its web server as a child process that would outlive us (and block port 7000): stop it on exit
    proc = getattr(getattr(viewer, 'window', None), 'server_proc', None)
    def stop_server(*_):
        if proc is not None and proc.poll() is None: proc.terminate()
        sys.exit(0)
    atexit.register(lambda: proc is not None and proc.poll() is None and proc.terminate())
    signal.signal(signal.SIGTERM, stop_server); signal.signal(signal.SIGHUP, stop_server)
    target = goal = None
    if not args.no_ghosts:
        target = make_viz(model, cmodel, vmodel, viewer, 'amy/target', color=[0.2, 0.9, 0.3, 0.25])
        goal = make_viz(model, cmodel, vmodel, viewer, 'amy/goal', color=[0.2, 0.4, 1.0, 0.25])
        viewer['amy/goal'].set_property('visible', False)
    scene = read_scene(); draw_scene(viewer, scene, None); last_scene = time.time()
    print(f"AmyView: open {viewer.url()}   (in a browser on this computer)", flush=True)
    # 127.0.0.1 is the browser's own machine: from another computer the NUC's address is needed (server listens on all)
    port = viewer.url().split(':')[-1].split('/')[0]
    try:
        addrs = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2).stdout.split()
    except Exception:
        addrs = []
    for a in addrs:
        if ':' not in a and not a.startswith(('172.17.', '192.168.122.', '192.168.88.')):   # IPv4; not docker/libvirt bridges or the robot network
            print(f"        from another computer: http://{a}:{port}/static/", flush=True)

    q_act = pin.neutral(model); q_tgt = q_act.copy()
    known = [False] * 8
    t0 = time.time(); last_print = 0; goal_shown = False
    while True:
        if time.time() - last_scene > 2:          # collision.txt edited? (table, obstacles)
            new = read_scene(); draw_scene(viewer, new, scene); scene = new; last_scene = time.time()
        if args.demo:
            t = time.time() - t0
            neutral = [0, 0, -1.18, 0, 2.18, 0, 0.57, 3.3161]
            for k, i in enumerate(idx):
                q_act[i] = neutral[k] + 0.4 * math.sin(0.5 * t + k)
                q_tgt[i] = neutral[k]
            known = [True] * 8; move, age, gq = 'demo', 0.0, None
        else:
            r = read_pose()
            if r is None or r[4] > 1.0:          # no or stale pose file: try the status file
                r2 = read_status()
                r = r2 if r2 is not None else r
            if r is None:
                if time.time() - last_print > 2:
                    print("waiting for /tmp/amycore.pose (is AmyCore running?)", flush=True); last_print = time.time()
                time.sleep(0.5); continue
            ang, tgt, gq, move, age = r
            for k, (side, i) in enumerate(zip(SIDES, idx)):
                if ang.get(side) is not None:
                    q_act[i] = ang[side]; known[k] = True
                q_tgt[i] = tgt[side] if tgt.get(side) is not None else q_act[i]
        actual.display(q_act)
        if target is not None:
            target.display(q_tgt)
            show = gq is not None
            if show:
                q_goal = q_act.copy()
                for k, i in enumerate(idx): q_goal[i] = gq[k]
                goal.display(q_goal)
            if show != goal_shown:
                viewer['amy/goal'].set_property('visible', show); goal_shown = show
        if time.time() - last_print > 2:
            unknown = [s for s, k in zip(SIDES, known) if not k]
            deg = ' '.join(f"{s}={math.degrees(q_act[i]):.1f}" for s, i in zip(SIDES, idx))
            print(f"pose age {age:4.2f} s  move {move}  {deg}" + (f"  unknown: {' '.join(unknown)}" if unknown else ''), flush=True)
            last_print = time.time()
        time.sleep(1.0 / args.rate)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
