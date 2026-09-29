#!/usr/bin/env python3
# Copyright (C) 2023-2026 Ola Gatner
# SPDX-License-Identifier: AGPL-3.0-only OR LicenseRef-AmyCore-Commercial
"""Gravity-model calibration for AmyCore (this robot's link masses / centres of mass + one torque offset per joint).

Why: with stiffness 0 (manual teaching, or the stiffness slider at 0) a joint only produces the torque feed-forward,
so every error of the gravity model and every torque-sensor offset becomes a real, unopposed torque (2026-09-26: the
arm went up and hit itself; in position mode j1 needed 1 Nm less than the model, j2 1.5 Nm less, j3 1 Nm more, j6's
sensor read +4 Nm unloaded). Holding still in position mode, a joint's measured effort IS the torque that holds the
arm (plus a little static friction), so efforts at many different poses identify the model.

  tools/gravcal.py record [--out FILE]   watch /tmp/amycore.status (read only, sends nothing) and store one averaged
                                         sample each time the arm has been still for 1.5 s at a new pose. Conditions:
                                         every arm joint ENABLED + HOMED, mode 7 (position), stiffness 100, teach off.
                                         Move the arm between poses any way you like (jog/goto/by hand); aim for
                                         20-40 poses that vary j1..j5 a lot (arm up, out, folded, wrist turned).
  tools/gravcal.py fit [--samples FILE] [--apply]
                                         fit masses, centres of mass (kept close to the URDF: prior) and per-joint
                                         offsets; prints the residual per joint with the URDF and with the fit.
                                         --apply writes AmyConfig/model/sawyer_calibrated.urdf and gravity_offsets.txt,
                                         which AmyCore uses for gravity compensation at its next start.
  tools/gravcal.py offsets [--apply]    quick fix without the full fit: j0 (vertical axis) and j6 (light, centred)
                                         carry almost no gravity torque, so their mean effort over 2 s with the arm
                                         still (stiffness 100, no jog) is their torque-sensor offset (j6 read +3-4 Nm
                                         on 2026-09-26, likely since the lamp impact; it turns the wrist whenever the
                                         stiffness is low). --apply writes those two lines of gravity_offsets.txt
                                         (other lines are kept); AmyCore loads the file at its next start.
  tools/gravcal.py selftest              fit a synthetic robot (masses +-15 %, COM shifts, offsets) to check the method

Units: status file positions are 1/1024 mrad, efforts 1/250 Nm. Pinocchio from /opt/openrobots."""
import sys, os, glob, time, math, argparse, collections
sys.path += glob.glob('/opt/openrobots/lib/python3*/site-packages')
import numpy as np
import pinocchio as pin

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
URDF = os.path.join(REPO, 'AmyConfig/model/sawyer_intera2023.urdf')
OUT_URDF = os.path.join(REPO, 'AmyConfig/model/sawyer_calibrated.urdf')
OUT_OFFS = os.path.join(REPO, 'AmyConfig/model/gravity_offsets.txt')
SAMPLES = os.path.join(REPO, 'AmyConfig/gravcal/samples.csv')
STATUS = '/tmp/amycore.status'
SIDES = ['head_pan', 'j0', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6']
ARM = SIDES[1:]
URDF_JOINT = {s: (s if s == 'head_pan' else 'right_' + s) for s in SIDES}
CNT_PER_RAD = 1024 * 1000.0
NM = 1 / 250.0
# prior (1 sigma) for the fit: residual noise (static friction) and how far the parameters may move from the URDF
SIGMA_TAU = 0.3            # Nm
SIGMA_MASS_REL = 0.15      # of the URDF mass
SIGMA_MC = 0.02            # kg*m on each first moment (mass * COM)
SIGMA_OFFSET = 5.0         # Nm (weak: offsets are mostly free)


# ------------------------------------------------------------------ model
def load_model(path=URDF):
    m = pin.buildModelFromUrdf(path)
    return m, m.createData()


def fit_columns(m):
    """regressor columns that are fitted: m, mc_x, mc_y, mc_z of the bodies of right_j0..right_j6"""
    cols = []
    for s in ARM:
        jid = m.getJointId(URDF_JOINT[s])
        cols += [10 * (jid - 1) + k for k in range(4)]
    return cols


def qvec(m, qd):
    q = pin.neutral(m)
    for s, v in qd.items(): q[m.joints[m.getJointId(URDF_JOINT[s])].idx_q] = v
    return q


def arm_rows(m, d, q):
    """static regressor (joint torque = Y theta) for the arm joints j0..j6 at pose q"""
    Y = pin.computeJointTorqueRegressor(m, d, q, np.zeros(m.nv), np.zeros(m.nv))
    idx = [m.joints[m.getJointId(URDF_JOINT[s])].idx_v for s in ARM]
    return Y[idx, :]


def theta0(m):
    return np.concatenate([m.inertias[i].toDynamicParameters() for i in range(1, m.njoints)])


def fit(m, d, poses, efforts):
    """poses: list of {side: rad}; efforts: N x 7 [Nm] (j0..j6). Returns theta (all params), offsets[7], report"""
    th0 = theta0(m); cols = fit_columns(m); ncol = len(cols); nj = len(ARM)
    A, b = [], []
    for qd, e in zip(poses, efforts):
        Y = arm_rows(m, d, qvec(m, qd))
        fixed = Y @ th0 - Y[:, cols] @ th0[cols]                   # torque of the parameters that stay at the URDF
        A.append(np.hstack([Y[:, cols], np.eye(nj)]) / SIGMA_TAU)   # [link params | offsets]
        b.append((e - fixed) / SIGMA_TAU)
    # prior rows: parameters near the URDF values, offsets near 0
    sig = []
    for k, c in enumerate(cols):
        sig.append(max(SIGMA_MASS_REL * th0[c], 0.05) if k % 4 == 0 else SIGMA_MC)
    P = np.zeros((ncol + nj, ncol + nj)); p = np.zeros(ncol + nj)
    for k in range(ncol): P[k, k] = 1 / sig[k]; p[k] = th0[cols[k]] / sig[k]
    for k in range(nj): P[ncol + k, ncol + k] = 1 / SIGMA_OFFSET
    x, *_ = np.linalg.lstsq(np.vstack(A + [P]), np.concatenate(b + [p]), rcond=None)
    th = th0.copy(); th[cols] = x[:ncol]
    for s in ARM:                                                    # a mass must stay positive
        k = 10 * (m.getJointId(URDF_JOINT[s]) - 1)
        if th[k] < 0.05: th[k] = 0.05
    return th, x[ncol:]


def predict(m, d, th, offs, qd):
    return arm_rows(m, d, qvec(m, qd)) @ th + offs


# ------------------------------------------------------------------ record
def read_status():
    try:
        age = time.time() - os.path.getmtime(STATUS)
        lines = open(STATUS).read().split('\n')
    except OSError:
        return None
    r = {'age': age, 'teach': '', 'sides': {}}
    for l in lines:
        f = l.split()
        if not f: continue
        if f[0] == 'teach': r['teach'] = f[1] if len(f) > 1 else ''
        if f[0] == 'side' and len(f) > 3 and f[1] in SIDES:
            g = lambda k, dflt=None: f[f.index(k) + 1] if k in f else dflt
            r['sides'][f[1]] = dict(pos=int(g('pos', 0)), eff=int(g('effort', 0)), stiff=int(g('stiff', 100)),
                                    mode=int(g('mode', 7)), ok='ENABLED' in f and 'HOMED' in f)
    return r


def record(out):
    os.makedirs(os.path.dirname(out), exist_ok=True)
    new = not os.path.exists(out)
    f = open(out, 'a')
    if new: f.write('time,' + ','.join('q_' + s for s in SIDES) + ',' + ','.join('eff_' + s for s in ARM) + '\n'); f.flush()
    n0 = max(0, sum(1 for _ in open(out)) - 1)
    print(f'recording to {out} ({n0} samples so far); move the arm to a pose, let go, wait ~2 s. Ctrl-C to stop.')
    win = collections.deque(maxlen=8)          # 8 x 0.2 s
    last_rec = None; last_why = ''; n = 0
    while True:
        time.sleep(0.2)
        r = read_status()
        why = ''
        if r is None or r['age'] > 1.0: why = 'AmyCore not running (status file old)'
        elif r['teach'] not in ('off', ''): why = 'manual teaching is on'
        else:
            for s in ARM:
                x = r['sides'].get(s)
                if not x or not x['ok']: why = f'{s} not ENABLED + HOMED'; break
                if x['mode'] != 7 or x['stiff'] != 100: why = f'{s}: needs position mode and stiffness 100 (stiffness all 100)'; break
        if why:
            win.clear()
            if why != last_why: print('  waiting:', why); last_why = why
            continue
        last_why = ''
        win.append(r['sides'])
        if len(win) < win.maxlen: continue
        span = max(max(w[s]['pos'] for w in win) - min(w[s]['pos'] for w in win) for s in ARM) / CNT_PER_RAD * 180 / math.pi
        if span > 0.03: continue                                  # still moving
        q = {s: np.mean([w[s]['pos'] for w in win]) / CNT_PER_RAD for s in SIDES if s in win[-1]}
        if last_rec is not None and max(abs(q[s] - last_rec[s]) for s in ARM) < math.radians(2): continue   # same pose
        eff = [np.mean([w[s]['eff'] for w in win]) * NM for s in ARM]
        f.write('%.1f,' % time.time() + ','.join('%.6f' % q.get(s, 0.0) for s in SIDES) + ',' + ','.join('%.4f' % e for e in eff) + '\n')
        f.flush(); last_rec = q; n += 1
        print(f'  pose {n0 + n}: ' + ' '.join(f'{s} {math.degrees(q[s]):7.1f}' for s in ARM) + '  | eff ' + ' '.join(f'{e:6.2f}' for e in eff))


# ------------------------------------------------------------------ fit / apply
def load_samples(path):
    a = np.genfromtxt(path, delimiter=',', names=True)
    a = np.atleast_1d(a)
    poses = [{s: float(row['q_' + s]) for s in SIDES} for row in a]
    eff = np.array([[row['eff_' + s] for s in ARM] for row in a])
    return poses, eff


def report(m, d, th, offs, poses, eff, label):
    P = np.array([predict(m, d, th, offs, q) for q in poses])
    r = eff - P
    print(f'{label:24s} ' + ' '.join(f'{s}:{np.sqrt(np.mean(r[:, i] ** 2)):5.2f}' for i, s in enumerate(ARM)) + '   (rms residual, Nm)')
    return r


def write_calibrated(m, th, offs):
    import xml.etree.ElementTree as ET
    t = ET.parse(URDF); root = t.getroot()
    child = {j.get('name'): j.find('child').get('link') for j in root.findall('joint')}
    for s in ARM:
        jid = m.getJointId(URDF_JOINT[s]); k = 10 * (jid - 1)
        mass, mc = th[k], th[k + 1:k + 4]
        link = next(l for l in root.findall('link') if l.get('name') == child[URDF_JOINT[s]])
        ine = link.find('inertial')
        ine.find('mass').set('value', '%.6f' % mass)
        o = ine.find('origin')
        if o is None: o = ET.SubElement(ine, 'origin'); o.set('rpy', '0 0 0')
        c = mc / mass
        o.set('xyz', '%.6f %.6f %.6f' % tuple(c))
    root.insert(0, ET.Comment(' gravity-calibrated by tools/gravcal.py %s from %s: link masses and COMs of right_l0..right_l6 '
                              % (time.strftime('%Y-%m-%d %H:%M'), os.path.basename(URDF))))
    t.write(OUT_URDF, xml_declaration=True, encoding='utf-8')
    with open(OUT_OFFS, 'w') as f:
        f.write('# per-joint torque offset [Nm] added to the gravity feed-forward (tools/gravcal.py %s)\n' % time.strftime('%Y-%m-%d %H:%M'))
        f.write('head_pan 0.0\n')
        for s, o in zip(ARM, offs): f.write('%s %.4f\n' % (URDF_JOINT[s], o))
    print('written', OUT_URDF, 'and', OUT_OFFS, '(AmyCore uses them from its next start)')


def cmd_fit(a):
    m, d = load_model()
    poses, eff = load_samples(a.samples)
    print(f'{len(poses)} poses from {a.samples}')
    if len(poses) < 8: print('WARNING: few poses; the fit mostly keeps the URDF values (prior). Aim for 20-40.')
    th0 = theta0(m)
    report(m, d, th0, np.zeros(len(ARM)), poses, eff, 'URDF, no offsets')
    th, offs = fit(m, d, poses, eff)
    report(m, d, th, offs, poses, eff, 'calibrated')
    print('offsets [Nm]: ' + ' '.join(f'{s} {o:+.2f}' for s, o in zip(ARM, offs)))
    print('link        mass URDF -> fit [kg]      COM URDF -> fit [mm]')
    for s in ARM:
        k = 10 * (m.getJointId(URDF_JOINT[s]) - 1)
        c0, c1 = th0[k + 1:k + 4] / th0[k] * 1000, th[k + 1:k + 4] / th[k] * 1000
        print(f'  {s:4s}  {th0[k]:6.3f} -> {th[k]:6.3f}     ({c0[0]:6.1f} {c0[1]:6.1f} {c0[2]:6.1f}) -> ({c1[0]:6.1f} {c1[1]:6.1f} {c1[2]:6.1f})')
    if len(poses) >= 6:                                       # leave-one-out: how well it predicts a pose it didn't see
        errs = []
        for i in range(len(poses)):
            keep = [k for k in range(len(poses)) if k != i]
            t1, o1 = fit(m, d, [poses[k] for k in keep], eff[keep])
            errs.append(eff[i] - predict(m, d, t1, o1, poses[i]))
        errs = np.array(errs)
        print('leave-one-out           ' + ' '.join(f'{s}:{np.sqrt(np.mean(errs[:, i] ** 2)):5.2f}' for i, s in enumerate(ARM)) + '   (rms, Nm: expected error at new poses)')
    if a.apply: write_calibrated(m, th, offs)
    else: print('(not written: add --apply)')


def cmd_offsets(a):
    """measure the torque-sensor offsets of j0 and j6 from the live status file (read only)"""
    m, d = load_model()
    effs = {'j0': [], 'j6': []}; first = None; t_end = time.time() + 2.0
    print('measuring j0 and j6 for 2 s: keep the arm still and untouched ...')
    while time.time() < t_end:
        time.sleep(0.2)
        r = read_status()
        if r is None or r['age'] > 1.0: sys.exit('AmyCore not running (status file old)')
        for s in ('j0', 'j6'):
            x = r['sides'].get(s)
            if not x or not x['ok'] or x['mode'] != 7 or x['stiff'] != 100:
                sys.exit(f'{s} must be ENABLED + HOMED, in position mode with stiffness 100')
            effs[s].append((x['pos'], x['eff'] * NM))
        if first is None: first = r
    offs = {}
    for s in ('j0', 'j6'):
        pos = [p for p, _ in effs[s]]
        if (max(pos) - min(pos)) / CNT_PER_RAD * 180 / math.pi > 0.05: sys.exit(f'{s} moved during the measurement: stop jogging and try again')
        # model torque at the current pose (unknown / unhomed joints count as 0; for j0 and j6 it is ~0 anyway)
        qd = {k: (v['pos'] / CNT_PER_RAD if v['ok'] else 0.0) for k, v in first['sides'].items()}
        g = pin.computeGeneralizedGravity(m, d, qvec(m, qd))[m.joints[m.getJointId(URDF_JOINT[s])].idx_v]
        offs[s] = np.mean([e for _, e in effs[s]]) - g
        print(f'  {s}: mean effort {np.mean([e for _, e in effs[s]]):+.2f} Nm, model {g:+.2f} Nm -> offset {offs[s]:+.2f} Nm')
    if not a.apply: print('(not written: add --apply)'); return
    lines = {}
    if os.path.exists(OUT_OFFS):
        for l in open(OUT_OFFS):
            f = l.split()
            if len(f) == 2 and not l.startswith('#'): lines[f[0]] = f[1]
    for s, o in offs.items(): lines[URDF_JOINT[s]] = '%.4f' % o
    with open(OUT_OFFS, 'w') as f:
        f.write('# per-joint torque offset [Nm] added to the gravity feed-forward (tools/gravcal.py; j0/j6 by "offsets" %s)\n' % time.strftime('%Y-%m-%d %H:%M'))
        for k in ['head_pan'] + [URDF_JOINT[s] for s in ARM]: f.write('%s %s\n' % (k, lines.get(k, '0.0')))
    print('written', OUT_OFFS, '(AmyCore loads it at its next start)')


def cmd_selftest(a):
    rng = np.random.default_rng(1)
    m, d = load_model()
    th0 = theta0(m); cols = fit_columns(m)
    true = th0.copy()
    for s in ARM:                                              # a "real" robot that differs from the URDF
        k = 10 * (m.getJointId(URDF_JOINT[s]) - 1)
        f = 1 + rng.uniform(-0.15, 0.15)
        c = th0[k + 1:k + 4] / th0[k] + rng.uniform(-0.01, 0.01, 3)
        true[k] = th0[k] * f; true[k + 1:k + 4] = true[k] * c
    toffs = rng.uniform(-3, 3, len(ARM))
    lim = {s: (m.lowerPositionLimit[m.joints[m.getJointId(URDF_JOINT[s])].idx_q], m.upperPositionLimit[m.joints[m.getJointId(URDF_JOINT[s])].idx_q]) for s in SIDES}
    rand_pose = lambda: {s: rng.uniform(*lim[s]) * 0.8 for s in SIDES}
    for n in (10, 20, 40):
        poses = [rand_pose() for _ in range(n)]
        eff = np.array([predict(m, d, true, toffs, q) + rng.normal(0, SIGMA_TAU, len(ARM)) for q in poses])
        th, offs = fit(m, d, poses, eff)
        test = [rand_pose() for _ in range(200)]
        e_urdf = np.array([predict(m, d, true, toffs, q) - predict(m, d, th0, np.zeros(len(ARM)), q) for q in test])
        e_fit = np.array([predict(m, d, true, toffs, q) - predict(m, d, th, offs, q) for q in test])
        print(f'{n:3d} poses: rms error at 200 new poses, URDF ' + ' '.join(f'{np.sqrt(np.mean(e_urdf[:, i]**2)):4.2f}' for i in range(len(ARM)))
              + '  | fitted ' + ' '.join(f'{np.sqrt(np.mean(e_fit[:, i]**2)):4.2f}' for i in range(len(ARM))) + '  Nm (j0..j6)')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest='cmd', required=True)
    r = sp.add_parser('record'); r.add_argument('--out', default=SAMPLES)
    f = sp.add_parser('fit'); f.add_argument('--samples', default=SAMPLES); f.add_argument('--apply', action='store_true')
    sp.add_parser('selftest')
    o = sp.add_parser('offsets'); o.add_argument('--apply', action='store_true')
    a = ap.parse_args()
    try:
        {'record': lambda: record(a.out), 'fit': lambda: cmd_fit(a), 'selftest': lambda: cmd_selftest(a), 'offsets': lambda: cmd_offsets(a)}[a.cmd]()
    except KeyboardInterrupt:
        print('\nstopped')
