#!/usr/bin/env python3
"""
Replay pose estimator that matches the on-robot 3D pipeline during pipeline 1,
and adds 2D bearing-triangulated updates during pipeline 0.

Pipeline 1: uses botpose_wpiblue / botpose_orb_wpiblue — same data as on-robot.
Pipeline 0: uses rawfiducials bearing + height triangulation with gyro heading.
Both fused with swerve odometry via SwerveDrive4PoseEstimator.

Camera constants match Constants.java exactly.

Usage:
    python3 replay_2d_vision.py [input.wpilog [output.wpilog]]
"""

import math, struct, sys, os, time
from collections import defaultdict

import torch

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    SwerveDrive4Kinematics, SwerveModulePosition, SwerveModuleState,
)
from wpimath import units
from wpiutil.log import DataLogReader

try:
    import ctypes, ctypes.util
    _libm = ctypes.CDLL(ctypes.util.find_library("m"), use_errno=True)
    _libm.fedisableexcept(0x3F)
except Exception:
    pass

# ─── AprilTag field layout (k2026RebuiltWelded) ──────────────────────────────
APRIL_TAGS = {
    1:(11.878,7.4248,0.889),2:(11.9154,4.638,1.124),3:(11.3119,4.3902,1.124),
    4:(11.3119,4.0346,1.124),5:(11.9154,3.4312,1.124),6:(11.878,0.6445,0.889),
    7:(11.9529,0.6445,0.889),8:(12.271,3.4312,1.124),9:(12.5192,3.679,1.124),
    10:(12.5192,4.0346,1.124),11:(12.271,4.638,1.124),12:(11.9529,7.4248,0.889),
    13:(16.5333,7.4033,0.552),14:(16.5333,6.9715,0.552),15:(16.533,4.3236,0.552),
    16:(16.533,3.8918,0.552),17:(4.6631,0.6445,0.889),18:(4.6256,3.4312,1.124),
    19:(5.2292,3.679,1.124),20:(5.2292,4.0346,1.124),21:(4.6256,4.638,1.124),
    22:(4.6631,7.4248,0.889),23:(4.5882,7.4248,0.889),24:(4.27,4.638,1.124),
    25:(4.0219,4.3902,1.124),26:(4.0219,4.0346,1.124),27:(4.27,3.4312,1.124),
    28:(4.5882,0.6445,0.889),29:(0.0077,0.666,0.552),30:(0.0077,1.0978,0.552),
    31:(0.0081,3.7457,0.552),32:(0.0081,4.1775,0.552),
}

# ─── Camera constants — match Constants.java ─────────────────────────────────
CAMERAS = {
    "limelight-left": {
        "id": 0,
        "cx": units.inchesToMeters(-10.5),
        "cy": units.inchesToMeters(13.0),
        "cz": units.inchesToMeters(10.5),   # calibrated: physical measurement
        "pitch_rad": units.degreesToRadians(-13.0),  # calibrated from P1 ground truth
        "yaw_rad": math.pi / 2.0,
    },
    "limelight-front": {
        "id": 1,
        "cx": units.inchesToMeters(2.75),
        "cy": 0.0,
        "cz": units.inchesToMeters(16.25),
        "pitch_rad": 0.0,
        "yaw_rad": 0.0,
    },
    "limelight-right": {
        "id": 2,
        "cx": units.inchesToMeters(-10.375),
        "cy": units.inchesToMeters(-13.0),
        "cz": units.inchesToMeters(8.25),   # calibrated: physical measurement
        "pitch_rad": units.degreesToRadians(-16.0),  # calibrated from P1 ground truth
        "yaw_rad": -math.pi / 2.0,
    },
}

RF_PER_FID = 7

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CORRECTION_MODEL_PATH = os.path.join(SCRIPT_DIR, "bearing_correction_net.pt")

# ─── Swerve constants ────────────────────────────────────────────────────────
WHEEL_RADIUS_M = units.inchesToMeters(2.0)
_HS = units.inchesToMeters(10.875)
KINEMATICS = SwerveDrive4Kinematics(
    Translation2d(_HS, _HS), Translation2d(_HS, -_HS),
    Translation2d(-_HS, _HS), Translation2d(-_HS, -_HS),
)

# ─── Vision constants — match Constants.java / VisionSubsystem.java ──────────
MAX_AMBIGUITY = 0.3
MAX_Z_ERROR = 0.75
LINEAR_STD_BASELINE = 0.02
ANGULAR_STD_BASELINE = 0.06
LINEAR_STD_MT2_FACTOR = 0.5
FIELD_LENGTH = 17.548  # k2026RebuiltWelded
FIELD_WIDTH = 8.052


# ─── 3D rotation helper ──────────────────────────────────────────────────────
def _rotate_cam_to_robot(vx, vy, vz, pitch, yaw):
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    x1 = vx * cp + vz * sp; y1 = vy; z1 = -vx * sp + vz * cp
    return x1 * cy - y1 * sy, x1 * sy + y1 * cy, z1


def compute_tag_robot_pos(tag_id, txnc_deg, tync_deg, dist_cam,
                          cam, robot_heading_rad):
    """Bearing + height triangulation → robot (x, y, d_h)."""
    tag = APRIL_TAGS.get(tag_id)
    if tag is None:
        return None
    tag_x, tag_y, tag_z = tag
    txnc_rad = math.radians(txnc_deg)
    tync_rad = math.radians(tync_deg)
    dz = tag_z - cam["cz"]

    if dist_cam > 0.01:
        if dz * dz >= dist_cam * dist_cam:
            return None
        d_h = math.sqrt(dist_cam * dist_cam - dz * dz)
    else:
        ll_Z = math.cos(tync_rad) * math.cos(txnc_rad)
        ll_X = math.cos(tync_rad) * math.sin(txnc_rad)
        ll_Y = math.sin(tync_rad)
        vx, vy, vz = _rotate_cam_to_robot(
            ll_Z, -ll_X, ll_Y, cam["pitch_rad"], cam["yaw_rad"])
        if abs(vz) < 0.005:
            return None
        scale = dz / vz
        if scale < 0.05 or scale > 30.0:
            return None
        d_h = scale * math.sqrt(vx * vx + vy * vy)

    if d_h < 0.10 or d_h > 15.0:
        return None

    bearing = robot_heading_rad + cam["yaw_rad"] - txnc_rad
    cos_h, sin_h = math.cos(robot_heading_rad), math.sin(robot_heading_rad)
    dx_cam = cam["cx"] * cos_h - cam["cy"] * sin_h
    dy_cam = cam["cx"] * sin_h + cam["cy"] * cos_h
    return (tag_x - d_h * math.cos(bearing) - dx_cam,
            tag_y - d_h * math.sin(bearing) - dy_cam, d_h)


def combine_tag_estimates(estimates):
    if not estimates:
        return None
    tw = wx = wy = ds = 0.0
    for rx, ry, dh in estimates:
        w = 1.0 / max(dh * dh, 1e-4)
        wx += w * rx; wy += w * ry; tw += w; ds += dh
    return wx / tw, wy / tw, ds / len(estimates)


# ─── Decode / utility ─────────────────────────────────────────────────────────
def _dbl(data):
    n = len(data) // 8
    return list(struct.unpack_from(f"<{n}d", data)) if n else []

def _rot2d_rad(data):
    n = len(data) // 8
    return [struct.unpack_from("<d", data, i * 8)[0] for i in range(n)]

def _bisect(lst, ts):
    lo, hi = 0, len(lst)
    while lo < hi:
        mid = (lo + hi) // 2
        if lst[mid][0] < ts: lo = mid + 1
        else: hi = mid
    return lo

def _nearest(lst, ts, max_gap=None):
    if not lst: return None
    i = _bisect(lst, ts)
    i = max(0, min(i, len(lst) - 1))
    if i > 0 and abs(lst[i-1][0] - ts) < abs(lst[i][0] - ts): i -= 1
    if max_gap is not None and abs(lst[i][0] - ts) > max_gap: return None
    return lst[i]

def _encode_pose2d(x, y, h):
    return struct.pack("<3d", x, y, h)


# ─── WPILog writer ────────────────────────────────────────────────────────────
class _WPILogWriter:
    _MAGIC = b"WPILOG"; _VERSION = struct.pack("<H", 0x0100); _BF = 0x5D
    def __init__(self, path):
        self._f = open(path, "wb"); self._next = 1
        self._f.write(self._MAGIC + self._VERSION + struct.pack("<I", 0))
    def _append(self, eid, payload, ts_us):
        self._f.write(struct.pack("<BHI", self._BF, eid, len(payload)))
        self._f.write(struct.pack("<Q", ts_us & 0xFFFFFFFFFFFF)[:6])
        self._f.write(payload)
    def _start(self, name, type_str, meta, ts_us):
        eid = self._next; self._next += 1
        nb, tb, mb = name.encode(), type_str.encode(), meta.encode()
        self._append(0, bytes([0]) + struct.pack("<I", eid) +
            struct.pack("<I", len(nb)) + nb + struct.pack("<I", len(tb)) + tb +
            struct.pack("<I", len(mb)) + mb, ts_us)
        return eid
    def addSchema(self, sn, st, s, ts=0):
        eid = self._start(f"/.schema/{sn}", st, "", ts); self._append(eid, s.encode(), ts)
    def start(self, name, ts, m=""): return self._start(name, "struct:Pose2d", m, ts)
    def startD(self, name, ts=0): return self._start(name, "double", "", ts)
    def appendD(self, eid, v, ts): self._append(eid, struct.pack("<d", v), ts)
    def appendP(self, eid, x, y, h, ts): self._append(eid, _encode_pose2d(x, y, h), ts)
    def flush(self):
        self._f.flush(); sz = self._f.tell(); self._f.close(); return sz


# ─── Load log ─────────────────────────────────────────────────────────────────
def load_log(path):
    print(f"  Reading {os.path.getsize(path)/1e6:.0f} MB...")
    t0 = time.monotonic()
    reader = DataLogReader(path)
    eh = {}
    odom_buf = {}; orig_list = []; gyro_flat = []; speed_list = []
    ll_rf = defaultdict(list); ll_tl = defaultdict(list); ll_cl = defaultdict(list)
    ll_mt1 = defaultdict(list); ll_mt2 = defaultdict(list); ll_pipe = defaultdict(list)

    for rec in reader:
        if rec.isControl():
            if rec.isStart():
                sd = rec.getStartData(); n, e = sd.name, sd.entry
                if "Drive/Module" in n:
                    try: m = int(n.split("/Module")[1][0])
                    except: continue
                    if n.endswith("/OdometryDrivePositionsRad"): eh[e] = ("dd", m)
                    elif n.endswith("/OdometryTurnPositions"): eh[e] = ("dt", m)
                    elif n.endswith("/OdometryTimestamps"): eh[e] = ("dts", m)
                elif n.endswith("Gyro/OdometryYawPositions"): eh[e] = "gyro"
                elif n.endswith("Odometry/Robot"): eh[e] = "orig"
                elif n.endswith("SwerveStates/Measured"): eh[e] = "swerve"
                else:
                    for cn in CAMERAS:
                        pfx = f"NT:/{cn}/"
                        if n.startswith(pfx):
                            k = n[len(pfx):]
                            if k == "rawfiducials": eh[e] = ("rf", cn)
                            elif k == "tl": eh[e] = ("tl", cn)
                            elif k == "cl": eh[e] = ("cl", cn)
                            elif k == "botpose_wpiblue": eh[e] = ("mt1", cn)
                            elif k == "botpose_orb_wpiblue": eh[e] = ("mt2", cn)
                            elif k == "pipeline": eh[e] = ("pipe", cn)
                            break
            continue
        e = rec.getEntry(); h = eh.get(e)
        if h is None: continue
        ts = rec.getTimestamp() / 1e6; raw = rec.getRaw()

        if h == "gyro":
            arr = _rot2d_rad(raw)
            if arr: gyro_flat.append((ts, arr[-1]))
            odom_buf.setdefault(ts, {})["g"] = arr
        elif h == "orig":
            if len(raw) >= 24:
                x, y, hh = struct.unpack_from("<3d", raw)
                orig_list.append((ts, x, y, hh))
        elif h == "swerve":
            SS = 16; ns = len(raw) // SS
            if ns == 4:
                st = tuple(SwerveModuleState(
                    struct.unpack_from("<d", raw, i*SS)[0],
                    Rotation2d(struct.unpack_from("<d", raw, i*SS+8)[0]))
                    for i in range(4))
                cs = KINEMATICS.toChassisSpeeds(st)
                speed_list.append((ts, math.hypot(cs.vx, cs.vy), abs(cs.omega)))
        elif isinstance(h, tuple):
            k = h[0]
            if k == "dd": odom_buf.setdefault(ts, {}).setdefault("d", {})[h[1]] = _dbl(raw)
            elif k == "dt": odom_buf.setdefault(ts, {}).setdefault("t", {})[h[1]] = _rot2d_rad(raw)
            elif k == "dts": odom_buf.setdefault(ts, {}).setdefault("ts", {})[h[1]] = _dbl(raw)
            elif k == "rf":
                v = _dbl(raw)
                if v: ll_rf[h[1]].append((ts, v))
            elif k == "tl":
                v = _dbl(raw)
                if v: ll_tl[h[1]].append((ts, v[0]))
            elif k == "cl":
                v = _dbl(raw)
                if v: ll_cl[h[1]].append((ts, v[0]))
            elif k == "mt1":
                v = _dbl(raw)
                if len(v) >= 11: ll_mt1[h[1]].append((ts, v))
            elif k == "mt2":
                v = _dbl(raw)
                if len(v) >= 11: ll_mt2[h[1]].append((ts, v))
            elif k == "pipe":
                v = _dbl(raw)
                if v: ll_pipe[h[1]].append((ts, v[0]))

    print(f"  Read in {time.monotonic()-t0:.1f}s")

    # Odometry batches
    odom = []
    for lts, buf in sorted(odom_buf.items()):
        if not all(k in buf for k in ("d", "t", "ts", "g")): continue
        ta = buf["ts"].get(0, []); ga = buf["g"]
        if not ta or not ga or len(ta) != len(ga): continue
        samps = []
        for i in range(len(ta)):
            if not (math.isfinite(ta[i]) and math.isfinite(ga[i])): continue
            ok = True; pos = []
            for m in range(4):
                dr = buf["d"].get(m, []); tr = buf["t"].get(m, [])
                if i >= len(dr) or i >= len(tr): ok = False; break
                if not (math.isfinite(dr[i]) and math.isfinite(tr[i])): ok = False; break
                pos.append(SwerveModulePosition(dr[i] * WHEEL_RADIUS_M, Rotation2d(tr[i])))
            if ok and len(pos) == 4: samps.append((ta[i], pos, Rotation2d(ga[i])))
        if samps: odom.append((lts, samps))
    odom.sort()

    # Vision events: megatag1/2 (pipeline 1) + rawfiducials bearing (pipeline 0)
    # Type: ("mt1"|"mt2"|"bearing", obs_ts, cam_id, cam_name, data...)
    events = []

    for cn, cp in CAMERAS.items():
        # MegaTag 1 (botpose_wpiblue) — same as on-robot VisionIOLimelight
        for rec_ts, vals in sorted(ll_mt1[cn]):
            obs_ts = rec_ts - vals[6] * 1e-3  # server_ts - latency
            x, y, z = vals[0], vals[1], vals[2]
            if x == 0.0 and y == 0.0:
                continue  # no valid pose
            roll = math.radians(vals[3]); pitch = math.radians(vals[4])
            yaw = math.radians(vals[5])
            tag_count = int(vals[7])
            avg_dist = vals[9]
            amb = vals[17] if len(vals) >= 18 else 0.0
            events.append(("mt1", obs_ts, cp["id"], cn,
                           x, y, z, roll, pitch, yaw,
                           tag_count, avg_dist, amb))

        # MegaTag 2 (botpose_orb_wpiblue) — gyro-constrained
        for rec_ts, vals in sorted(ll_mt2[cn]):
            obs_ts = rec_ts - vals[6] * 1e-3
            x, y, z = vals[0], vals[1], vals[2]
            if x == 0.0 and y == 0.0:
                continue
            yaw = math.radians(vals[5])
            tag_count = int(vals[7])
            avg_dist = vals[9]
            events.append(("mt2", obs_ts, cp["id"], cn,
                           x, y, z, 0.0, 0.0, yaw,
                           tag_count, avg_dist, 0.0))

        # Rawfiducials bearing triangulation (works on pipeline 0 AND 1)
        tl_l = sorted(ll_tl[cn]); cl_l = sorted(ll_cl[cn])
        for rec_ts, fids in sorted(ll_rf[cn]):
            n_tags = len(fids) // RF_PER_FID
            if n_tags == 0: continue
            # Check if any tag has distToCamera — if so, skip (megatag handles it)
            has_dist = any(fids[i*7 + 4] > 0.01 for i in range(n_tags))
            if has_dist:
                continue  # pipeline 1 — megatag events handle this
            tl_e = _nearest(tl_l, rec_ts, max_gap=0.1)
            cl_e = _nearest(cl_l, rec_ts, max_gap=0.1)
            tl_ms = tl_e[1] if tl_e else 20.0
            cl_ms = cl_e[1] if cl_e else 0.0
            obs_ts = rec_ts - (tl_ms + cl_ms) / 1000.0
            events.append(("bearing", obs_ts, cp["id"], cn, fids))

    events.sort(key=lambda e: e[1])

    gyro_flat.sort(); speed_list.sort(); orig_list.sort()
    n_mt = sum(1 for e in events if e[0] in ("mt1", "mt2"))
    n_br = sum(1 for e in events if e[0] == "bearing")
    for v in ll_pipe.values(): v.sort()
    print(f"  odom={len(odom)} megatag={n_mt} bearing={n_br} orig={len(orig_list)}")
    return odom, events, gyro_flat, speed_list, orig_list, ll_pipe


# ─── Process ──────────────────────────────────────────────────────────────────
def process_log(in_path, out_path):
    t_start = time.monotonic()
    print(f"\nInput : {in_path}")
    print(f"Output: {out_path}")

    odom, events, gyro_flat, speed_list, orig_list, ll_pipe = load_log(in_path)
    if not odom:
        print("  No odometry — skipping."); return

    # Load correction model
    print(f"  Loading correction model: {CORRECTION_MODEL_PATH}")
    correction_model = torch.jit.load(CORRECTION_MODEL_PATH, map_location="cpu")
    correction_model.eval()

    init_pos = tuple(SwerveModulePosition(0.0, Rotation2d()) for _ in range(4))
    estimator = SwerveDrive4PoseEstimator(
        KINEMATICS, Rotation2d(), init_pos, Pose2d(),
        (0.1, 0.1, 0.1), (0.9, 0.9, 1e9))

    out = _WPILogWriter(out_path)
    out.addSchema("struct:Rotation2d", "structschema", "double value")
    out.addSchema("struct:Translation2d", "structschema", "double x;double y")
    out.addSchema("struct:Pose2d", "structschema",
                  "Translation2d translation;Rotation2d rotation")
    eid_2d = out.start("NT:/AdvantageKit/RealOutputs/Odometry/Robot2D", 0)
    eid_orig = out.start("NT:/AdvantageKit/RealOutputs/Odometry/RobotOriginal", 0)

    CAM_LABELS = {"limelight-left": "Left", "limelight-front": "Front",
                  "limelight-right": "Right"}
    pipe_eids = {
        cn: out.startD(f"NT:/AdvantageKit/RealOutputs/Vision/{CAM_LABELS[cn]}/Pipeline")
        for cn in CAMERAS
    }

    # Write pipeline data: interpolate onto every odom cycle so AdvantageScope
    # shows a continuous trace. Use last-known value per camera.
    pipe_state = {cn: 0.0 for cn in CAMERAS}
    pipe_idx = {cn: 0 for cn in CAMERAS}
    for lts, _ in odom:
        tus = int(lts * 1e6)
        if tus <= 0:
            continue
        for cn in CAMERAS:
            samples = ll_pipe.get(cn, [])
            while pipe_idx[cn] < len(samples) and samples[pipe_idx[cn]][0] <= lts:
                pipe_state[cn] = samples[pipe_idx[cn]][1]
                pipe_idx[cn] += 1
            out.appendD(pipe_eids[cn], pipe_state[cn], tus)

    import gc; gc.collect(); gc.disable()

    ev_idx = 0
    accepted_mt = accepted_br = rejected = 0
    first_lock = False; odom_init = False
    last_odom_ts = -1.0; heading_offset = None

    # Per-camera rolling buffer for bearing estimates — we collect multiple
    # frames and submit a single averaged update every BEARING_INTERVAL_S.
    # This prevents 30-Hz noisy measurements from jerking the Kalman filter.
    BEARING_INTERVAL_S = 0.25   # max one update per camera per 250 ms
    BEARING_WINDOW_S   = 0.25   # collect estimates from last 250 ms
    bearing_buf = {cn: [] for cn in CAMERAS}   # list of (obs_ts, rx, ry, dh)
    last_bearing_submit = {cn: -999.0 for cn in CAMERAS}

    nb = len(odom); REP = max(1, nb // 10)

    for bi, (lts, samps) in enumerate(odom):
        if bi % REP == 0:
            print(f"  {bi*100//nb:3d}% mt={accepted_mt} br={accepted_br} rej={rejected}",
                  flush=True)

        while ev_idx < len(events):
            ev = events[ev_idx]
            if ev[1] > lts:
                break
            ev_idx += 1

            typ, obs_ts = ev[0], ev[1]

            if typ in ("mt1", "mt2"):
                # ── MegaTag: exact same logic as VisionSubsystem.java ──
                _, _, cam_id, cam_name, x, y, z, roll, pitch, yaw, \
                    tag_count, avg_dist, amb = ev

                # Reject checks (same as VisionSubsystem)
                if tag_count == 0:
                    rejected += 1; continue
                if tag_count == 1 and amb > MAX_AMBIGUITY:
                    rejected += 1; continue
                if abs(z) > MAX_Z_ERROR:
                    rejected += 1; continue
                if x < 0 or x > FIELD_LENGTH or y < 0 or y > FIELD_WIDTH:
                    rejected += 1; continue

                # Std devs (same formula as VisionSubsystem)
                std_factor = avg_dist * avg_dist / tag_count
                lin_std = LINEAR_STD_BASELINE * std_factor
                ang_std = ANGULAR_STD_BASELINE * std_factor
                if typ == "mt2":
                    lin_std *= LINEAR_STD_MT2_FACTOR
                    ang_std = 1e9  # gyro owns rotation

                pose = Pose2d(x, y, Rotation2d(yaw))

                if not first_lock:
                    # Get heading offset from this good megatag pose
                    g = _nearest(gyro_flat, obs_ts, max_gap=0.1)
                    if g is None: rejected += 1; continue
                    heading_offset = yaw - g[1]
                    estimator.resetPosition(
                        Rotation2d(g[1]), tuple(samps[-1][1]), pose)
                    first_lock = True; odom_init = True
                    last_odom_ts = samps[-1][0]
                    print(f"  First lock (megatag): ({x:.2f}, {y:.2f}) t={obs_ts:.1f}s")
                    accepted_mt += 1; continue

                try:
                    estimator.addVisionMeasurement(
                        pose, obs_ts, (lin_std, lin_std, ang_std))
                    accepted_mt += 1
                except Exception:
                    rejected += 1

            elif typ == "bearing":
                # ── Pipeline 0 bearing triangulation ──
                # Accumulate every raw frame into a per-camera buffer;
                # only submit a fused update every BEARING_INTERVAL_S.
                _, _, cam_id, cam_name, fids = ev

                if not first_lock:
                    continue

                cam = CAMERAS[cam_name]
                g = _nearest(gyro_flat, obs_ts, max_gap=0.1)
                if g is None:
                    continue
                robot_heading = g[1] + heading_offset

                n_tags = len(fids) // RF_PER_FID
                tag_ests = []
                for i in range(n_tags):
                    off = i * 7
                    tag_id = int(fids[off])
                    txnc = fids[off+1]; tync = fids[off+2]
                    r = compute_tag_robot_pos(
                        tag_id, txnc, tync, fids[off+4], cam, robot_heading)
                    if r is not None:
                        tag_z = APRIL_TAGS.get(tag_id, (0,0,1.0))[2]
                        tag_ests.append((*r, txnc, tync, tag_z))

                if not tag_ests:
                    continue  # no valid tags this frame — don't count as rejected

                # Per-frame weighted combine
                tw = wx = wy = 0.0
                for rx_f, ry_f, dh, *_ in tag_ests:
                    w = 1.0 / max(dh*dh, 1e-4)
                    wx += w*rx_f; wy += w*ry_f; tw += w
                est_x, est_y = wx/tw, wy/tw

                n_valid = len(tag_ests)
                avg_dh = sum(e[2] for e in tag_ests) / n_valid
                avg_txnc_abs = sum(abs(e[3]) for e in tag_ests) / n_valid
                avg_tync = sum(e[4] for e in tag_ests) / n_valid
                avg_tag_z = sum(e[5] for e in tag_ests) / n_valid

                if not (-0.5 < est_x < 17.5 and -0.5 < est_y < 9.0):
                    continue

                # NN correction on the per-frame estimate
                cam_oh = [1.0 if cam_id == c else 0.0 for c in range(3)]
                feats = cam_oh + [
                    avg_dh, avg_dh**2, 1.0/max(n_valid, 1),
                    avg_txnc_abs, avg_tync, avg_tag_z,
                    math.sin(robot_heading), math.cos(robot_heading),
                    est_x, est_y,
                ]
                with torch.no_grad():
                    pred = correction_model(
                        torch.tensor([feats], dtype=torch.float32))
                    dx, dy = pred[0].tolist()

                rx = est_x + dx
                ry = est_y + dy

                if not (-0.5 < rx < 17.5 and -0.5 < ry < 9.0):
                    continue

                # Add to buffer (prune old entries)
                bearing_buf[cam_name].append((obs_ts, rx, ry, avg_dh, n_valid))
                bearing_buf[cam_name] = [
                    e for e in bearing_buf[cam_name]
                    if obs_ts - e[0] <= BEARING_WINDOW_S]

                # Only submit once per BEARING_INTERVAL_S
                if obs_ts - last_bearing_submit[cam_name] < BEARING_INTERVAL_S:
                    continue

                # Average all buffered estimates (weighted by 1/dh²)
                buf = bearing_buf[cam_name]
                tw2 = wx2 = wy2 = 0.0; total_n = 0; total_dh = 0.0
                for _, bx, by, bdh, bn in buf:
                    w = 1.0 / max(bdh*bdh, 1e-4)
                    wx2 += w*bx; wy2 += w*by; tw2 += w
                    total_n += bn; total_dh += bdh
                avg_rx = wx2/tw2; avg_ry = wy2/tw2
                avg_dh_buf = total_dh / len(buf)
                avg_n_buf = total_n / len(buf)

                if not (-0.5 < avg_rx < 17.5 and -0.5 < avg_ry < 9.0):
                    rejected += 1; continue

                # Outlier gate: if bearing estimate is far from current pose,
                # don't trust it — likely NN systematic error in this area.
                # This prevents bearing from pulling a good estimate to a wrong place.
                cur_pos = estimator.getEstimatedPosition()
                residual = math.hypot(avg_rx - cur_pos.X(), avg_ry - cur_pos.Y())
                MAX_RESIDUAL = 1.5  # meters — reject bearing if > 1.5m from current
                if residual > MAX_RESIDUAL:
                    rejected += 1; continue

                sw = _nearest(speed_list, obs_ts, max_gap=0.1)
                speed = sw[1] if sw else 0.0
                omega = sw[2] if sw else 0.0
                speed_factor = 1.0 + 3.0 * speed + 8.0 * omega

                std_factor = avg_dh_buf * avg_dh_buf / max(avg_n_buf, 1)
                lin_std = LINEAR_STD_BASELINE * std_factor * speed_factor * 3.0
                lin_std = max(lin_std, 0.12)

                # Use the heading at the midpoint of the buffer window for heading
                g_submit = _nearest(gyro_flat, obs_ts, max_gap=0.1)
                submit_heading = (g_submit[1] + heading_offset
                                  if g_submit else robot_heading)

                try:
                    estimator.addVisionMeasurement(
                        Pose2d(Translation2d(avg_rx, avg_ry),
                               Rotation2d(submit_heading)),
                        obs_ts, (lin_std, lin_std, 1e9))
                    accepted_br += 1
                    last_bearing_submit[cam_name] = obs_ts
                except Exception:
                    rejected += 1

        # Odometry
        last_ts, last_pos, last_gyro = samps[-1]
        if not odom_init:
            estimator.resetPosition(
                last_gyro, tuple(last_pos), Pose2d(0, 0, last_gyro))
            odom_init = True
        if last_ts > last_odom_ts:
            try:
                estimator.updateWithTime(last_ts, last_gyro, tuple(last_pos))
                last_odom_ts = last_ts
            except Exception:
                pass

        pos = estimator.getEstimatedPosition()
        if not (math.isfinite(pos.X()) and math.isfinite(pos.Y())
                and abs(pos.X()) < 50):
            estimator = SwerveDrive4PoseEstimator(
                KINEMATICS, Rotation2d(), init_pos, Pose2d(),
                (0.1, 0.1, 0.1), (0.9, 0.9, 1e9))
            first_lock = False; odom_init = False
            last_odom_ts = -1.0; heading_offset = None; continue

        tus = int(lts * 1e6)
        out.appendP(eid_2d, pos.X(), pos.Y(), pos.rotation().radians(), tus)
        orig = _nearest(orig_list, lts, max_gap=0.02)
        if orig:
            out.appendP(eid_orig, orig[1], orig[2], orig[3], tus)

    gc.enable()
    nbytes = out.flush()
    print(f"  MegaTag: {accepted_mt}, Bearing: {accepted_br}, Rejected: {rejected}")
    print(f"  Written: {out_path} ({nbytes:,} bytes) {time.monotonic()-t_start:.1f}s")


if __name__ == "__main__":
    if len(sys.argv) >= 2:
        inp = sys.argv[1]
        outp = sys.argv[2] if len(sys.argv) >= 3 else \
            os.path.splitext(inp)[0] + "_2d.wpilog"
    else:
        inp = "/mnt/c/frc/logs/14th/vision/FRC_20260315_014426.wpilog"
        outp = os.path.splitext(inp)[0] + "_2d.wpilog"
    process_log(inp, outp)
    print("\nDone. Open in AdvantageScope:")
    print("  Odometry/Robot2D       — megatag + 2D bearing + odometry")
    print("  Odometry/RobotOriginal — original on-robot estimate")
