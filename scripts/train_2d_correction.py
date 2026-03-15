#!/usr/bin/env python3
"""
Extract training data from logs and train a correction network for 2D bearing
triangulation. For every rawfiducials frame during pipeline 1 (distToCamera > 0),
compute the bearing estimate as if it were pipeline 0 (height tri), and learn
the correction to match the original fused robot pose.

Outputs: bearing_correction_net.pt
"""

import glob, math, struct, sys, os, time
from collections import defaultdict

import torch
import torch.nn as nn

from wpiutil.log import DataLogReader
from wpimath import units

try:
    import ctypes, ctypes.util
    _libm = ctypes.CDLL(ctypes.util.find_library("m"), use_errno=True)
    _libm.fedisableexcept(0x3F)
except: pass

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = "/home/greg/frc/logs/14th/vision"
MODEL_PATH = os.path.join(SCRIPT_DIR, "bearing_correction_net.pt")

# ─── Constants (match Constants.java) ─────────────────────────────────────────
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

CAMERAS = {
    "limelight-left": {
        "id": 0, "cx": units.inchesToMeters(-10.5), "cy": units.inchesToMeters(13.0),
        "cz": units.inchesToMeters(10.5), "pitch_rad": units.degreesToRadians(-13.0),
        "yaw_rad": math.pi / 2.0,
    },
    "limelight-front": {
        "id": 1, "cx": units.inchesToMeters(2.75), "cy": 0.0,
        "cz": units.inchesToMeters(16.25), "pitch_rad": 0.0, "yaw_rad": 0.0,
    },
    "limelight-right": {
        "id": 2, "cx": units.inchesToMeters(-10.375), "cy": units.inchesToMeters(-13.0),
        "cz": units.inchesToMeters(8.25), "pitch_rad": units.degreesToRadians(-16.0),
        "yaw_rad": -math.pi / 2.0,
    },
}

RF_PER_FID = 7


def _rotate(vx, vy, vz, pitch, yaw):
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    x1 = vx*cp + vz*sp; y1 = vy; z1 = -vx*sp + vz*cp
    return x1*cy - y1*sy, x1*sy + y1*cy, z1


def bearing_estimate_height_tri(tag_id, txnc_deg, tync_deg, cam, heading):
    """Bearing estimate using height triangulation (pipeline 0 mode)."""
    tag = APRIL_TAGS.get(tag_id)
    if tag is None: return None
    tag_x, tag_y, tag_z = tag
    txnc_rad = math.radians(txnc_deg); tync_rad = math.radians(tync_deg)
    dz = tag_z - cam["cz"]

    ll_Z = math.cos(tync_rad) * math.cos(txnc_rad)
    ll_X = math.cos(tync_rad) * math.sin(txnc_rad)
    ll_Y = math.sin(tync_rad)
    vx, vy, vz = _rotate(ll_Z, -ll_X, ll_Y, cam["pitch_rad"], cam["yaw_rad"])
    if abs(vz) < 0.005: return None
    scale = dz / vz
    if scale < 0.05 or scale > 30.0: return None
    d_h = scale * math.sqrt(vx*vx + vy*vy)
    if d_h < 0.1 or d_h > 15.0: return None

    bearing = heading + cam["yaw_rad"] - txnc_rad
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    dx_cam = cam["cx"]*cos_h - cam["cy"]*sin_h
    dy_cam = cam["cx"]*sin_h + cam["cy"]*cos_h
    rx = tag_x - d_h*math.cos(bearing) - dx_cam
    ry = tag_y - d_h*math.sin(bearing) - dy_cam
    return rx, ry, d_h, txnc_deg, tync_deg, tag_z


def combine(ests):
    if not ests: return None
    tw=wx=wy=0.0
    for rx, ry, dh, *_ in ests:
        w = 1.0/max(dh*dh, 1e-4); wx+=w*rx; wy+=w*ry; tw+=w
    return wx/tw, wy/tw


def _dbl(data):
    n = len(data)//8
    return list(struct.unpack_from(f"<{n}d", data)) if n else []

def _rot2d_rad(data):
    n = len(data)//8
    return [struct.unpack_from("<d", data, i*8)[0] for i in range(n)]

def _bisect(lst, ts):
    lo, hi = 0, len(lst)
    while lo < hi:
        mid = (lo+hi)//2
        if lst[mid][0] < ts: lo = mid+1
        else: hi = mid
    return lo

def _nearest(lst, ts, max_gap=None):
    if not lst: return None
    i = _bisect(lst, ts); i = max(0, min(i, len(lst)-1))
    if i > 0 and abs(lst[i-1][0]-ts) < abs(lst[i][0]-ts): i -= 1
    if max_gap is not None and abs(lst[i][0]-ts) > max_gap: return None
    return lst[i]


# ─── Extract training samples from one log ────────────────────────────────────
def extract_from_log(path):
    """Returns list of (features, target_dx, target_dy) tuples."""
    print(f"  Extracting from {os.path.basename(path)} ({os.path.getsize(path)/1e6:.0f}MB)...")
    reader = DataLogReader(path)
    eh = {}
    orig_list = []; ll_rf = defaultdict(list)

    for rec in reader:
        if rec.isControl():
            if rec.isStart():
                sd = rec.getStartData(); n, e = sd.name, sd.entry
                if n.endswith("Odometry/Robot"): eh[e] = "orig"
                else:
                    for cn in CAMERAS:
                        if n == f"NT:/{cn}/rawfiducials": eh[e] = ("rf", cn)
        else:
            e = rec.getEntry(); h = eh.get(e)
            if h is None: continue
            ts = rec.getTimestamp()/1e6; raw = rec.getRaw()
            if h == "orig":
                if len(raw) >= 24:
                    x, y, hh = struct.unpack_from("<3d", raw)
                    orig_list.append((ts, x, y, hh))
            elif isinstance(h, tuple) and h[0] == "rf":
                v = _dbl(raw)
                if v: ll_rf[h[1]].append((ts, v))

    orig_list.sort()
    if not orig_list:
        print("    No original poses — skipping.")
        return []

    samples = []
    for cn, cam in CAMERAS.items():
        cam_id = cam["id"]
        for rec_ts, fids in sorted(ll_rf[cn]):
            n_tags = len(fids) // RF_PER_FID
            if n_tags == 0: continue

            # Only pipeline 1 data (all tags have distToCamera > 0)
            all_have_dist = all(fids[i*7 + 4] > 0.01 for i in range(n_tags))
            if not all_have_dist: continue

            # Get reference pose
            orig = _nearest(orig_list, rec_ts, max_gap=0.05)
            if orig is None: continue
            _, ox, oy, heading = orig

            # Skip if reference is at origin (not yet localized)
            if abs(ox) < 0.01 and abs(oy) < 0.01: continue

            # Compute bearing estimate using height triangulation (simulating P0)
            tag_ests = []
            for i in range(n_tags):
                off = i * 7
                tag_id = int(fids[off])
                txnc = fids[off+1]; tync = fids[off+2]
                r = bearing_estimate_height_tri(tag_id, txnc, tync, cam, heading)
                if r is not None:
                    tag_ests.append(r)

            if not tag_ests: continue
            combined = combine(tag_ests)
            if combined is None: continue
            est_x, est_y = combined

            if not (-0.5 < est_x < 17.5 and -0.5 < est_y < 9.0): continue

            # Error (target for correction)
            dx = ox - est_x
            dy = oy - est_y
            err = math.hypot(dx, dy)
            if err > 5.0: continue  # skip gross outliers

            # Features for each tag estimate
            n_valid = len(tag_ests)
            avg_d_h = sum(e[2] for e in tag_ests) / n_valid
            avg_txnc_abs = sum(abs(e[3]) for e in tag_ests) / n_valid
            avg_tync = sum(e[4] for e in tag_ests) / n_valid
            avg_tag_z = sum(e[5] for e in tag_ests) / n_valid

            # Feature vector:
            # [cam0, cam1, cam2, avg_d_h, avg_d_h², 1/n_tags,
            #  avg_txnc_abs, avg_tync, avg_tag_z,
            #  sin(heading), cos(heading), est_x, est_y]
            cam_oh = [1.0 if cam_id == c else 0.0 for c in range(3)]
            feats = cam_oh + [
                avg_d_h, avg_d_h**2, 1.0/max(n_valid, 1),
                avg_txnc_abs, avg_tync, avg_tag_z,
                math.sin(heading), math.cos(heading),
                est_x, est_y,
            ]
            samples.append((feats, dx, dy))

    print(f"    {len(samples)} training samples")
    return samples


# ─── Model ────────────────────────────────────────────────────────────────────
N_FEATURES = 13

class CorrectionNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(N_FEATURES, 64),
            nn.BatchNorm1d(64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 16),
            nn.ReLU(),
            nn.Linear(16, 2),  # dx, dy correction
        )

    def forward(self, x):
        return self.net(x)


# ─── Main ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # Extract from all logs
    all_samples = []
    logs = sorted(glob.glob(os.path.join(LOG_DIR, "FRC_*.wpilog")))
    logs = [l for l in logs if not l.endswith(("_2d.wpilog", "_nn.wpilog"))]
    print(f"Found {len(logs)} source logs in {LOG_DIR}")

    for log_path in logs:
        try:
            samples = extract_from_log(log_path)
            all_samples.extend(samples)
        except Exception as e:
            print(f"    Error: {e}")

    print(f"\nTotal training samples: {len(all_samples)}")
    if len(all_samples) < 50:
        print("Not enough data to train. Need more pipeline 1 observations.")
        sys.exit(1)

    # Build tensors
    X = torch.tensor([s[0] for s in all_samples], dtype=torch.float32)
    Y = torch.tensor([[s[1], s[2]] for s in all_samples], dtype=torch.float32)

    print(f"Features shape: {X.shape}, Targets shape: {Y.shape}")
    print(f"Target stats: dx mean={Y[:,0].mean():.4f} std={Y[:,0].std():.4f}, "
          f"dy mean={Y[:,1].mean():.4f} std={Y[:,1].std():.4f}")
    print(f"Target error magnitude: mean={Y.norm(dim=1).mean():.4f}m "
          f"median={Y.norm(dim=1).median():.4f}m")

    # Train/val split
    n = len(X)
    perm = torch.randperm(n)
    n_val = max(int(n * 0.15), 10)
    X_val, Y_val = X[perm[:n_val]], Y[perm[:n_val]]
    X_train, Y_train = X[perm[n_val:]], Y[perm[n_val:]]
    print(f"Train: {len(X_train)}, Val: {len(X_val)}")

    # Train
    model = CorrectionNet()
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-5)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, patience=50, factor=0.5, min_lr=1e-5)

    best_val = float("inf")
    best_state = None
    BATCH = min(256, len(X_train))

    print("\nTraining...")
    for epoch in range(1000):
        model.train()
        idx = torch.randperm(len(X_train))
        epoch_loss = 0.0; n_batches = 0
        for i in range(0, len(X_train), BATCH):
            batch_idx = idx[i:i+BATCH]
            xb, yb = X_train[batch_idx], Y_train[batch_idx]
            pred = model(xb)
            loss = nn.functional.mse_loss(pred, yb)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item()
            n_batches += 1

        # Validation
        model.eval()
        with torch.no_grad():
            val_pred = model(X_val)
            val_loss = nn.functional.mse_loss(val_pred, Y_val).item()
            val_err = (val_pred - Y_val).norm(dim=1).mean().item()

            # Baseline: no correction (error magnitude)
            baseline_err = Y_val.norm(dim=1).mean().item()

        scheduler.step(val_loss)

        if val_loss < best_val:
            best_val = val_loss
            best_state = {k: v.clone() for k, v in model.state_dict().items()}

        if epoch % 100 == 0 or epoch == 999:
            train_loss = epoch_loss / max(n_batches, 1)
            print(f"  epoch {epoch:4d}: train_loss={train_loss:.6f} "
                  f"val_loss={val_loss:.6f} val_err={val_err:.4f}m "
                  f"(baseline={baseline_err:.4f}m)")

    # Load best model
    model.load_state_dict(best_state)
    model.eval()

    # Final evaluation
    with torch.no_grad():
        # Full dataset
        all_pred = model(X)
        corrected = torch.stack([
            X[:, 11] + all_pred[:, 0],  # est_x + dx
            X[:, 12] + all_pred[:, 1],  # est_y + dy
        ], dim=1)
        true_xy = torch.stack([
            X[:, 11] + Y[:, 0],
            X[:, 12] + Y[:, 1],
        ], dim=1)

        err_before = Y.norm(dim=1)
        err_after = (all_pred - Y).norm(dim=1)
        # Wait, corrected error should be |true - corrected|
        # true = est + Y, corrected = est + pred
        # err_after = |true - corrected| = |Y - pred|
        err_after = (Y - all_pred).norm(dim=1)

        print(f"\n=== Results on full dataset ({len(X)} samples) ===")
        print(f"  Before correction: mean={err_before.mean():.4f}m "
              f"median={err_before.median():.4f}m "
              f"p90={err_before.quantile(0.9):.4f}m")
        print(f"  After correction:  mean={err_after.mean():.4f}m "
              f"median={err_after.median():.4f}m "
              f"p90={err_after.quantile(0.9):.4f}m")
        print(f"  Improvement: {(1-err_after.mean()/err_before.mean())*100:.1f}%")

        # Per camera
        for cam_name, cam in CAMERAS.items():
            cid = cam["id"]
            mask = X[:, cid] > 0.5
            if mask.sum() == 0: continue
            eb = err_before[mask]; ea = err_after[mask]
            print(f"  {cam_name}: before={eb.mean():.4f}m → after={ea.mean():.4f}m "
                  f"({(1-ea.mean()/eb.mean())*100:.1f}%)")

    # Save as TorchScript
    scripted = torch.jit.script(model)
    scripted.save(MODEL_PATH)
    print(f"\nSaved model to {MODEL_PATH}")
