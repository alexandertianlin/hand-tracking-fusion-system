"""
HAMER → Unity UDP Bridge
Architecture:
  Camera → ViTPose → HAMER → 21 3D keypoints + MANO params → UDP JSON → Unity (onlytip)
  IMU Glove → (future) → fusion at Python level → Unity
"""

import os, sys, time, json, socket
import cv2
import numpy as np
import torch

# ─── Config ────────────────────────────────────────────────────────────
UNITY_IP = "127.0.0.1"
UNITY_PORT = 5055
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
VITPOSE_SKIP = 5  # Run ViTPose every N frames

# Paths
HAMER_DIR = r"C:\Users\Administrator\Documents\Codex\2026-06-16\files-mentioned-by-the-user-gpu2-3\hamer_code\hamer-main"
CKPT = os.path.join(HAMER_DIR, "_DATA", "hamer_ckpts", "checkpoints", "hamer.ckpt")

FINGER_JOINTS = {
    "thumb":  [1, 2, 3, 4],
    "index":  [5, 6, 7, 8],
    "middle": [9, 10, 11, 12],
    "ring":   [13, 14, 15, 16],
    "little": [17, 18, 19, 20],
}

# ─── Setup ─────────────────────────────────────────────────────────────
os.environ["PYOPENGL_PLATFORM"] = "wgl"
np.bool = bool
os.chdir(HAMER_DIR)
sys.path.insert(0, HAMER_DIR)
sys.path.insert(0, os.path.join(HAMER_DIR, "third-party", "ViTPose"))
from hamer.models import load_hamer
from hamer.datasets.vitdet_dataset import ViTDetDataset
from hamer.utils import recursive_to
from vitpose_model import ViTPoseModel

print("[Bridge] Loading HAMER ...")
model, cfg = load_hamer(CKPT, init_renderer=False)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = model.to(device).eval()
print(f"[Bridge] HAMER on {device}")

print("[Bridge] Loading ViTPose ...")
vitpose = ViTPoseModel("cuda")
print("[Bridge] ViTPose ready")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_json(data: dict):
    msg = json.dumps(data)
    sock.sendto(msg.encode("utf-8"), (UNITY_IP, UNITY_PORT))

HAND_EDGES = [(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),(0,9),(9,10),
              (10,11),(11,12),(0,13),(13,14),(14,15),(15,16),(0,17),(17,18),(18,19),(19,20)]

# ─── Finger Curl Estimation ───────────────────────────────────────────
def compute_curls(kp3d):
    """Each finger: curl(0-1), spread(0-1) from MANO 21x3 keypoints."""
    result = {}
    for name, jj in FINGER_JOINTS.items():
        a, b, c, d = kp3d[jj[0]], kp3d[jj[1]], kp3d[jj[2]], kp3d[jj[3]]
        v1, v2, v3 = b-a, c-b, d-c
        cos1 = np.clip(np.dot(v1,v2)/(np.linalg.norm(v1)+1e-8)/(np.linalg.norm(v2)+1e-8), -1, 1)
        cos2 = np.clip(np.dot(v2,v3)/(np.linalg.norm(v2)+1e-8)/(np.linalg.norm(v3)+1e-8), -1, 1)
        curl = float((np.arccos(cos1) + np.arccos(cos2)) / (2*np.pi))
        # Spread
        wrist = kp3d[0]
        pn = np.cross(kp3d[5]-wrist, kp3d[17]-wrist)
        pnl = np.linalg.norm(pn)
        if pnl > 1e-6:
            md = kp3d[jj[0]] - wrist
            ml = np.linalg.norm(md)
            spread = float(np.linalg.norm(np.cross(md/ml, pn/pnl))) * 2 if ml > 1e-6 else 0.0
        else:
            spread = 0.0
        result[name] = {"curl": min(1,max(0,curl)), "spread": min(1,max(0,spread))}
    return result

def aa2q(aa):
    """Convert axis-angle (3,) to quaternion [x,y,z,w] for Unity."""
    aa = np.asarray(aa).ravel()
    angle = float(np.linalg.norm(aa))
    if angle < 1e-8:
        return [0.0, 0.0, 0.0, 1.0]
    axis = aa / angle
    s = np.sin(angle / 2)
    c = np.cos(angle / 2)
    return [float(axis[0]) * float(s), float(axis[1]) * float(s), float(axis[2]) * float(s), float(c)]

def rot2q(r):
    qw = np.sqrt(1 + r[0,0] + r[1,1] + r[2,2]) / 2
    return [float(x) for x in [qw, (r[2,1]-r[1,2])/(4*qw+1e-8),
            (r[0,2]-r[2,0])/(4*qw+1e-8), (r[1,0]-r[0,1])/(4*qw+1e-8)]]

def pack_hand(idx, is_right, kp3d, orient, conf, hand_pose=None):
    curls = compute_curls(kp3d)
    p = f"hand_{idx}"
    d = {f"{p}_label": "right" if is_right else "left",
         f"{p}_conf": float(conf),
         f"{p}_wrist": [float(kp3d[0,j]) for j in range(3)],
         f"{p}_kp3d": [float(v) for v in kp3d.flatten()],
         f"{p}_orient_q": rot2q(orient[0].cpu().numpy())}
    for fn in ["thumb","index","middle","ring","little"]:
        d[f"{p}_curl_{fn}"] = curls[fn]["curl"]
        d[f"{p}_spread_{fn}"] = curls[fn]["spread"]
    # Per-joint quaternions from MANO hand_pose (45 = 15 joints x 3 axis-angle)
    if hand_pose is not None:
        jq = []
        for j in range(15):
            jq.extend(aa2q(hand_pose[j*3:(j+1)*3]))
        d[f"{p}_joint_q"] = jq
    return d

class IMUFusion:
    """Placeholder: future STM32 IMU + HAMER fusion."""
    def __init__(self): self.enabled = False
    def fuse(self, d): return d

# ─── Main ─────────────────────────────────────────────────────────────
def main():
    imu = IMUFusion()
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened(): cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not cap.isOpened(): print("[Bridge] No camera!"); return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    print(f"[Bridge] Camera OK -> UDP {UNITY_IP}:{UNITY_PORT}"); sys.stdout.flush()

    fc = 0; ft = time.time(); fps = 0; vit_skip = 0; seq = 0
    last_boxes = None; last_right = None

    while cap.isOpened():
        t0 = time.time()
        ok, frame = cap.read()
        if not ok: break
        h, w = frame.shape[:2]

        # ViTPose
        if vit_skip <= 0:
            vitposes = vitpose.predict_pose(frame, [np.array([[0,0,w,h,0.9]])])
            vit_skip = VITPOSE_SKIP
        else:
            vitposes = []; vit_skip -= 1

        canvas = frame.copy()
        boxes_list = []; right_list = []
        for vp in vitposes:
            kps = vp["keypoints"]
            for hk, ir in [(kps[-42:-21], False), (kps[-21:], True)]:
                v = hk[:,2] > 0.5
                if sum(v) < 8: continue
                boxes_list.append([int(hk[v,0].min())-20, int(hk[v,1].min())-20,
                                   int(hk[v,0].max())+20, int(hk[v,1].max())+20])
                right_list.append(1.0 if ir else 0.0)

        if boxes_list:
            boxes = np.stack(boxes_list).astype(np.float32)
            right = np.stack(right_list)
            last_boxes, last_right = boxes, right
        elif last_boxes is not None:
            boxes, right = last_boxes, last_right
        else:
            boxes = None

        hamer_start = time.time(); hands_data = []
        if boxes is not None:
            ds = ViTDetDataset(cfg, frame, boxes, right, rescale_factor=2.0)
            loader = torch.utils.data.DataLoader(ds, batch_size=4, shuffle=False, num_workers=0)
            for batch in loader:
                batch = recursive_to(batch, device)
                with torch.no_grad(): out = model(batch)
                for n in range(batch["img"].shape[0]):
                    kp2d_crop = out["pred_keypoints_2d"][n].cpu().numpy()
                    kp3d = out["pred_keypoints_3d"][n].cpu().numpy()
                    mp = out["pred_mano_params"]
                    orient = mp["global_orient"][n]
                    is_right = batch["right"][n].item() > 0.5
                    conf = abs(batch["right"][n].item() - 0.5) * 2

                    kc = kp2d_crop + 0.5
                    if not is_right: kc[:,0] = 1.0 - kc[:,0]
                    cx = batch["box_center"][n,0].item()
                    cy = batch["box_center"][n,1].item()
                    bs = batch["box_size"][n].item()
                    kp2d = np.zeros((21,2), dtype=int)
                    kp2d[:,0] = (cx - bs/2 + kc[:,0]*bs).astype(int)
                    kp2d[:,1] = (cy - bs/2 + kc[:,1]*bs).astype(int)

                    for e in HAND_EDGES:
                        cv2.line(canvas, tuple(kp2d[e[0]]), tuple(kp2d[e[1]]), (0,255,255), 2)
                    for kp in kp2d: cv2.circle(canvas, tuple(kp), 4, (0,255,0), -1)
                    col = (255,0,0) if is_right else (0,255,0)
                    x1=int(cx-bs/2); y1=int(cy-bs/2); x2=int(cx+bs/2); y2=int(cy+bs/2)
                    cv2.rectangle(canvas, (x1,y1),(x2,y2), col, 2)
                    cv2.putText(canvas, "R" if is_right else "L", (int(cx)-10, int(cy)+5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)

                    hd = pack_hand(len(hands_data), is_right, kp3d, orient, conf, mp["hand_pose"][n].cpu().numpy())
                    hands_data.append(hd)
                    ct = " ".join(f"{fn[0]}={hd[f'hand_{len(hands_data)-1}_curl_{fn}']:.2f}"
                                  for fn in ["thumb","index","middle","ring","little"])
                    cv2.putText(canvas, ct, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)

        th = time.time() - hamer_start

        # UDP
        seq += 1
        pkt = {"type":"hamer_hand", "seq":seq, "ts":int(time.time()*1000), "num_hands":len(hands_data)}
        for hd in hands_data: pkt.update(hd)
        send_json(pkt)

        canvas = cv2.flip(canvas, 1)
        fc += 1
        if time.time() - ft >= 1: fps, fc, ft = fc, 0, time.time()
        cv2.putText(canvas, f"FPS:{fps} H:{th:.2f}s Hands:{len(hands_data)}",
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,200,0), 2)
        cv2.imshow("HAMER -> Unity Bridge", canvas)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27): break

    cap.release(); cv2.destroyAllWindows(); sock.close()
    print("[Bridge] Stopped.")

if __name__ == "__main__":
    main()
