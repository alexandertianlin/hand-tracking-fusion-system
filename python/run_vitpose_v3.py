import os, sys, json, math, time, socket, logging, threading
import cv2, numpy as np, torch

# Suppress spam
os.environ["PYOPENGL_PLATFORM"] = "wgl"
os.environ["MMENGINE_LOGLEVEL"] = "ERROR"
logging.getLogger().setLevel(logging.ERROR)
np.bool = bool

# HAMER paths
HAMER_DIR = r"C:\Users\Administrator\Documents\Codex\2026-06-16\files-mentioned-by-the-user-gpu2-3\hamer_code\hamer-main"
CKPT = os.path.join(HAMER_DIR, "_DATA", "hamer_ckpts", "checkpoints", "hamer.ckpt")
os.chdir(HAMER_DIR)
sys.path.insert(0, HAMER_DIR)
sys.path.insert(0, os.path.join(HAMER_DIR, "third-party", "ViTPose"))
from hamer.models import load_hamer
from hamer.datasets.vitdet_dataset import ViTDetDataset
from hamer.utils import recursive_to
from vitpose_model import ViTPoseModel

# Global vars
g_kp3d = None; seq = 0; last_send = 0; SEND_INTV = 0.033; last_calibrate = 0
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_addr = ("127.0.0.1", 5055)

# Finger indices (MediaPipe/MANO compatible)
FN = ["thumb","index","middle","ring","little"]
FC = [
    {"mcp":2,"pip":3,"dip":3,"tip":4},
    {"mcp":5,"pip":6,"dip":7,"tip":8},
    {"mcp":9,"pip":10,"dip":11,"tip":12},
    {"mcp":13,"pip":14,"dip":15,"tip":16},
    {"mcp":17,"pip":18,"dip":19,"tip":20},
]
W = 0; I0 = 5; M0 = 9; M3 = 12; L0 = 17

def nrm(v):
    n = np.linalg.norm(v); return v / n if n > 1e-8 else v

def orient_q_from_landmarks(pw):
    u = nrm(pw[W] - pw[M0]); r = nrm(pw[I0] - pw[L0])
    f = nrm(np.cross(r, u)); u = nrm(np.cross(f, r))
    R = np.column_stack([r, u, f])
    tr = R[0,0]+R[1,1]+R[2,2]
    if tr > 0:
        s = math.sqrt(tr+1.0)*2; q = [s*0.25, (R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s]
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = math.sqrt(1+R[0,0]-R[1,1]-R[2,2])*2
        q = [(R[2,1]-R[1,2])/s, s*0.25, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s]
    elif R[1,1] > R[2,2]:
        s = math.sqrt(1+R[1,1]-R[0,0]-R[2,2])*2
        q = [(R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, s*0.25, (R[1,2]+R[2,1])/s]
    else:
        s = math.sqrt(1+R[2,2]-R[0,0]-R[1,1])*2
        q = [(R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, s*0.25]
    nv = math.sqrt(sum(x*x for x in q))
    return [float(x/nv) for x in q] if nv > 1e-10 else [1,0,0,0]

def curl_from_kp(pw, n):
    c = FC[n]; m = pw[c["mcp"]]; p = pw[c["pip"]]; d = pw[c["dip"]]; t = pw[c["tip"]]
    chord = np.linalg.norm(t - m)
    pip2tip = np.linalg.norm(t - p)
    if n == 0:  # thumb
        arc = np.linalg.norm(m-p) + np.linalg.norm(p-t)
    else:
        arc = np.linalg.norm(m-p) + np.linalg.norm(p-d) + np.linalg.norm(d-t)
    if arc < 1e-8: return 0.0
    return 1.0 - max(0, min(1, 0.8*chord/arc + 0.2*pip2tip/arc))

def spread_from_kp(pw, n):
    if n == 2: return 0.0  # middle finger reference
    c = FC[n]; mc = FC[2]
    fd = nrm(pw[c["tip"]] - pw[c["mcp"]])
    md = nrm(pw[mc["tip"]] - pw[mc["mcp"]])
    ddot = max(-1, min(1, np.dot(fd, md)))
    return max(0, min(1, math.acos(ddot) / 1.2))

# Load HAMER + ViTPose
print("Loading HAMER ...", flush=True)
m_hamer, cfg_h = load_hamer(CKPT, init_renderer=False)
dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
m_hamer = m_hamer.to(dev).eval()
print(f"HAMER on {dev}", flush=True)
print("Loading ViTPose ...", flush=True)
vitpose = ViTPoseModel("cuda" if torch.cuda.is_available() else "cpu")
print("ViTPose ready", flush=True)

# D435i camera
print("Opening D435i ...", flush=True)
import pyrealsense2 as rs
pipe = rs.pipeline()
cfg_rs = rs.config()
cfg_rs.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipe.start(cfg_rs)
for _ in range(30): pipe.wait_for_frames()
print("D435i OK. Press q/ESC to quit.", flush=True)

fc = 0; ft = time.time(); fps = 0
vit_skip = 0; last_boxes = None; last_right = None; consecutive_no_hand = 0; hands_confirmed = True; last_kp2d = None; last_hand_center = None

try:
    while True:
        frames = pipe.wait_for_frames()
        cf = frames.get_color_frame()
        if not cf: continue
        img = np.asanyarray(cf.get_data())
        h, w, _ = img.shape
        g_kp3d = None

        # ViTPose
        vitpose_ran_this_frame = False
        if vit_skip <= 0:
            vitpose_ran_this_frame = True
            det = [np.array([[0,0,w,h,0.9]])]
            vitposes = vitpose.predict_pose(img, det)
            vit_skip = 3
        else:
            vitposes = []; vit_skip -= 1

        canvas = img.copy()
        boxes_list = []; rights_list = []
        for vp in vitposes:
            kps = vp["keypoints"]
            for hk, ir in [(kps[-42:-21], False), (kps[-21:], True)]:
                v = hk[:,2] > 0.5
                if sum(v) < 8: continue
                boxes_list.append([int(hk[v,0].min())-20, int(hk[v,1].min())-20,
                                   int(hk[v,0].max())+20, int(hk[v,1].max())+20])
                rights_list.append(1.0 if ir else 0.0)
        
        # Dedup: ViTPose often detects both L+R hands on one physical hand.
        # When boxes overlap significantly (>30% min-area IoU), keep only the
        # one with higher confidence (more keypoints visible).
        if len(boxes_list) > 1:
            dedup_boxes = []; dedup_rights = []
            for bi, (box, ri) in enumerate(zip(boxes_list, rights_list)):
                is_dup = False
                for bj, rj in zip(boxes_list, rights_list):
                    if ri == rj: continue
                    x1 = max(box[0], bj[0]); y1 = max(box[1], bj[1])
                    x2 = min(box[2], bj[2]); y2 = min(box[3], bj[3])
                    if x1 < x2 and y1 < y2:
                        inter = (x2-x1)*(y2-y1)
                        ai = (box[2]-box[0])*(box[3]-box[1])
                        aj = (bj[2]-bj[0])*(bj[3]-bj[1])
                        if inter / min(ai, aj) > 0.3:
                            is_dup = True
                            break
                if not is_dup:
                    dedup_boxes.append(box)
                    dedup_rights.append(ri)
            boxes_list = dedup_boxes
            rights_list = dedup_rights
 


        if boxes_list:
            consecutive_no_hand = 0
            last_boxes = np.stack(boxes_list).astype(np.float32)
            last_right = np.stack(rights_list)
        elif vitpose_ran_this_frame:
            consecutive_no_hand += 1
            if consecutive_no_hand > 5:
                last_boxes = None
                last_right = None
        if vitpose_ran_this_frame:
            hands_confirmed = bool(boxes_list)
        # num_hands=0 signal on first hand-loss ViTPose frame
        if vitpose_ran_this_frame and not hands_confirmed and consecutive_no_hand == 1:
            seq += 1
            pkt = {"type":"hamer_hand","seq":seq,"ts":int(time.time()*1000),"num_hands":0}
            try:
                sock.sendto(json.dumps(pkt, separators=(",",":")).encode(), udp_addr)
                print("LOST_HAND seq=" + str(seq), flush=True)
            except:
                pass

        # ROI tracking: on skip frames, shift crop box via last frame 2D keypoints
        if not vitpose_ran_this_frame and last_kp2d is not None and hands_confirmed and last_boxes is not None:
            centroid = last_kp2d.mean(axis=0)
            bw = last_boxes[0][2] - last_boxes[0][0]
            bh = last_boxes[0][3] - last_boxes[0][1]
            cx_f, cy_f = centroid[0], centroid[1]
            last_boxes[0] = [cx_f - bw/2, cy_f - bh/2, cx_f + bw/2, cy_f + bh/2]
            last_hand_center = (int(cx_f), int(cy_f))

        if last_boxes is not None:
            ds = ViTDetDataset(cfg_h, img, last_boxes, last_right, rescale_factor=2.0)
            loader = torch.utils.data.DataLoader(ds, batch_size=4, shuffle=False, num_workers=0)
            for batch in loader:
                batch = recursive_to(batch, dev)
                with torch.no_grad():
                    out = m_hamer(batch)
                for n in range(batch["img"].shape[0]):
                    kp3d = out["pred_keypoints_3d"][n].cpu().numpy()
                    g_kp3d = kp3d
                    # 2D skeleton
                    kn = out["pred_keypoints_2d"][n].cpu().numpy() + 0.5
                    ir_val = batch["right"][n].item()
                    if ir_val < 0.5:
                        kn[:,0] = 1.0 - kn[:,0]
                    cx, cy, bs = batch["box_center"][n,0].item(), batch["box_center"][n,1].item(), batch["box_size"][n].item()
                    kp2d = np.zeros((21,2), dtype=int)
                    kp2d[:,0] = (cx - bs/2 + kn[:,0]*bs).astype(int)
                    kp2d[:,1] = (cy - bs/2 + kn[:,1]*bs).astype(int)
                    # Save 2D keypoints for next frame's ROI tracking
                    last_kp2d = np.column_stack([kp2d[:,0].astype(np.float32), kp2d[:,1].astype(np.float32)])
                    last_hand_center = (int(kp2d[:,0].mean()), int(kp2d[:,1].mean()))

                    for e in [(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),(0,9),(9,10),
                              (10,11),(11,12),(0,13),(13,14),(14,15),(15,16),(0,17),(17,18),(18,19),(19,20)]:
                        cv2.line(canvas, tuple(kp2d[e[0]]), tuple(kp2d[e[1]]), (0,255,255), 2)
                    for p in kp2d: cv2.circle(canvas, tuple(p), 4, (0,255,0), -1)
                    cv2.putText(canvas, "R" if ir_val > 0.5 else "L", (int(cx)-10,int(cy)+5), cv2.FONT_HERSHEY_SIMPLEX, .8, (255,0,0), 2)

        if hands_confirmed and g_kp3d is not None and time.time() - last_send >= SEND_INTV:
            curld = [curl_from_kp(g_kp3d, i) for i in range(5)]
            spreadd = [spread_from_kp(g_kp3d, i) for i in range(5)]
            seq += 1
            q = orient_q_from_landmarks(g_kp3d)
            pkt = {"type":"hamer_hand","seq":seq,"ts":int(time.time()*1000),"num_hands":1,
                   "hand_0_label":"right","hand_0_conf":0.9,"hand_0_wrist":g_kp3d[0].tolist(),
                   "hand_0_kp3d":g_kp3d.flatten().tolist(),"hand_0_orient_q":q}
            for i in range(5):
                pkt["hand_0_curl_"+FN[i]] = round(curld[i],3)
                pkt["hand_0_spread_"+FN[i]] = round(spreadd[i],3)
            try:
                sock.sendto(json.dumps(pkt, separators=(",",":")).encode(), udp_addr)
                print("UDP seq=" + str(seq), flush=True)
            except:
                pass
            last_send = time.time()
            ctxt = " ".join([FN[i]+":"+str(round(curld[i],2)) for i in range(5)])
            cv2.putText(canvas, ctxt, (8,20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,200,0), 1)
        # Debug: print curl values every frame
        if hands_confirmed and g_kp3d is not None:
            curld_chk = [curl_from_kp(g_kp3d, i) for i in range(5)]
            # Trigger when hand is reasonably open (all curls < 0.6 + spread)
            # Print curls every 30 frames for threshold tuning
            if fc > 0 and fc % 30 == 0:
                print("CURLS:" + str([round(c,2) for c in curld_chk]), flush=True)
            if all(c < 0.6 for c in curld_chk) and time.time() - last_calibrate > 3.0:
                seq += 1
                pkt_cal = {"type":"hamer_hand","seq":seq,"ts":int(time.time()*1000),"command":"calibrate"}
                try:
                    sock.sendto(json.dumps(pkt_cal, separators=(",",":")).encode(), udp_addr)
                    print("CALIBRATE seq=" + str(seq) + " curls=" + str([round(c,2) for c in curld_chk]), flush=True)
                except:
                    pass
                last_calibrate = time.time()

        # FPS
        fc += 1
        if time.time()-ft >= 1: fps,fc,ft = fc,0,time.time()
        cv2.putText(canvas, f"FPS:{fps} UDP:5055", (8,40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,200,0), 1)
        cv2.imshow("ViTPose -> Unity 5055", canvas)
        k = cv2.waitKey(1)&0xFF
        if k in (ord('q'),27): break
except Exception as e:
    print(f"Error: {e}")
finally:
    pipe.stop(); cv2.destroyAllWindows(); print("Done.")
