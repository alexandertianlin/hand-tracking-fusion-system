"""
D435i + MediaPipe Hand to Unity UDP Bridge (v2.0)

Usage:
    python src/d435i_mediapipe_hand_sender.py
    python src/d435i_mediapipe_hand_sender.py --webcam 0
"""
import json, math, os, socket, sys, time, urllib.request
import cv2, numpy as np
rs = None
try: import pyrealsense2 as _rs; rs = _rs
except: pass
import mediapipe as mp
UNITY_IP = "127.0.0.1"; PORT = 5055
MIN_CONF = 0.6; MIN_TRACK = 0.5; SEND_INTV = 0.033
SHOW_PREV = True
MDIR = os.path.join(os.path.dirname(__file__), "..", "..", "models")
MFILE = "hand_landmarker.task"
MURL = ("https://storage.googleapis.com/mediapipe-models/"
        "hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task")
W = 0; _T = (1,2,3,4); _I = (5,6,7,8); _M = (9,10,11,12); _R = (13,14,15,16); _L = (17,18,19,20)
FN = ["thumb","index","middle","ring","little"]
FC = {"thumb":{"mcp":_T[1],"pip":_T[2],"tip":_T[3]},
    "index":{"mcp":_I[0],"pip":_I[1],"dip":_I[2],"tip":_I[3]},
    "middle":{"mcp":_M[0],"pip":_M[1],"dip":_M[2],"tip":_M[3]},
    "ring":{"mcp":_R[0],"pip":_R[1],"dip":_R[2],"tip":_R[3]},
    "little":{"mcp":_L[0],"pip":_L[1],"dip":_L[2],"tip":_L[3]}}

def nrm(v):
    n = np.linalg.norm(v); return v / n if n > 1e-8 else v

def orient_q(pw):
    u = nrm(pw[W] - pw[_M[0]]); r = nrm(pw[_I[0]] - pw[_L[0]])
    f = nrm(np.cross(r, u)); u = nrm(np.cross(f, r))
    R = np.column_stack([r, u, f])
    qw = math.sqrt(max(0, 1 + R[0,0] + R[1,1] + R[2,2])) / 2.0
    qx = math.sqrt(max(0, 1 + R[0,0] - R[1,1] - R[2,2])) / 2.0
    qy = math.sqrt(max(0, 1 - R[0,0] + R[1,1] - R[2,2])) / 2.0
    qz = math.sqrt(max(0, 1 - R[0,0] - R[1,1] + R[2,2])) / 2.0
    qx = qx if R[2,1] >= R[1,2] else qx * -1
    qy = qy if R[0,2] >= R[2,0] else qy * -1
    qz = qz if R[1,0] >= R[0,1] else qz * -1
    return [float(q) for q in nrm(np.array([qw, qx, qy, qz]))]

def curl(pw, n):
    c = FC[n]; m = pw[c["mcp"]]; p = pw[c["pip"]]; t = pw[c["tip"]]
    if "dip" in c:
        d = pw[c["dip"]]; ch = max(1e-6, np.linalg.norm(m-p) + np.linalg.norm(p-d) + np.linalg.norm(d-t))
    else:
        ch = max(1e-6, np.linalg.norm(m-p) + np.linalg.norm(p-t))
    return 1.0 - np.clip(0.8 * np.linalg.norm(t-m) / ch + 0.2 * np.linalg.norm(t-p) / ch, 0, 1)

def spread(pw, n):
    c = FC[n]; m = pw[c["mcp"]]; t = pw[c["tip"]]
    df = nrm(t - m); dm = nrm(pw[_M[3]] - pw[_M[0]])
    return float(np.clip(math.acos(max(-1, min(1, np.dot(df, dm)))) / 1.2, 0, 1))

_seq = 0
def build_pkt(hl, hw, label, score, ts):
    global _seq; _seq += 1
    kp = []; [kp.extend([float(l.x), float(l.y), float(l.z)]) for l in hw]
    pw = np.array([[l.x, l.y, l.z] for l in hw])
    cu = {}; sp = {}
    for n in FN: cu[n] = float(curl(pw, n)); sp[n] = float(spread(pw, n))
    oq = orient_q(pw); wr = [float(hw[W].x), float(hw[W].y), float(hw[W].z)]
    return {"type":"hamer_hand","seq":_seq,"ts":ts,"num_hands":1,
        "hand_0_label":label.lower(),"hand_0_conf":float(score),
        "hand_0_wrist":wr,"hand_0_kp3d":kp,"hand_0_orient_q":oq,
        "hand_0_curl_thumb":cu["thumb"],"hand_0_curl_index":cu["index"],
        "hand_0_curl_middle":cu["middle"],"hand_0_curl_ring":cu["ring"],
        "hand_0_curl_little":cu["little"],
        "hand_0_spread_thumb":sp["thumb"],"hand_0_spread_index":sp["index"],
        "hand_0_spread_middle":sp["middle"],"hand_0_spread_ring":sp["ring"],
        "hand_0_spread_little":sp["little"]}

def send(sock, pkt):
    try: sock.sendto(json.dumps(pkt,separators=(",",":")).encode(), (UNITY_IP, PORT))
    except: pass

HC = [(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),
    (0,9),(9,10),(10,11),(11,12),(0,13),(13,14),(14,15),(15,16),
    (0,17),(17,18),(18,19),(19,20),(5,9),(9,13),(13,17)]

def draw_hl(frame, lm, h, w):
    pt = [(int(l.x*w), int(l.y*h)) for l in lm]
    for a, b in HC:
        if a < len(pt) and b < len(pt): cv2.line(frame, pt[a], pt[b], (0,255,0), 2)
    for p in pt: cv2.circle(frame, p, 4, (0,255,0), cv2.FILLED)

def ensure_model():
    d = MDIR; os.makedirs(d, exist_ok=True); p = os.path.join(d, MFILE)
    if not os.path.exists(p):
        print("  Downloading model...", flush=True); urllib.request.urlretrieve(MURL, p)
    return p

class D435IBackend:
    def __init__(self): self.pipe = None
    def open(self):
        for attempt in range(2):
            print('  Attempt {}/2: creating pipeline...'.format(attempt+1), flush=True)
            cfg = rs.config(); cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipe = rs.pipeline(); self.pipe.start(cfg)
            print('  Attempt {}/2: waiting for frames...'.format(attempt+1), flush=True)
            t0 = time.monotonic()
            for i in range(3):
                try:
                    frames = self.pipe.wait_for_frames(timeout_ms=5000)
                    cf = frames.get_color_frame()
                    if cf:
                        np.asanyarray(cf.get_data())
                        elapsed = time.monotonic()-t0
                        print('  D435i OK ({:.1f}s)'.format(elapsed), flush=True)
                        return True
                    else:
                        print('  Attempt {}/{}: frame #{}, no color stream'.format(attempt+1, i+1), flush=True)
                except RuntimeError:
                    print('  Attempt {0}/{1}: frame timeout ({2:.1f}s)'.format(attempt+1, i+1, time.monotonic()-t0), flush=True)
            print('  Attempt {}/2: stopping pipeline...'.format(attempt+1), flush=True)
            try: self.pipe.stop()
            except: print('  Attempt {}/2: stop failed'.format(attempt+1), flush=True)
            self.pipe = None
            if attempt < 1:
                print('  Waiting 1s before retry...', flush=True)
                time.sleep(1)
        print('  D435i FAILED after 2 attempts', flush=True)
        return False
    def read(self):
        try:
            frames = self.pipe.wait_for_frames(timeout_ms=5000)
            cf = frames.get_color_frame()
            return np.asanyarray(cf.get_data()) if cf else None
        except: return None

class WebcamBackend:
    def __init__(self, cam_id=0): self.cap = None; self.cam_id = cam_id
    def open(self):
        self.cap = cv2.VideoCapture(self.cam_id, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, _ = self.cap.read(); return ok
    def read(self):
        if self.cap is None: return None
        ok, frame = self.cap.read(); return frame if ok else None
    def close(self):
        if self.cap: self.cap.release()

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--webcam", nargs="?", const=0, type=int, default=None)
    ap.add_argument("--port", type=int, default=5055)
    args = ap.parse_args()

    global PORT
    PORT = args.port

    print("=" * 60)
    print("MediaPipe Hand to Unity UDP Bridge")
    print("=" * 60)
    print()

    if args.webcam is not None:
        print("Mode: Webcam (ID={})".format(args.webcam))
        cam = WebcamBackend(args.webcam)
    elif rs is not None:
        print("Mode: D435i")
        cam = D435IBackend()
    else:
        print("D435i n/a, trying webcam 0...")
        cam = WebcamBackend(0)

    print("Opening camera...", flush=True)
    if not cam.open():
        print("[FAIL] Camera open failed", flush=True)
        print("  Check USB connection (D435i) or camera ID (--webcam N)", flush=True)
        sys.exit(1)

    mp_path = ensure_model()
    HL = mp.tasks.vision.HandLandmarker
    HLO = mp.tasks.vision.HandLandmarkerOptions
    BO = mp.tasks.BaseOptions
    RM = mp.tasks.vision.RunningMode
    print("Starting HandLandmarker...", flush=True)

    opts = HLO(
        base_options=BO(model_asset_path=mp_path),
        running_mode=RM.VIDEO, num_hands=1,
        min_hand_detection_confidence=MIN_CONF,
        min_tracking_confidence=MIN_TRACK,
    )
    lmr = HL.create_from_options(opts)
    print("  HandLandmarker OK", flush=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("  UDP -> {}:{}".format(UNITY_IP, PORT), flush=True)

    last_send = 0.0; fc = 0; fps_t = time.monotonic(); fps_d = 0.0
    show = SHOW_PREV; send_on = True; ts = 0
    print("\n  ESC/Q=quit  R=preview  S=send\n-- Running --\n", flush=True)

    try:
        while True:
            frame = cam.read()
            if frame is None:
                print("x", end="", flush=True); continue
            fc += 1; now = time.monotonic(); ts += 1
            h, w, _ = frame.shape

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = lmr.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)

            disp = frame.copy()
            if res.hand_landmarks and res.hand_world_landmarks:
                hl = res.hand_landmarks[0]; hw = res.hand_world_landmarks[0]
                label = res.handedness[0][0].category_name if res.handedness else "Right"
                score = res.handedness[0][0].score if res.handedness else 1.0
                draw_hl(disp, hl, h, w)
                if send_on and (now - last_send) >= SEND_INTV:
                    pkt = build_pkt(hl, hw, label, score, int(now * 1000))
                    send(sock, pkt); last_send = now
                    if fc % 60 == 0:
                        cstr = ", ".join("{:.2f}".format(pkt["hand_0_curl_" + n]) for n in FN)
                        sstr = ", ".join("{:.2f}".format(pkt["hand_0_spread_" + n]) for n in FN)
                        print("  curl=[{}]  spread=[{}]  conf={:.2f}  fps={:.1f}".format(cstr, sstr, score, fps_d), flush=True)
            if now - fps_t >= 1.0:
                fps_d = fc / (now - fps_t); fc = 0; fps_t = now
            if show:
                st = "ON" if send_on else "PAUSED"
                cv2.putText(disp, "FPS:{:.1f} S:{}".format(fps_d, st), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("MP->Unity", disp)
                k = cv2.waitKey(1) & 0xFF
                if k in (27, ord('q')): break
                elif k == ord('r'): show = not show; cv2.destroyWindow("MP->Unity")
                elif k == ord('s'): send_on = not send_on; print(" >Send:{}".format("ON" if send_on else "OFF"), flush=True)
            else:
                if cv2.waitKey(1) & 0xFF in (27, ord('q')): break
    except KeyboardInterrupt:
        print("\nInterrupted", flush=True)
    finally:
        print("Cleanup...", flush=True)
        lmr.close(); cam.close(); sock.close(); cv2.destroyAllWindows()
        print("Done.", flush=True)

if __name__ == "__main__": main()
