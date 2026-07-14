"""D435i + ViTPose + HAMER + IMU - 非阻塞式数据画布融合

以 d435i_hamer_vitpose.py 为基底，零侵入式叠加 IMU 数据显示。
IMU 在独立后台线程更新全局变量，主线程仅做 cv2.putText 绘制，永不阻塞。
"""
import os, time, cv2, numpy as np, torch, sys, pyrealsense2 as rs
import threading, struct, socket, json

os.environ["PYOPENGL_PLATFORM"] = "wgl"
os.environ["MMENGINE_LOGLEVEL"] = "ERROR"
np.bool = bool

HAMER_DIR = r"C:\Users\Administrator\Documents\Codex\2026-06-16\files-mentioned-by-the-user-gpu2-3\hamer_code\hamer-main"
CKPT = os.path.join(HAMER_DIR, "_DATA", "hamer_ckpts", "checkpoints", "hamer.ckpt")

os.chdir(HAMER_DIR)
sys.path.insert(0, HAMER_DIR)
sys.path.insert(0, os.path.join(HAMER_DIR, "third-party", "ViTPose"))

from hamer.models import load_hamer
from hamer.datasets.vitdet_dataset import ViTDetDataset
from hamer.utils import recursive_to
from vitpose_model import ViTPoseModel

# ========== [IMU 非阻塞数据槽] ==========
class ImuSlot:
    __slots__ = ('qw', 'qx', 'qy', 'qz', 'ts', 'valid')
    def __init__(self):
        self.qw, self.qx, self.qy, self.qz = 1.0, 0.0, 0.0, 0.0
        self.ts = 0.0
        self.valid = False

imu_data = ImuSlot()

# ViTPose 21 hand keypoints grouped by finger (indices 0-20)
# 0=wrist, 1-4=thumb, 5-8=index, 9-12=middle, 13-16=ring, 17-20=little
PER_FINGER_KP = {
    "thumb":  [1, 2, 3, 4],
    "index":  [5, 6, 7, 8],
    "middle": [9, 10, 11, 12],
    "ring":   [13, 14, 15, 16],
    "little": [17, 18, 19, 20],
}
FINGER_NAMES = ["thumb", "index", "middle", "ring", "little"]

# MANO 15 joint indices -> finger mapping (hand_pose order)
MANO_TO_FINGER = {
    "thumb":  [14, 12, 13],
    "index":  [0, 1, 2],
    "middle": [3, 4, 5],
    "ring":   [6, 7, 8],
    "little": [9, 10, 11],
}

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_addr = ("127.0.0.1", 8080)
last_finger_src = {n: "hold" for n in FINGER_NAMES}
last_finger_q = {n: [(1.0, 0.0, 0.0, 0.0)] * 3 for n in FINGER_NAMES}

def stm32_parse_frame(frame: bytes):
    """解析 STM32 35字节帧 (0xB5 0xA5 0x55)"""
    if len(frame) < 35: return None
    if frame[0] != 0xB5 or frame[1] != 0xA5 or frame[2] != 0x55: return None
    crc = 0
    for b in frame[:34]: crc ^= b
    if crc != frame[34]: return None
    qw = struct.unpack_from('<h', frame, 8)[0] / 10000.0
    qx = struct.unpack_from('<h', frame, 10)[0] / 10000.0
    qy = struct.unpack_from('<h', frame, 12)[0] / 10000.0
    qz = struct.unpack_from('<h', frame, 14)[0] / 10000.0
    norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if norm > 0.001:
        qw /= norm; qx /= norm; qy /= norm; qz /= norm
    return (qw, qx, qy, qz)

def imu_listener_loop(port: str, baud: int):
    """IMU 后台线程：全速读取串口，更新全局变量，零阻塞主线"""
    global imu_data
    try:
        import serial
        ser = serial.Serial(port, baud, timeout=0.01)
        print(f"[IMU] COM {port} opened")
        buf = bytearray()
        while True:
            data = ser.read(256)
            if not data:
                time.sleep(0.001)
                continue
            buf.extend(data)
            while True:
                idx = buf.find(b'\xB5\xA5\x55')
                if idx < 0:
                    buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < 35:
                    break
                frame = bytes(buf[:35])
                del buf[:35]
                q = stm32_parse_frame(frame)
                if q:
                    imu_data.qw, imu_data.qx, imu_data.qy, imu_data.qz = q
                    imu_data.ts = time.time()
                    imu_data.valid = True
    except ImportError:
        print("[IMU] pyserial not installed")
    except Exception as e:
        print(f"[IMU] Error: {e}")

# ========== 加载 HAMER 模型 ==========
print("Loading HAMER ..."); sys.stdout.flush()
m, c = load_hamer(CKPT, init_renderer=False)
d = torch.device("cuda" if torch.cuda.is_available() else "cpu")
m = m.to(d).eval()
print(f"HAMER on {d}"); sys.stdout.flush()

print("Loading ViTPose ..."); sys.stdout.flush()
cpm = ViTPoseModel("cuda" if torch.cuda.is_available() else "cpu")
print("ViTPose ready"); sys.stdout.flush()

# ========== 启动 IMU 后台线程 ==========
import argparse
p = argparse.ArgumentParser()
p.add_argument("--com", default="COM122")
p.add_argument("--baud", type=int, default=460800)
args, _ = p.parse_known_args()
t = threading.Thread(target=imu_listener_loop, args=(args.com, args.baud), daemon=True)
t.start()

# ========== 启动 D435i ==========
print("Opening D435i ..."); sys.stdout.flush()
pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipe.start(config)
for _ in range(30): pipe.wait_for_frames()
print("D435i OK. Press q/ESC to quit."); sys.stdout.flush()

fc = 0; ft = time.time(); fps = 0
vit_skip = 0
last_boxes = None; last_right = None
out_dir = r"C:\Users\Administrator\Documents\Codex\2026-06-18\hamer-d435i-usb\outputs"
os.makedirs(out_dir, exist_ok=True)

# ========== 主循环 ==========
try:
    while True:
        t0 = time.time()
        frames = pipe.wait_for_frames()
        cf = frames.get_color_frame()
        if not cf: continue
        f = np.asanyarray(cf.get_data())
        h, w = f.shape[:2]

        # ---- ViTPose ----
        if vit_skip <= 0:
            det = [np.array([[0, 0, w, h, 0.9]])]
            vitposes = cpm.predict_pose(f, det)
            vit_skip = 15
        else:
            vitposes = []; vit_skip -= 1

        canvas = f.copy()
        bl = []; rl = []; vit_kps_store = {}
        for vp in vitposes:
            kps = vp["keypoints"]
            for hk, ir in [(kps[-42:-21], False), (kps[-21:], True)]:
                v = hk[:,2] > 0.5
                if sum(v) < 8: continue
                bl.append([int(hk[v,0].min())-20, int(hk[v,1].min())-20,
                           int(hk[v,0].max())+20, int(hk[v,1].max())+20])
                rl.append(1.0 if ir else 0.0)
                vit_kps_store[1.0 if ir else 0.0] = hk

        if bl:
            boxes = np.stack(bl).astype(np.float32)
            right = np.stack(rl)
            last_boxes, last_right = boxes, right
        elif last_boxes is not None:
            boxes, right = last_boxes, last_right
        else: boxes = None

        # ---- HAMER 推理 ----
        if boxes is not None:
            ds = ViTDetDataset(c, f, boxes, right, rescale_factor=2.0)
            ld = torch.utils.data.DataLoader(ds, batch_size=4, shuffle=False, num_workers=0)
            for batch in ld:
                batch = recursive_to(batch, d)
                with torch.no_grad(): out = m(batch)
                for n in range(batch["img"].shape[0]):
                    kn = out["pred_keypoints_2d"][n].cpu().numpy() + 0.5
                    ir_val = batch["right"][n].item(); hp = out["pred_mano_params"]["hand_pose"][n].cpu().numpy()
                    if ir_val < 0.5: kn[:,0] = 1.0 - kn[:,0]
                    cx, cy, bs = batch["box_center"][n,0].item(), batch["box_center"][n,1].item(), batch["box_size"][n].item()
                    kp = np.zeros((21,2), dtype=int)
                    kp[:,0] = (cx - bs/2 + kn[:,0]*bs).astype(int)
                    kp[:,1] = (cy - bs/2 + kn[:,1]*bs).astype(int)
                    HAND_EDGES = [(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),(0,9),(9,10),
                                  (10,11),(11,12),(0,13),(13,14),(14,15),(15,16),(0,17),(17,18),(18,19),(19,20)]
                    for e in HAND_EDGES:
                        cv2.line(canvas, tuple(kp[e[0]]), tuple(kp[e[1]]), (0,255,255), 2)
                    for p in kp: cv2.circle(canvas, tuple(p), 4, (0,255,0), -1)
                    col = (255,0,0) if ir_val > 0.5 else (0,255,0)
                    x1,y1 = int(cx-bs/2), int(cy-bs/2)
                    cv2.rectangle(canvas, (x1,y1), (int(cx+bs/2), int(cy+bs/2)), col, 2)
                    cv2.putText(canvas, "R" if ir_val > 0.5 else "L", (int(cx)-10, int(cy)+5),
                                cv2.FONT_HERSHEY_SIMPLEX, .8, col, 2)
                    # Per-finger confidence + decision
                    kp_hand_21 = vit_kps_store.get(ir_val)
                    if kp_hand_21 is not None:
                        for fn in FINGER_NAMES:
                            idxs = PER_FINGER_KP[fn]
                            scores = [kp_hand_21[j][2] for j in idxs]
                            finger_conf = sum(scores) / len(scores) if scores else 0.0
                            if finger_conf >= 0.7:
                                last_finger_src[fn] = "vision"
                                mano_idxs = MANO_TO_FINGER[fn]
                                q_list = []
                                for mi in mano_idxs:
                                    if mi < len(hp):
                                        R = hp[mi]
                                        tr = R[0,0]+R[1,1]+R[2,2]
                                        if tr > 0:
                                            s = 0.5 / np.sqrt(tr+1.0)
                                            qq = (0.25/s, (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s)
                                        else:
                                            qq = (1.0,0.0,0.0,0.0)
                                        nq = np.sqrt(sum(x*x for x in qq))
                                        if nq > 0.001: qq = tuple(x/nq for x in qq)
                                        q_list.append(qq)
                                    else:
                                        q_list.append((1.0,0.0,0.0,0.0))
                                last_finger_q[fn] = q_list
                            else:
                                last_finger_src[fn] = "hold"
                    # Build JSON + UDP
                    payload = {
                        "ts": int(time.time()*1000), "fps": fps, "conf": 0.0,
                        "wrist_pos": [0.0,0.0,0.0], "wrist_rot": [1.0,0.0,0.0,0.0],
                        "wrist_source": "vision",
                        "fingers": [
                            {"name": fn, "joints": last_finger_q[fn], "source": last_finger_src[fn]}
                            for fn in FINGER_NAMES
                        ]
                    }
                    try:
                        udp_sock.sendto(json.dumps(payload).encode("utf-8"), udp_addr)
                    except: pass

        # ========== [IMU 仪表盘 Overlay] ==========
        panel_w = 280
        panel = np.zeros((h, panel_w, 3), dtype=np.uint8) + 40
        im = imu_data
        F = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(panel, "IMU STATUS", (15, 30), F, 0.55, (0,255,255), 2)
        cv2.putText(panel, f"Connected: {'YES' if im.valid else 'NO'}", (15, 60), F, 0.45, (0,255,0) if im.valid else (0,0,255), 1)
        qw, qx, qy, qz = im.qw, im.qx, im.qy, im.qz
        cv2.putText(panel, f"Qw: {qw:.3f}", (15, 100), F, 0.45, (200,200,200), 1)
        cv2.putText(panel, f"Qx: {qx:.3f}", (15, 125), F, 0.45, (200,200,200), 1)
        cv2.putText(panel, f"Qy: {qy:.3f}", (15, 150), F, 0.45, (200,200,200), 1)
        cv2.putText(panel, f"Qz: {qz:.3f}", (15, 175), F, 0.45, (200,200,200), 1)

        cv2.putText(panel, "FINGER CONFIDENCE", (15, 210), F, 0.5, (0,255,255), 2)
        y_base = 235
        for i, fn in enumerate(FINGER_NAMES):
            src = last_finger_src.get(fn, "hold")
            src_color = (0,255,0) if src == "vision" else (100,100,100)
            src_label = "V" if src == "vision" else "H"
            q_list = last_finger_q.get(fn, [(1,0,0,0)]*3)
            q0 = q_list[0] if q_list else (1,0,0,0)
            yp = y_base + i * 22
            colors = {"thumb":(255,180,0),"index":(0,255,0),"middle":(0,200,255),"ring":(255,200,0),"little":(0,100,255)}
            c_f = colors.get(fn, (200,200,200))
            cv2.putText(panel, f"  {fn[:3].upper()}", (15, yp), F, 0.4, c_f, 1)
            cv2.putText(panel, f"[{src_label}] qw:{q0[0]:.2f} qx:{q0[1]:.2f}", (85, yp), F, 0.35, src_color, 1)

        udp_y = y_base + len(FINGER_NAMES) * 22 + 10
        cv2.putText(panel, "UDP: 127.0.0.1:8080", (15, udp_y), F, 0.4, (100,255,100), 1)
        cv2.putText(panel, f"FPS: {fps}", (15, udp_y + 22), F, 0.45, (255,255,0), 1)
        frame_view = np.hstack((canvas, panel))

        # ---- FPS 统计 + 显示 ----
        fc += 1
        if time.time()-ft >= 1: fps,fc,ft = fc,0,time.time()
        cv2.putText(frame_view, f"D435i FPS:{fps}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,200,0), 2)
        cv2.imshow("HAMER + IMU (Non-blocking)", frame_view)
        k = cv2.waitKey(1)&0xFF
        if k in (ord("q"),27): break
        elif k == ord("s"):
            sp = os.path.join(out_dir, f'd435i_{time.strftime("%H%M%S")}.jpg')
            cv2.imwrite(sp, frame_view)
            print(f"Saved {sp}"); sys.stdout.flush()
finally:
    pipe.stop(); cv2.destroyAllWindows()
    print("Done.")
