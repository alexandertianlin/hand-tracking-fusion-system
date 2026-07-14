#!/usr/bin/env python3
# ==============================================================================
# fusion_pipeline.py ? IMU + Visual (D435i+HAMER) Fusion Pipeline
#
# Reads:  D435i camera -> ViTPose detection -> HAMER inference -> global_orient
# Reads:  STM32 IMU gloves (COM122, 460800 baud) -> quaternion
# Fuses:  Complementary filter (Slerp with dynamic alpha)
# Sends:  UDP JSON to:
#   127.0.0.1:8080 (FusionUDPReceiver.cs - fused IMU+visual)
#   127.0.0.1:5055 (VisionBridge.cs - visual-only curl/spread)
#
# 2026-06-29 ? P0 Step 1-3 implementation
# ==============================================================================

import os, sys, time, json, socket, struct, threading, queue, logging, gc, math
os.environ["PYOPENGL_PLATFORM"] = "wgl"
os.environ["PYTORCH_JIT"] = "0"

# CPU mode selector: balanced|performance|max
_cpu_mode = "performance"
for _arg in sys.argv[1:]:
    if _arg.startswith("--cpu="):
        _cpu_mode = _arg.split("=")[1]
        break
if _cpu_mode == "balanced":
    _omp, _mkl, _torch = "8", "4", 4
elif _cpu_mode == "performance":
    _omp, _mkl, _torch = "16", "8", 8
elif _cpu_mode == "max":
    _omp, _mkl, _torch = "0", "0", 0
else:
    print(f"Unknown CPU mode {_cpu_mode}, using performance")
    _omp, _mkl, _torch = "16", "8", 8
if _omp != "0":
    os.environ["OMP_NUM_THREADS"] = _omp
    os.environ["MKL_NUM_THREADS"] = _mkl
    os.environ["OPENBLAS_NUM_THREADS"] = _mkl
import torch
torch.cuda.set_device(0)
if _torch > 0:
    torch.set_num_threads(_torch)
    torch.set_num_interop_threads(_torch)
torch.set_grad_enabled(False)
import numpy as np
import cv2
import time as time_module  # for profiling

HAMER_DIR = r"C:\Users\Administrator\Documents\Codex\2026-06-16\files-mentioned-by-the-user-gpu2-3\hamer_code\hamer-main"
CKPT = os.path.join(HAMER_DIR, "_DATA", "hamer_ckpts", "checkpoints", "hamer.ckpt")
os.chdir(HAMER_DIR)
sys.path.insert(0, HAMER_DIR)
sys.path.insert(0, os.path.join(HAMER_DIR, "third-party", "ViTPose"))
sys.path.insert(0, os.path.join(HAMER_DIR, "third-party"))

# ==============================================================================
# Constants
# ==============================================================================
COM_PORT, COM_BAUD = "COM122", 460800
FRAME_HEADER = bytes([0xB5, 0xA5, 0x55])
PACKET_LEN = 35
QUAT_SCALE = 10000.0
UDP_IP = "127.0.0.1"
UDP_PORT_VIS = 5055
UDP_PORT_FUS = 8080
SEND_INTERVAL = 0.033
IMU_ALPHA_HIGH = 0.85
IMU_ALPHA_LOW = 1.0
CONF_THRESH_HIGH = 0.7
CONF_THRESH_LOW = 0.5
ALPHA_RECOVERY_STEP = 0.05
CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 640, 480, 30
FINGER_NAMES = ["thumb", "index", "middle", "ring", "little"]
MANO_TO_FINGER = {"thumb": [14, 12, 13], "index": [0, 1, 2], "middle": [3, 4, 5], "ring": [6, 7, 8], "little": [9, 10, 11]}

# Finger config for VisionBridge curl/spread (matching run_vitpose_v3.py)
FC = [{"mcp":2,"pip":3,"dip":3,"tip":4},{"mcp":5,"pip":6,"dip":7,"tip":8},{"mcp":9,"pip":10,"dip":11,"tip":12},{"mcp":13,"pip":14,"dip":15,"tip":16},{"mcp":17,"pip":18,"dip":19,"tip":20}]
W, I0, M0, M3, L0 = 0, 5, 9, 12, 17

# ==============================================================================
# Data classes
# ==============================================================================
from dataclasses import dataclass, field
from typing import Optional

@dataclass
class ImuData:
    device_id: int = 0
    quat_wxyz: tuple = (1.0, 0.0, 0.0, 0.0)
    timestamp_ms: int = 0
    valid: bool = False

@dataclass
class VisualData:
    wrist_rotmat: np.ndarray = None
    wrist_pos: tuple = (0.0, 0.0, 0.0)
    hand_pose_rotmats: list = field(default_factory=list)
    keypoints_3d: np.ndarray = None
    confidence: float = 0.0
    timestamp_ms: int = 0
    valid: bool = False

@dataclass
class FusionOutput:
    wrist_quat: tuple = (1.0, 0.0, 0.0, 0.0)
    wrist_pos: tuple = (0.0, 0.0, 0.0)
    finger_quats: dict = field(default_factory=dict)
    confidence: float = 0.0
    fps: float = 0.0


# ==============================================================================
# Quaternion utilities
# ==============================================================================
def quat_mul(a, b):
    aw,ax,ay,az = a; bw,bx,by,bz = b
    return (aw*bw - ax*bx - ay*by - az*bz, aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx, aw*bz + ax*by - ay*bx + az*bw)

def quat_conj(q): w,x,y,z = q; return (w, -x, -y, -z)

def quat_norm(q):
    w,x,y,z = q; n = np.sqrt(w*w+x*x+y*y+z*z)
    return (1.0,0.0,0.0,0.0) if n < 1e-10 else (w/n, x/n, y/n, z/n)

def quat_inv(q): return quat_norm(quat_conj(q))

def quat_slerp(a, b, t):
    a, b = quat_norm(a), quat_norm(b)
    d = a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3]
    if d < 0: b = (-b[0], -b[1], -b[2], -b[3]); d = -d
    d = np.clip(d, -1.0, 1.0)
    if d > 0.9995:
        r = (a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]), a[2]+t*(b[2]-a[2]), a[3]+t*(b[3]-a[3]))
        return quat_norm(r)
    th0 = np.arccos(d); th = th0*t; s0 = np.cos(th)-d*np.sin(th)/np.sin(th0)
    s1 = np.sin(th)/np.sin(th0)
    return quat_norm((s0*a[0]+s1*b[0], s0*a[1]+s1*b[1], s0*a[2]+s1*b[2], s0*a[3]+s1*b[3]))

def rotmat_to_quat(R):
    tr = R[0,0]+R[1,1]+R[2,2]
    if tr > 0:
        s = 0.5/np.sqrt(tr+1.0); return (0.25/s, (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s)
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0*np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])
        return ((R[2,1]-R[1,2])/s, 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s)
    elif R[1,1] > R[2,2]:
        s = 2.0*np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])
        return ((R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s)
    else:
        s = 2.0*np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])
        return ((R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s)

def mat3_to_quat_list(R): return [round(float(q),6) for q in rotmat_to_quat(R)]

def quat_to_list(q): return [round(float(q[0]),6), round(float(q[1]),6), round(float(q[2]),6), round(float(q[3]),6)]

# ==============================================================================
# VisionBridge curl/spread from 3D keypoints
# ==============================================================================
def nrm(v): n = np.linalg.norm(v); return v/n if n > 1e-8 else v

def curl_from_kp(pw, n):
    c = FC[n]; m = pw[c["mcp"]]; p = pw[c["pip"]]; d = pw[c["dip"]]; t = pw[c["tip"]]
    chord = np.linalg.norm(t-m); pip2tip = np.linalg.norm(t-p)
    arc = np.linalg.norm(m-p)+np.linalg.norm(p-t) if n == 0 else np.linalg.norm(m-p)+np.linalg.norm(p-d)+np.linalg.norm(d-t)
    return 0.0 if arc < 1e-8 else 1.0 - max(0, min(1, 0.8*chord/arc + 0.2*pip2tip/arc))

def spread_from_kp(pw, n):
    if n == 2: return 0.0
    c = FC[n]; mc = FC[2]
    fd = nrm(pw[c["tip"]]-pw[c["mcp"]]); md = nrm(pw[mc["tip"]]-pw[mc["mcp"]])
    ddot = max(-1, min(1, float(np.dot(fd, md))))
    return max(0, min(1, math.acos(ddot)/1.2))

# ==============================================================================
# STM32 IMU Parser (35-byte frame protocol)
# ==============================================================================
class Stm32Parser:
    def __init__(self):
        self._buffer = bytearray()
        self._frame_count = 0
        self._error_count = 0

    def feed(self, data: bytes) -> list:
        self._buffer.extend(data); frames = []
        while True:
            idx = self._buffer.find(FRAME_HEADER)
            if idx < 0: self._buffer.clear(); break
            if idx > 0: del self._buffer[:idx]
            if len(self._buffer) < PACKET_LEN: break
            frame = bytes(self._buffer[:PACKET_LEN]); del self._buffer[:PACKET_LEN]
            try:
                imu = self._parse_frame(frame)
                if imu: frames.append(imu); self._frame_count += 1
                else: self._error_count += 1
            except: self._error_count += 1
        return frames

    def _parse_frame(self, frame: bytes) -> Optional[ImuData]:
        if frame[0]!=0xB5 or frame[1]!=0xA5 or frame[2]!=0x55: return None
        crc = 0; [crc:=crc^b for b in frame[:PACKET_LEN-1]]
        if crc != frame[PACKET_LEN-1]: return None
        device_id = frame[6]
        qw,qx,qy,qz = [struct.unpack_from("<h",frame,8+i)[0]/QUAT_SCALE for i in (0,2,4,6)]
        n = np.sqrt(qw*qw+qx*qx+qy*qy+qz*qz)
        if n > 0.001: qw/=n; qx/=n; qy/=n; qz/=n
        else: qw,qx,qy,qz = 1.0,0.0,0.0,0.0
        return ImuData(device_id=device_id, quat_wxyz=(qw,qx,qy,qz), timestamp_ms=int(time.time()*1000), valid=True)

# ==============================================================================
# Fusion Engine (Complementary Filter)
# ==============================================================================
class FusionEngine:
    def __init__(self):
        self.calibrated = False
        self.q_imu_offset = (1.0,0.0,0.0,0.0)
        self.wrist_pos_zero = (0.0,0.0,0.0)
        self.calib_imu, self.calib_visual = [], []
        self.calib_samples_needed = 15
        self.last_finger_quats = {name: [(1.0,0.0,0.0,0.0)]*3 for name in FINGER_NAMES}
        self.last_confidence = 1.0
        self.current_alpha = IMU_ALPHA_HIGH
        self.wrist_pos_last = (0.0,0.0,0.0)
        self._fps_counter, self._fps_timer = 0, time.time()
        self._current_fps = 0.0

    def calibrate(self, imu_q, wrist_rotmat, wrist_pos):
        self.calib_imu.append(imu_q)
        hamer_q = rotmat_to_quat(wrist_rotmat)
        self.calib_visual.append({"q": hamer_q, "pos": wrist_pos})
        n = len(self.calib_imu)
        if n < self.calib_samples_needed: return False
        self.calib_imu = self.calib_imu[-self.calib_samples_needed:]
        self.calib_visual = self.calib_visual[-self.calib_samples_needed:]
        q_avg = tuple(np.mean([list(q) for q in self.calib_imu], axis=0))
        self.q_imu_offset = quat_inv(quat_norm(q_avg))
        pos_avg = tuple(np.mean([s["pos"] for s in self.calib_visual], axis=0))
        self.wrist_pos_zero = pos_avg
        self.calibrated = True
        print(f"[CALIB] Done. Samples={n}, IMU offset=({self.q_imu_offset[0]:.3f},...)")
        return True

    def _compute_alpha(self, conf):
        if conf >= CONF_THRESH_HIGH: target = IMU_ALPHA_HIGH
        elif conf >= CONF_THRESH_LOW:
            ratio = 1.0-(conf-CONF_THRESH_LOW)/(CONF_THRESH_HIGH-CONF_THRESH_LOW)
            target = IMU_ALPHA_HIGH+(1.0-IMU_ALPHA_HIGH)*ratio
        else: target = IMU_ALPHA_LOW
        step = ALPHA_RECOVERY_STEP*2
        if self.current_alpha < target: self.current_alpha = min(target, self.current_alpha+step)
        elif self.current_alpha > target: self.current_alpha = max(target, self.current_alpha-step)
        return self.current_alpha

    def fuse(self, imu, visual):
        conf = visual.confidence if visual.valid else 0.0
        self._compute_alpha(conf)
        q_vis = rotmat_to_quat(visual.wrist_rotmat) if visual.valid else (1.0,0.0,0.0,0.0)
        q_imu = quat_mul(self.q_imu_offset, imu.quat_wxyz) if imu.valid else (1.0,0.0,0.0,0.0)
        q_wrist = quat_slerp(q_vis, q_imu, self.current_alpha)
        if visual.valid and conf >= CONF_THRESH_LOW:
            wp = visual.wrist_pos; self.wrist_pos_last = (wp[0]-self.wrist_pos_zero[0], -(wp[1]-self.wrist_pos_zero[1]), wp[2]-self.wrist_pos_zero[2])
        finger_q = {}
        for name in FINGER_NAMES:
            joints = []
            if visual.valid and visual.hand_pose_rotmats:
                for idx in MANO_TO_FINGER[name]:
                    joints.append(rotmat_to_quat(visual.hand_pose_rotmats[idx]) if idx < len(visual.hand_pose_rotmats) else (1.0,0.0,0.0,0.0))
                self.last_finger_quats[name] = joints
            else: joints = self.last_finger_quats[name]
            finger_q[name] = joints
        self._fps_counter += 1
        now = time.time()
        if now-self._fps_timer >= 1.0:
            self._current_fps = self._fps_counter/(now-self._fps_timer)
            self._fps_counter, self._fps_timer = 0, now
        return FusionOutput(wrist_quat=q_wrist, wrist_pos=self.wrist_pos_last, finger_quats=finger_q, confidence=conf, fps=self._current_fps)

# ==============================================================================
# IMU Background Thread
# ==============================================================================
def imu_reader_thread(imu_queue, stop_event):
    parser = Stm32Parser()
    import serial
    print(f"[IMU] Opening {COM_PORT} @ {COM_BAUD}...")
    try:
        ser = serial.Serial(COM_PORT, COM_BAUD, timeout=0.01)
        ser.reset_input_buffer()
        print("[IMU] OK")
        while not stop_event.is_set():
            data = ser.read(256)
            if data:
                for f in parser.feed(data): imu_queue.put(f)
            else: time.sleep(0.005)  # 200Hz, IMU is 100Hz
    except serial.SerialException as e:
        print(f"[IMU] ERROR: {e}")
    finally:
        if "ser" in locals() and ser.is_open: ser.close()
        print("[IMU] Thread stopped")

# ==============================================================================
# UDP Senders
# ==============================================================================
def send_udp_fusion(sock, fused, visual, seq):
    packet = {"ts":int(time.time()*1000),"seq":seq,"fps":round(fused.fps,1),"conf":round(fused.confidence,3),
              "wrist_pos":[round(float(fused.wrist_pos[0]),4),round(float(fused.wrist_pos[1]),4),round(float(fused.wrist_pos[2]),4)],
              "wrist_rot":quat_to_list(fused.wrist_quat),"fingers":[]}
    for name in FINGER_NAMES:
        joints = fused.finger_quats.get(name, [(1.0,0.0,0.0,0.0)]*3)
        packet["fingers"].append({"name":name,"joints":[quat_to_list(q) for q in joints]})
    try: sock.sendto(json.dumps(packet, separators=(",",":")).encode(), (UDP_IP, UDP_PORT_FUS))
    except Exception as e: print(f"[UDP] Send error: {e}")

def send_udp_vision(sock, kp3d, seq):
    pkt = {"type":"hamer_hand","seq":seq,"ts":int(time.time()*1000),"num_hands":1,
           "hand_0_label":"right","hand_0_conf":0.9,"hand_0_wrist":kp3d[0].tolist()}
    for i in range(5):
        fn = ["thumb","index","middle","ring","little"][i]
        pkt["hand_0_curl_"+fn] = round(float(curl_from_kp(kp3d, i)), 3)
        pkt["hand_0_spread_"+fn] = round(float(spread_from_kp(kp3d, i)), 3)
    try: sock.sendto(json.dumps(pkt, separators=(",",":")).encode(), ("127.0.0.1", 5055))
    except: pass


# ==============================================================================
# Main Pipeline
# ==============================================================================
def main():
    print("=== IMU + Visual Fusion Pipeline ===")
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"VRAM: {torch.cuda.get_device_properties(0).total_memory/1e9:.1f} GB")
    print(f"IMU: {COM_PORT} @ {COM_BAUD}")
    print(f"Camera: D435i @ {CAM_WIDTH}x{CAM_HEIGHT} {CAM_FPS}fps")
    print(f"UDP fusion: {UDP_IP}:{UDP_PORT_FUS}, vision: {UDP_IP}:{UDP_PORT_VIS}")
    print()

    # Load HAMER
    print("Loading HAMER...")
    from hamer.models import load_hamer
    m_hamer, cfg_h = load_hamer(CKPT, init_renderer=False)
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    m_hamer = m_hamer.to(dev).eval()
    print(f"HAMER loaded ({sum(p.numel() for p in m_hamer.parameters())/1e6:.0f}M params)")

    # Load ViTPose
    print("Loading ViTPose...")
    from vitpose_model import ViTPoseModel
    vitpose = ViTPoseModel(dev)
    print("ViTPose ready")

    # Open D435i
    print("Opening D435i...")
    import pyrealsense2 as rs
    pipe, cfg_rs = rs.pipeline(), rs.config()
    cfg_rs.enable_stream(rs.stream.color, CAM_WIDTH, CAM_HEIGHT, rs.format.bgr8, CAM_FPS)
    try:
        pipe.start(cfg_rs)
        for _ in range(15): pipe.wait_for_frames()
        print("D435i OK")
    except Exception as e:
        print(f"D435i ERROR: {e}"); return

    # Start IMU reader thread
    imu_queue, stop_event = queue.Queue(), threading.Event()
    imu_thread = threading.Thread(target=imu_reader_thread, args=(imu_queue, stop_event), daemon=True)
    imu_thread.start()

    # UDP sockets
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_vision = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Fusion engine
    fusion = FusionEngine()

    # State variables
    seq = 0; last_boxes = None; last_right = None; last_kp2d = None
    vit_skip = 0; consecutive_no_hand = 0; hands_confirmed = True
    frame_count = 0; fps_timer = time.time(); current_fps = 0.0
    visual = VisualData(valid=False); calib_frame_count = 0
    show_debug = True; saved_kps = None

    print()
    print("=== Running. Press q/ESC to quit, c to recalibrate ===")
    print()

    try:
        while True:
            _t_start = time_module.time()
            frames = pipe.wait_for_frames()
            cf = frames.get_color_frame()
            if not cf: continue
            img = np.asanyarray(cf.get_data())
            h, w = img.shape[:2]
            frame_count += 1

            if frame_count % 30 == 0:
                now = time.time(); current_fps = 30/(now-fps_timer); fps_timer = now

            # ViTPose every 3rd frame
            ran_vitpose = False
            if vit_skip <= 0:
                ran_vitpose = True
                det = [np.array([[0,0,w,h,0.9]])]
                _t0 = time.time()
                vitposes = vitpose.predict_pose(img, det)
                _t_vit = (time.time() - _t0) * 1000
                vit_skip = 3
            else:
                vitposes = []; vit_skip -= 1

            boxes_list, rights_list = [], []
            saved_kps = None
            for vp in vitposes:
                kps = vp["keypoints"]
                for hk, ir in [(kps[-42:-21], False), (kps[-21:], True)]:
                    v = hk[:,2] > 0.5
                    if sum(v) < 8: continue
                    boxes_list.append([int(hk[v,0].min())-20, int(hk[v,1].min())-20, int(hk[v,0].max())+20, int(hk[v,1].max())+20])
                    rights_list.append(1.0 if ir else 0.0)
                    if saved_kps is None: saved_kps = kps

            # IoU dedup
            if len(boxes_list) > 1:
                dedup_boxes, dedup_rights = [], []
                for bi, (box, ri) in enumerate(zip(boxes_list, rights_list)):
                    is_dup = False
                    for bj, rj in zip(boxes_list, rights_list):
                        if ri == rj: continue
                        x1,x2 = max(box[0],bj[0]), min(box[2],bj[2])
                        y1,y2 = max(box[1],bj[1]), min(box[3],bj[3])
                        if x1<x2 and y1<y2 and (x2-x1)*(y2-y1)/min((box[2]-box[0])*(box[3]-box[1]), (bj[2]-bj[0])*(bj[3]-bj[1])) > 0.3:
                            is_dup = True; break
                    if not is_dup: dedup_boxes.append(box); dedup_rights.append(ri)
                boxes_list, rights_list = dedup_boxes, dedup_rights

            if boxes_list:
                consecutive_no_hand = 0
                last_boxes = np.stack(boxes_list).astype(np.float32)
                last_right = np.stack(rights_list)
            elif ran_vitpose:
                consecutive_no_hand += 1
                if consecutive_no_hand > 5: last_boxes = None; last_right = None
            if ran_vitpose: hands_confirmed = bool(boxes_list)

            if not ran_vitpose and last_kp2d is not None and hands_confirmed and last_boxes is not None:
                centroid = last_kp2d.mean(axis=0)
                bw = last_boxes[0][2]-last_boxes[0][0]; bh = last_boxes[0][3]-last_boxes[0][1]
                last_boxes[0] = [centroid[0]-bw/2, centroid[1]-bh/2, centroid[0]+bw/2, centroid[1]+bh/2]

            # HAMER inference
            visual.valid = False
            if last_boxes is not None and hands_confirmed:
                from hamer.datasets.vitdet_dataset import ViTDetDataset
                from hamer.utils import recursive_to
                ds = ViTDetDataset(cfg_h, img, last_boxes, last_right, rescale_factor=2.0)
                loader = torch.utils.data.DataLoader(ds, batch_size=4, shuffle=False, num_workers=0)
                for batch in loader:
                    batch = recursive_to(batch, dev)
                    _t0 = time.time()
                    with torch.no_grad(): out = m_hamer(batch)
                    _t_hamer = (time.time() - _t0) * 1000
                    for n in range(batch["img"].shape[0]):
                        # KEY: Extract global_orient rotation matrix -> quaternion
                        go = out["pred_mano_params"]["global_orient"][n]
                        wrist_rotmat = go.cpu().numpy().squeeze().reshape(3,3)
                        hp = out["pred_mano_params"]["hand_pose"][n]
                        hand_pose_mats = hp.cpu().numpy()
                        kp3d = out["pred_keypoints_3d"][n].cpu().numpy()
                        kp2d = out["pred_keypoints_2d"][n].cpu().numpy()+0.5

                        ir_val = batch["right"][n].item()
                        if ir_val < 0.5: kp2d[:,0] = 1.0-kp2d[:,0]
                        cx,cy,bs = batch["box_center"][n,0].item(), batch["box_center"][n,1].item(), batch["box_size"][n].item()
                        kp2d_img = np.zeros((21,2), dtype=int)
                        kp2d_img[:,0] = (cx-bs/2+kp2d[:,0]*bs).astype(int)
                        kp2d_img[:,1] = (cy-bs/2+kp2d[:,1]*bs).astype(int)
                        last_kp2d = np.column_stack([kp2d_img[:,0].astype(np.float32), kp2d_img[:,1].astype(np.float32)])

                        conf = 0.5
                        if saved_kps is not None:
                            scores = []; [scores.extend(s[:,2].tolist()) for s in [saved_kps[-42:-21], saved_kps[-21:]]]
                            conf = float(np.mean([s for s in scores if s > 0])) if scores else 0.5

                        visual = VisualData(wrist_rotmat=wrist_rotmat, wrist_pos=tuple(kp3d[0].tolist()),
                            hand_pose_rotmats=[hand_pose_mats[j] for j in range(15)], keypoints_3d=kp3d,
                            confidence=conf, timestamp_ms=int(time.time()*1000), valid=True)

                        # Draw debug skeleton
                        for e in [(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),(0,9),(9,10),(10,11),(11,12),
                                  (0,13),(13,14),(14,15),(15,16),(0,17),(17,18),(18,19),(19,20)]:
                            cv2.line(img, tuple(kp2d_img[e[0]]), tuple(kp2d_img[e[1]]), (0,255,255), 2)
                        for p in kp2d_img: cv2.circle(img, tuple(p), 4, (0,255,0), -1)
                        cv2.putText(img, "R" if ir_val>0.5 else "L", (int(cx)-10,int(cy)+5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2)

            # Get latest IMU data
            latest_imu = ImuData(valid=False)
            while not imu_queue.empty():
                try: latest_imu = imu_queue.get_nowait()
                except queue.Empty: break

            # Calibration phase
            if not fusion.calibrated and visual.valid:
                if fusion.calibrate(latest_imu.quat_wxyz, visual.wrist_rotmat, visual.wrist_pos):
                    print("[CALIB] Sensor fusion calibrated")
                calib_frame_count = len(fusion.calib_imu)

            # Fusion
            if fusion.calibrated:
                fused = fusion.fuse(latest_imu, visual)
                seq += 1
                send_udp_fusion(sock, fused, visual, seq)
                if visual.valid and visual.keypoints_3d is not None:
                    send_udp_vision(sock_vision, visual.keypoints_3d, seq)
            elif visual.valid:
                q_vis = rotmat_to_quat(visual.wrist_rotmat)
                sock.sendto(json.dumps({"ts":int(time.time()*1000),"seq":seq,"fps":0,"conf":0,
                    "wrist_pos":[0,0,0],"wrist_rot":quat_to_list(q_vis),"fingers":[]},
                    separators=(",",":")).encode(), (UDP_IP, UDP_PORT_FUS))

            # Profiling every 30 frames
            if frame_count % 30 == 0:
                _t_total = (time_module.time() - _t_start) * 1000
                _vit_ms = _t_vit if ran_vitpose else 0
                _hamer_ms = _t_hamer if visual.valid else 0
                print(f"[PERF] CPU={_cpu_mode} total={_t_total:.0f}ms vit={_vit_ms:.0f}ms hamer={_hamer_ms:.0f}ms fps={current_fps:.1f}", flush=True)

            # Debug overlay
            overlay_lines = []
            if fusion.calibrated:
                overlay_lines.append(f"FUSION | alpha={fusion.current_alpha:.2f} conf={fused.confidence:.2f} fps={fused.fps:.0f}")
                q = fused.wrist_quat
                overlay_lines.append(f"Wrist Q: ({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f})")
                overlay_lines.append(f"Seq: {seq} | IMU: {latest_imu.valid}")
            else:
                overlay_lines.append(f"CALIBRATING... ({calib_frame_count}/{fusion.calib_samples_needed})")
                if visual.valid:
                    q = rotmat_to_quat(visual.wrist_rotmat)
                    overlay_lines.append(f"Visual Q: ({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f})")
            for i, line in enumerate(overlay_lines):
                cv2.putText(img, line, (8, 20+i*20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,200,0), 1)

            if show_debug:
                cv2.imshow("IMU+Visual Fusion Pipeline", img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27: break
                elif key == ord("c"):
                    fusion.calibrated = False; fusion.calib_imu.clear(); fusion.calib_visual.clear()
                    print("[CALIB] Reset. Collecting samples...")

    except KeyboardInterrupt:
        print(); print("Interrupted")
    finally:
        print("Cleaning up..."); stop_event.set(); pipe.stop()
        sock.close(); sock_vision.close(); cv2.destroyAllWindows()
        print("Done")

if __name__ == "__main__":
    main()

