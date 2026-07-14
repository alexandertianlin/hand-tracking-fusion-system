"""
Measure latency: camera capture, ViTPose, HAMER, full pipeline.
Runs for ~20 frames with current pipeline, prints summary table.
"""
import os, time, cv2, numpy as np, torch
os.environ["PYOPENGL_PLATFORM"] = "wgl"
np.bool = bool

HAMER_DIR = r"C:\Users\Administrator\Documents\Codex\2026-06-16\files-mentioned-by-the-user-gpu2-3\hamer_code\hamer-main"
CKPT = os.path.join(HAMER_DIR, "_DATA", "hamer_ckpts","checkpoints","hamer.ckpt")

os.chdir(HAMER_DIR)
import sys; sys.path.insert(0, HAMER_DIR)
sys.path.insert(0, os.path.join(HAMER_DIR, "third-party", "ViTPose"))

from hamer.models import load_hamer
from hamer.datasets.vitdet_dataset import ViTDetDataset
from hamer.utils import recursive_to
from vitpose_model import ViTPoseModel

print("Loading models ..."); sys.stdout.flush()
m, c = load_hamer(CKPT, init_renderer=False)
d = torch.device("cuda" if torch.cuda.is_available() else "cpu")
m = m.to(d).eval()
cpm = ViTPoseModel("cuda")
imsz = c.MODEL.IMAGE_SIZE

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap.isOpened(): cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
if not cap.isOpened(): print("No camera!"); exit()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

N_FRAMES = 30
times = {"cap":[], "vit":[], "hamer":[], "total":[], "camera_interval":[]}
vit_skip = 0
last_boxes = last_right = None

print(f"Measuring {N_FRAMES} frames ...")
for frame_i in range(N_FRAMES):
    t0 = time.time()
    r, f = cap.read()
    if not r: break
    h, w = f.shape[:2]
    t_cap = time.time()

    if vit_skip <= 0:
        det = [np.array([[0, 0, w, h, 0.9]])]
        vitposes = cpm.predict_pose(f, det)
        vit_skip = 5
    else:
        vitposes = []
        vit_skip -= 1
    t_vit = time.time()

    boxes_list = []; right_list = []
    for vp in vitposes:
        kps_all = vp["keypoints"]
        for hk, ir in [(kps_all[-42:-21], False), (kps_all[-21:], True)]:
            v = hk[:,2] > 0.5
            if sum(v) < 4: continue
            x1b = int(hk[v,0].min()); y1b = int(hk[v,1].min())
            x2b = int(hk[v,0].max()); y2b = int(hk[v,1].max())
            boxes_list.append([x1b-20, y1b-20, x2b+20, y2b+20])
            right_list.append(1.0 if ir else 0.0)

    t_hamer_start = time.time()
    boxes = right = None
    if boxes_list:
        boxes = np.stack(boxes_list).astype(np.float32)
        right = np.stack(right_list)
        last_boxes, last_right = boxes, right
    elif last_boxes is not None:
        boxes, right = last_boxes, last_right

    if boxes is not None:
        dataset = ViTDetDataset(c, f, boxes, right, rescale_factor=2.0)
        loader = torch.utils.data.DataLoader(dataset, batch_size=4, shuffle=False, num_workers=0)
        for batch in loader:
            batch = recursive_to(batch, d)
            with torch.no_grad(): out = m(batch)
    t_hamer_end = time.time()

    # Collect timing
    if frame_i > 2:  # skip first few frames for warm-up
        times["cap"].append(t_cap - t0)
        times["vit"].append(t_vit - t_cap)
        times["hamer"].append(t_hamer_end - t_hamer_start)
        times["total"].append(t_hamer_end - t0)
        if frame_i > 0:
            times["camera_interval"].append(t0 - prev_t0)
    prev_t0 = t0

cap.release()

# Camera-only latency test
print("\nMeasuring camera capture latency ...")
cap2 = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap2.isOpened(): cap2 = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
N_CAM = 60
cam_times = []
for i in range(N_CAM+5):
    t0 = time.time()
    r, f = cap2.read()
    if r and i >= 5:
        cam_times.append(time.time() - t0)
cap2.release()

# Print results
print("\n=========================== LATENCY TABLE ===========================")
print(f"{'Stage':<30} {'Min':>10} {'Avg':>10} {'Max':>10} {'FPS':>10}")
print("-"*70)

def stat(arr, label):
    if len(arr) == 0: return
    a = np.array(arr) * 1000  # to ms
    fps = 1000 / a.mean() if a.mean() > 0 else 0
    print(f"{label:<30} {a.min():>8.1f}ms {a.mean():>8.1f}ms {a.max():>8.1f}ms {fps:>8.1f}")

stat(times["camera_interval"], "Camera frame interval")
stat(cam_times, "Camera cap.read() delay")
stat(times["cap"], "Camera capture (in pipeline)")
stat(times["vit"], "ViTPose inference")
stat(times["hamer"], "HAMER inference")
stat(times["total"], "Full pipeline (one frame)")

# Effective FPS
total_time = sum(times["total"]) / 1000  # sum in seconds... wait
if len(times["total"]) > 0:
    avg_frame = np.mean(times["total"])
    eff_fps = 1.0 / avg_frame if avg_frame > 0 else 0
    print(f"\nEffective FPS (with frame skip): {eff_fps:.1f}")
    # Total frames per ViTPose run
    vit_freq = 6  # every 6 frames
    vit_avg = np.mean(times["vit"]) if times["vit"] else 0
    hamer_avg = np.mean(times["hamer"]) if times["hamer"] else 0
    cam_avg = np.mean(times["cap"]) if times["cap"] else 0
    effective_per_frame = cam_avg + vit_avg/vit_freq + hamer_avg
    print(f"Without frame skip (all ViTPose): {1/(cam_avg+vit_avg+hamer_avg):.1f} FPS" if (cam_avg+vit_avg+hamer_avg)>0 else "")
    print(f"With frame skip ({vit_freq}f):      {1/effective_per_frame:.1f} FPS" if effective_per_frame>0 else "")
print("====================================================================")
