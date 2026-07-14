"""HAMER + ViTPose. ViTDetDataset crop + linear bbox mapping."""
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

print("Loading HAMER ..."); sys.stdout.flush()
m, c = load_hamer(CKPT, init_renderer=False)
d = torch.device("cuda" if torch.cuda.is_available() else "cpu")
m = m.to(d).eval()
print(f"HAMER on {d}"); sys.stdout.flush()

print("Loading ViTPose ..."); sys.stdout.flush()
cpm = ViTPoseModel("cuda")
print("ViTPose ready"); sys.stdout.flush()

imsz = c.MODEL.IMAGE_SIZE
HAND_EDGES = [(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),(0,9),(9,10),
              (10,11),(11,12),(0,13),(13,14),(14,15),(15,16),(0,17),(17,18),(18,19),(19,20)]

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap.isOpened(): cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
if not cap.isOpened(): print("No camera!"); exit()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
print("Camera OK. Press q/ESC to quit."); sys.stdout.flush()

fc=0; ft=time.time(); fps=0; vit_skip=0; last_boxes=None; last_right=None
while cap.isOpened():
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
    t_hamer = 0.0
    canvas = f.copy()
    boxes_list = []; right_list = []; t_hamer = 0.0

    for vp in vitposes:
        kps_all = vp["keypoints"]
        for hk, ir in [(kps_all[-42:-21], False), (kps_all[-21:], True)]:
            v = hk[:,2] > 0.5
            if sum(v) < 8: continue
            x1b = int(hk[v,0].min()); y1b = int(hk[v,1].min())
            x2b = int(hk[v,0].max()); y2b = int(hk[v,1].max())
            boxes_list.append([x1b-20, y1b-20, x2b+20, y2b+20])
            right_list.append(1.0 if ir else 0.0)

    t_hamer_start = time.time()
    if boxes_list:
        boxes = np.stack(boxes_list).astype(np.float32)
        right = np.stack(right_list)
        last_boxes, last_right = boxes, right
    elif last_boxes is not None:
        boxes, right = last_boxes, last_right
    else:
        boxes = None
    if boxes is not None:
        dataset = ViTDetDataset(c, f, boxes, right, rescale_factor=2.0)
        loader = torch.utils.data.DataLoader(dataset, batch_size=4, shuffle=False, num_workers=0)

        for batch in loader:
            batch = recursive_to(batch, d)
            with torch.no_grad(): out = m(batch)

            for n in range(batch["img"].shape[0]):
                # pred_keypoints_2d: normalized crop coords [-0.5, 0.5]
                kn = out["pred_keypoints_2d"][n].cpu().numpy()
                kc = kn + 0.5  # [0, 1] within crop

                # Flip x for left hands (ViTDetDataset flips internally)
                is_right = batch["right"][n].item()
                if is_right < 0.5:  # left hand
                    kc[:,0] = 1.0 - kc[:,0]

                # Map from crop [0,1] to full image via bbox center & size
                cx = batch["box_center"][n, 0].item()
                cy = batch["box_center"][n, 1].item()
                bs = batch["box_size"][n].item()
                kp2d = np.zeros((21, 2), dtype=int)
                kp2d[:,0] = (cx - bs/2 + kc[:,0] * bs).astype(int)
                kp2d[:,1] = (cy - bs/2 + kc[:,1] * bs).astype(int)

                for e in HAND_EDGES:
                    cv2.line(canvas, tuple(kp2d[e[0]]), tuple(kp2d[e[1]]), (0,255,255), 2)
                for kp in kp2d:
                    cv2.circle(canvas, tuple(kp), 4, (0,255,0), -1)

                # Draw bbox
                col = (255,0,0) if is_right > 0.5 else (0,255,0)
                x1 = int(cx - bs/2); y1 = int(cy - bs/2)
                x2 = int(cx + bs/2); y2 = int(cy + bs/2)
                cv2.rectangle(canvas, (x1,y1), (x2,y2), col, 2)
                cv2.putText(canvas, "R" if is_right > 0.5 else "L",
                            (int(cx)-10, int(cy)+5), cv2.FONT_HERSHEY_SIMPLEX, .8, col, 2)

    t_hamer = time.time() - t_hamer_start
    fc += 1
    if time.time()-ft >= 1: fps,fc,ft = fc,0,time.time()
    t_total = time.time()-t0; cv2.putText(canvas,f"FPS:{fps} V:{t_vit-t_cap:.2f}s H:{t_hamer:.2f}s",(10,30),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,200,0),2)
    cv2.imshow("HAMER", canvas)
    if cv2.waitKey(1)&0xFF in (ord("q"),27): break
cap.release(); cv2.destroyAllWindows()





