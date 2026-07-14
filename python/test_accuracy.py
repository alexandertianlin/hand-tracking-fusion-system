"""
Hand detection accuracy test:
1. Show LEFT hand only (3s) → saves left_test.jpg
2. Show RIGHT hand only (3s) → saves right_test.jpg
3. Show BOTH hands (3s) → saves both_test.jpg
Reports confidence and hand count stats.
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
print("Loading ..."); sys.stdout.flush()
m, c = load_hamer(CKPT, init_renderer=False)
d = torch.device("cuda" if torch.cuda.is_available() else "cpu"); m = m.to(d).eval()
cpm = ViTPoseModel("cuda")
imsz = c.MODEL.IMAGE_SIZE
HAND_EDGES = [(0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),(0,9),(9,10),
              (10,11),(11,12),(0,13),(13,14),(14,15),(15,16),(0,17),(17,18),(18,19),(19,20)]
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap.isOpened(): cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
def process_frame(frame):
    h,w=frame.shape[:2]
    det=[np.array([[0,0,w,h,0.9]])]
    vit=cpm.predict_pose(frame,det)
    canvas=frame.copy(); info=[]
    for vp in vit:
        kps_all=vp["keypoints"]
        for hk,ir,side in [(kps_all[-42:-21],False,"L"),(kps_all[-21:],True,"R")]:
            v=hk[:,2]>0.5; cnt=int(sum(v))
            if cnt<4: continue
            cx=int(hk[v,0].mean()); cy=int(hk[v,1].mean())
            x1b=int(hk[v,0].min()); y1b=int(hk[v,1].min())
            x2b=int(hk[v,0].max()); y2b=int(hk[v,1].max())
            bsz=max(x2b-x1b,y2b-y1b)+40
            avg_conf=float(hk[v,2].mean()) if cnt>0 else 0
            info.append((side,cx,cy,cnt,avg_conf))
            ds=ViTDetDataset(c,frame,np.array([[x1b-20,y1b-20,x2b+20,y2b+20]]).astype(np.float32),
                             np.array([1.0 if ir else 0.0]),rescale_factor=2.0)
            ld=torch.utils.data.DataLoader(ds,batch_size=1,shuffle=False)
            for batch in ld:
                batch=recursive_to(batch,d)
                with torch.no_grad(): out=m(batch)
                kn=out["pred_keypoints_2d"][0].cpu().numpy()
                kc=kn+0.5
                if ir==0: kc[:,0]=1.0-kc[:,0]
                bs2=float(batch["box_size"][0].item())
                cx2=float(batch["box_center"][0,0].item())
                cy2=float(batch["box_center"][0,1].item())
                kp2d=np.zeros((21,2),dtype=int)
                kp2d[:,0]=(cx2-bs2/2+kc[:,0]*bs2).astype(int)
                kp2d[:,1]=(cy2-bs2/2+kc[:,1]*bs2).astype(int)
                col=(255,0,0)if ir else(0,255,0)
                cv2.rectangle(canvas,(int(cx2-bs2/2),int(cy2-bs2/2)),(int(cx2+bs2/2),int(cy2+bs2/2)),col,2)
                cv2.putText(canvas,side,(int(cx2)-10,int(cy2)+5),cv2.FONT_HERSHEY_SIMPLEX,.8,col,2)
                for e in HAND_EDGES: cv2.line(canvas,tuple(kp2d[e[0]]),tuple(kp2d[e[1]]),(0,255,255),2)
                for kp in kp2d: cv2.circle(canvas,tuple(kp),4,(0,255,0),-1)
    return canvas,info
print("Camera ready. Press SPACE to capture or q to quit.")
for phase in ["LEFT hand only","RIGHT hand only","BOTH hands"]:
    print(f"\n=== {phase} ===")
    print("Show your hand(s) and press SPACE to capture, q to skip...")
    while True:
        r,f=cap.read()
        if not r: break
        f=cv2.flip(f,1)
        annotated,info=process_frame(f)
        cv2.putText(annotated,f"Phase: {phase}  Hands: {[x[0] for x in info]}",(10,30),
                    cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,0),2)
        for i,(side,cx,cy,cnt,conf) in enumerate(info):
            y=60+i*25
            cv2.putText(annotated,f"{side}: {cnt}pts conf={conf:.2f} pos=({cx},{cy})",
                        (10,y),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,0),2)
        cv2.imshow("HAMER Accuracy Test",annotated)
        k=cv2.waitKey(30)&0xFF
        if k==ord(' ') or k==13:
            fname=f"D:\\hamer_data\\{phase.replace(' ','_')}.jpg"
            cv2.imwrite(fname,f)
            cv2.imwrite(fname.replace('.jpg','_annotated.jpg'),annotated)
            print(f"  Saved {fname}")
            print(f"  Hands: {[(x[0],f'{x[3]}pts,conf={x[4]:.2f}') for x in info]}")
            break
        elif k==ord('q'): break
cap.release(); cv2.destroyAllWindows()
print("\n=== Test Results ===")
print("Check these files:")
for phase in ["LEFT_hand_only","RIGHT_hand_only","BOTH_hands"]:
    print(f"  D:\\hamer_data\\{phase}.jpg (original)")
    print(f"  D:\\hamer_data\\{phase}_annotated.jpg (with markers)")
print("Compare if the hand markers align with your actual hand.") 
