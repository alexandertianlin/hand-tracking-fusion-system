"""Download HAMER demo data to D:\hamer_data"""
import os, tarfile
BASE = r"D:\hamer_data"
os.makedirs(BASE, exist_ok=True)
TAR_PATH = os.path.join(BASE, "hamer_demo_data.tar.gz")
DATA_DIR = os.path.join(BASE, "_DATA")

def download():
    print(f"Downloading -> {TAR_PATH}  (~6 GB, 1-2 hrs)")
    import gdown
    url = "https://drive.google.com/uc?id=1mv7CUAnm73oKsEEG1xE3xH2C_oqcFSzT"
    gdown.download(url, TAR_PATH, quiet=False)
    if not os.path.exists(TAR_PATH):
        print("Download failed!"); return False
    sz = os.path.getsize(TAR_PATH)
    print(f"Downloaded: {sz/1024**3:.2f} GB"); return True

def extract():
    print("Extracting ...")
    with tarfile.open(TAR_PATH, "r:gz") as tar:
        for m in tar.getmembers():
            p = m.path
            if p.startswith("_DATA/"): m.path = p[6:]
            elif p.startswith("_DATA"): m.path = p[5:]
            tar.extract(m, path=DATA_DIR)
    print(f"Extracted to {DATA_DIR}")
    ckpt = os.path.join(DATA_DIR, "hamer_ckpts","checkpoints","hamer.ckpt")
    cfg  = os.path.join(DATA_DIR, "hamer_ckpts","model_config.yaml")
    if os.path.exists(ckpt): print(f"Checkpoint: {os.path.getsize(ckpt)/1024**3:.2f} GB")
    if os.path.exists(cfg):  print("Config: OK")
    return os.path.exists(ckpt) and os.path.exists(cfg)

if __name__ == "__main__":
    ckpt = os.path.join(DATA_DIR, "hamer_ckpts","checkpoints","hamer.ckpt")
    if not os.path.exists(ckpt):
        if download() and extract():
            if os.path.exists(TAR_PATH): os.remove(TAR_PATH)
            print("=== HAMER demo data ready ===")
    else:
        print("Checkpoint already exists")
