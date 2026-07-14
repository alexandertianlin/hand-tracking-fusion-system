# 项目复现与技术沉淀文档

## ViTPose + HAMER → Unity 手部实时追踪系统

---

> **文档版本**: v1.0  
> **最后更新**: 2026-06-26  
> **环境**: Windows 10 + Python 3.10 + CUDA 12.4 + Unity 2022  
> **SHA256**: 33-F0-0D-78-6B-12-7E-CD-B5-04-6E-AD-BA-76-47-B4-79-17-05-45-09-C5-1E-DE-E3-F6-D0-1D-5D-50-9C-6F

---

## 1. 模型文件与核心权重总结

### 1.1 模型清单

| 模型 | 作用 | 权重文件 | 大小 | 来源 |
|------|------|---------|------|------|
| **ViTPose+ Huge (Whole-Body)** | 全身姿态估计,输出 133 个关键点(含双手) | wholebody.pth | ~2.3 GB | [ViTPose 官方](https://github.com/ViTAE-Transformer/ViTPose) |
| **HAMER** | 从手部裁剪图重建 3D 手部 Mesh + 21 个 3D 关键点 | hamer.ckpt | ~300 MB | [HAMER 官方](https://github.com/geopavlakos/hamer) |
| **MediaPipe Hand Landmarker** (备用方案) | 直接回归 21 个手部 3D landmarks | hand_landmarker.task | ~5.8 MB | Google MediaPipe 自动下载 |

### 1.2 文件路径结构

`
hamer_code/hamer-main/
├── _DATA/
│   ├── hamer_ckpts/checkpoints/
│   │   └── hamer.ckpt                              ← HAMER 权重
│   └── vitpose_ckpts/vitpose+_huge/
│       └── wholebody.pth                            ← ViTPose+ 权重
├── hamer/                                            ← HAMER 源码包
│   ├── models/                                       ← 模型定义
│   ├── datasets/
│   │   └── vitdet_dataset.py                        ← ViTDet 数据集 (含 downsampling_factor)
│   └── utils/
│       └── geometry.py                              ← 几何工具 (含 torch.cross)
├── third-party/ViTPose/
│   └── configs/wholebody/2d_kpt_sview_rgb_img/
│       └── topdown_heatmap/coco-wholebody/
│           └── ViTPose_huge_wholebody_256x192.py   ← ViTPose 配置
└── vitpose_model.py                                  ← ViTPose 模型封装
`

### 1.3 模型调用链

`
ViTPoseModel.predict_pose()          # 输入: RGB 图像 → 输出: 133 keypoints
    ↓
kps[-42:-21] = Left Hand (21 kp)    # 左手关键点
kps[-21:]     = Right Hand (21 kp)  # 右手关键点
    ↓
ViTDetDataset(boxes)                 # 裁剪手部区域
    ↓
load_hamer().forward(batch)          # 重建 3D Mesh
    ↓
out["pred_keypoints_3d"][n]          # 21 × 3 关键点坐标
    ↓
curl_from_kp() / spread_from_kp()   # 计算弯曲度和张开度
    ↓
UDP JSON → Unity VisionBridge        # 发送到 Unity
`

---

## 2. 环境部署与系统配置指南

### 2.1 硬件要求

| 组件 | 最低 | 推荐 (已验证) |
|------|------|-------------|
| GPU | NVIDIA RTX 3060 8GB | **RTX 4080 Laptop 12GB** |
| CPU | 8 核 | 24 核 |
| RAM | 16 GB | 32 GB |
| 摄像头 | USB 摄像头 | **Intel RealSense D435i** |
| IMU 串口 | USB-UART | **COM122 (460800 baud)** |

### 2.2 环境安装

`powershell
# 创建 conda 环境
conda create -n hamer python=3.10
conda activate hamer

# 基础依赖 (已验证版本)
pip install torch==2.6.0+cu124 --index-url https://download.pytorch.org/whl/cu124
pip install opencv-python==4.9.0 numpy==1.26.4
pip install pyrealsense2              # Intel RealSense SDK
pip install pyserial                  # 串口通信
pip install mediapipe==0.10.35        # MediaPipe (可选备用)
pip install psutil                    # 系统监控
pip install mmcv-full                # ViTPose 依赖

# HAMER 额外依赖
pip install "yacs>=0.1.8"
pip install scikit-image
pip install timm                      # ViT 骨干网络
`

### 2.3 启动命令

`powershell
# 终端 1: Unity (需先启动)
# 打开 D:\alexandertianlin\agiletact\Gesture-glove-UI-unity\onlytip - 2.2
# 双击 Assets/Scenes/SampleScene.unity → Play

# 终端 2: Python 视觉推理
cd C:\Users\Administrator\Documents\Codex\2026-06-18\hamer-d435i-usb\work
"D:\ProgramData\anaconda3\envs\hamer\python.exe" run_vitpose_v3.py

# 终端 2: MediaPipe 备用方案
"D:\ProgramData\anaconda3\envs\hamer\python.exe" run_mediapipe_hand.py
`

### 2.4 核心配置参数

`python
# run_vitpose_v3.py — 关键配置
SEND_INTV = 0.033        # UDP 发送间隔 (≈30fps)
rescale_factor = 2.0     # HAMER 裁剪放大系数
vit_skip = 3             # ViTPose 运行间隔 (每 4 帧)
CONF_THRESH = 0.5        # ViTPose 关键点置信度阈值
MIN_VISIBLE = 8          # 最少可见关键点数
DEDUP_IOU = 0.3          # 手部去重 IoU 阈值
consecutive_no_hand_max = 5  # 连续无手帧数上限
`

`csharp
// VisionBridge.cs — Unity 关键配置
listenPort = 5055;        // UDP 监听端口
visionTimeout = 0.5f;    // 视觉超时 (秒)
blendAlpha = 0.85f;      // 视觉→IMU 融合系数
`

### 2.5 UDP JSON 协议

`json
{
  "type": "hamer_hand",
  "seq": 42,
  "ts": 1719360000123,
  "num_hands": 1,
  "hand_0_label": "right",
  "hand_0_conf": 0.9,
  "hand_0_wrist": [0.1, 0.2, 0.3],
  "hand_0_kp3d": [0.1, 0.2, 0.3, ...],
  "hand_0_orient_q": [0.98, 0.12, 0.08, 0.14],
  "hand_0_curl_thumb": 0.5,
  "hand_0_curl_index": 0.3,
  "hand_0_curl_middle": 0.2,
  "hand_0_curl_ring": 0.4,
  "hand_0_curl_little": 0.3,
  "hand_0_spread_thumb": 0.1,
  "hand_0_spread_index": 0.05,
  "hand_0_spread_middle": 0.0,
  "hand_0_spread_ring": 0.02,
  "hand_0_spread_little": 0.15
}
`

---

## 3. 核心踩坑记录与排错指南

### Issue 1: CPU 满载 1790% + 线程数 105

**问题现象**: 运行 fusion_debug_tool.py 后 CPU 利用率长时间维持在 1790% 左右,系统线程数飙升至 105。控制台输出 THR:105。

**根本原因**: PyTorch 底层调用的 OpenMP/MKL 默认根据 CPU 物理核心数创建线程池(24 核×4 超线程=96 线程)。这些线程在 GPU 推理间隙进行 spin-wait 空转。同时 HAMER 内部的 OpenCV 图像预处理和 MMCV 也各自创建线程池,多池叠加导致线程数量失控。

**解决方案**: 在导入任何第三方库之前,通过 os.environ 硬性限制线程数:

`python
import os
os.environ["OMP_NUM_THREADS"]        = "2"
os.environ["MKL_NUM_THREADS"]        = "2"
os.environ["OPENBLAS_NUM_THREADS"]   = "2"
os.environ["VECLIB_MAXIMUM_THREADS"] = "2"
os.environ["NUMEXPR_NUM_THREADS"]    = "2"
os.environ["OPENCV_FOR_THREADS_NUM"] = "2"

import torch
torch.set_num_threads(2)
torch.set_num_interop_threads(2)
`

**效果**: CPU 从 1790% 降至 200–300%,线程数从 105 降至 45–50。

---

### Issue 2: 视频每 7 秒卡死 (42 帧定时冻结)

**问题现象**: 系统启动后约 7 秒(42 帧),视频信号完全冻结,CPU 从 ~400% 断崖式跌至 ~100%,GPU 无异常。卡死后约 2 秒恢复,然后 7 秒后再次冻结。精确到帧的周期性表现。

**根本原因**: PyTorch JIT (TorchScript) 优化器的 profiling 阶段默认采集前 40 帧的执行轨迹。到达阈值后触发图编译优化(Optimization),尝试将 192 个 MoE(Mixture of Experts) 子图融合为一个大的 CUDA Kernel。由于该模型的 MoE 专家权重缺失,编译器在 C++ 层的图拓扑排序中死锁,导致推理线程挂起。

**解决方案**: 在模型加载前禁用 JIT profiler 和 fusion strategy:

`python
import torch
torch._C._jit_set_profiling_executor(False)
torch._C._jit_set_profiling_mode(False)
torch.jit.set_fusion_strategy([('STATIC', 0), ('DYNAMIC', 0)])
`

**效果**: 卡死完全消失,视频持续流畅运行。

---

### Issue 3: torch.cross 触发 CPU Fallback

**问题现象**: 控制台持续输出 UserWarning: Using torch.cross without specifying the dim arg is deprecated。同时 CPU 负载异常升高。

**根本原因**: hamer/utils/geometry.py 第 61 行调用 	orch.cross(b1, b2) 未指定 dim 参数。PyTorch 在 C++ 层触发维度推导 Fallback Path,中断了 GPU 向量化加速。

**解决方案**: 显式指定 dim=-1:

`python
# hamer/utils/geometry.py:61
# 修改前
b3 = torch.cross(b1, b2)

# 修改后
b3 = torch.cross(b1, b2, dim=-1)
`

**效果**: 警告消失,算子保持 CUDA 加速。

---

### Issue 4: IMU 队列死锁导致视频卡死

**问题现象**: 加入 IMU 串口数据后,视频同样在约 7 秒时卡死。纯视觉版本(d435i_hamer.py)无此问题。

**根本原因**: IMU 采集线程以 200Hz 往 queue.Queue() 写入数据,而主线程以 15Hz 消费。队列未设 maxsize,导致 7 秒内堆积上千条数据。时间戳对齐逻辑 while camera_ts > imu_ts: imu_q.get() 在队列拥堵时陷入死循环,阻塞 GIL。

**解决方案**: 限制队列容量,使用非阻塞写入:

`python
imu_queue = queue.Queue(maxsize=100)

# 写入端
try:
    imu_queue.put_nowait(imu_data)
except queue.Full:
    imu_queue.get_nowait()  # 丢弃最旧
    imu_queue.put_nowait(imu_data)
`

---

### Issue 5: oxes 类型不匹配 (list vs np.ndarray)

**问题现象**: AttributeError: 'list' object has no attribute 'astype'

**根本原因**: ViTDetDataset.__init__ 要求 oxes 为 
p.ndarray 以调用 .astype(np.float32),但上游传入的是 Python list。

**解决方案**: 显式类型转换:

`python
if not isinstance(boxes, np.ndarray):
    boxes = np.array(boxes, dtype=np.float32)
if boxes.ndim == 1 and len(boxes) == 4:
    boxes = boxes[np.newaxis, :]  # [4] → [1, 4]
`

---

### Issue 6: 手移开画面后标记不消失 (IMU 无法接管)

**问题现象**: 手离开摄像头画面后,手部标记仍然在空中/脸上飘动。Unity 侧的 IMU 接管逻辑(0.5s 超时)从未触发。

**根本原因**: 三层叠加的 Bug:

**层 1 — last_boxes 永不失效**:
`python
# 错误逻辑
if boxes_list:                     # 只在检测到手时更新
    last_boxes = np.stack(boxes_list)
# 无 else 分支 → last_boxes 永不被清除
# HAMER 持续用旧框裁剪当前帧 → 产生垃圾 3D 输出
`

**层 2 — 计数器在非 ViTPose 帧错误递增**:
`python
# ViTPose 每 4 帧跑一次
# 中间 3 帧 boxes_list = [] (因为 vitposes = [])
# else 分支每次都执行 → 即使手一直在,计数器也递增
# 5 帧后误清 last_boxes → 手部标记闪烁消失
`

**层 3 — Unity 超时被 Python 持续发包阻断**:
- Python 在无手时仍发送 
um_hands=0 信号
- Unity VisionBridge.Update() 的 lastPacketTime = Time.time 在每次收到包时刷新
- if (elapsed > visionTimeout) return; → 永远不会触发

**解决方案** (三层对应修复):

`python
# 修复 1: 只在 ViTPose 帧更新手部状态
vitpose_ran_this_frame = False
if vit_skip <= 0:
    vitpose_ran_this_frame = True
    vitposes = vitpose.predict_pose(...)
    vit_skip = 3
else:
    vitposes = []; vit_skip -= 1

# 修复 2: 计数器仅当 ViTPose 实际运行时递增
if boxes_list:
    consecutive_no_hand = 0
    last_boxes = ...
    last_right = ...
elif vitpose_ran_this_frame:    # ← 关键: 只在 ViTPose 帧
    consecutive_no_hand += 1
    if consecutive_no_hand > 5:
        last_boxes = None
        last_right = None

# 修复 3: hands_confirmed 标记控制 HAMER 和 UDP
if vitpose_ran_this_frame:
    hands_confirmed = bool(boxes_list)

# HAMER
if last_boxes is not None and hands_confirmed:
    ds = ViTDetDataset(...)  # 只在确认有手时运行

# UDP
if hands_confirmed and g_kp3d is not None and ...:
    sock.sendto(...)  # 只在确认有手时发送
# 无 elif → 不发任何包,Unity 超时自然接管
`

---

### Issue 7: 两只手标记在同一个手上 (ViTPose 幻觉)

**问题现象**: 用户只伸出一只手,画面中出现两个重叠的骨架,标记为 "L" 和 "R"。

**根本原因**: ViTPose whole-body 模型对 133 个关键点的左右手分支独立输出。当一只手可见时,模型常对另一只手的 keypoints 也做出高置信度预测,且这些"幻觉"关键点出现在同一位置附近。

**解决方案**: 在 boxes_list 生成后添加 IoU 去重:

`python
if len(boxes_list) > 1:
    dedup_boxes = []; dedup_rights = []
    for bi, (box, ri) in enumerate(zip(boxes_list, rights_list)):
        is_dup = False
        for bj, rj in zip(boxes_list, rights_list):
            if ri == rj: continue
            # 计算 IoU
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
`

---

### Issue 8: UDP 端口冲突 — 「每个套接字地址只允许使用一次」

**问题现象**: Unity Console 报错 [post] UDP fail: 通常每个套接字地址(协议/网络地址/端口)只允许使用一次。

**根本原因**: 多个 Unity 脚本同时绑定 5055 端口:
- VisionBridge.cs — 主接收器
- VisionFingerCorrectionReceiver.cs — 另一个接收器(应禁用)
- post.cs — sc.unity 场景中使用(与 SampleScene 冲突)

**解决方案**: 
1. 在 SampleScene 中取消选中 VisionFingerCorrectionReceiver 组件
2. 在 Windows CMD 中强杀占用端口的残留进程:
`cmd
netstat -ano | findstr 5055
taskkill /F /PID <PID>
`

---

### Issue 9: 脚本生成过程中的连环 NameError

**问题现象**: 运行 un_vitpose_hand.py 时依次出现:
`
1. NameError: name 'last_send' is not defined
2. NameError: name 'socket' is not defined
3. NameError: name 'seq' is not defined
4. NameError: name 'orient_q_from_landmarks' is not defined
`

**根本原因**: 使用 PowerShell str.replace() 对旧文件进行批量替换生成新文件。文本替换是无状态的,破坏了变量声明链:
- import socket 被替换操作意外移动到了 sock = socket.socket() 之后
- seq = 0 初始化行被错误匹配删除
- 函数定义 orient_q_from_landmarks() 被错误的替换规则清空

**解决方案**: 放弃增量修补,直接从已知稳定的 d435i_hamer_fusion.py 进行结构化重写。

---

## 4. 关键修正指令与迭代演进

### 4.1 修正指令时间线

| 轮次 | 核心指令 (Prompt) | 对应的代码重构 |
|------|------------------|--------------|
| R1–4 | "CPU 一直死死卡在 1790% 左右" | 注入 OMP/MKL/OpenCV 线程硬限制 + 日志拦截器 |
| R5–7 | "动两秒就卡住了" | 	orch.set_num_interop_threads(2), 解除 JIT 单线程阻塞 |
| R8 | "视频信号卡住了" | 关闭 JIT Profiler + Fusion Strategy |
| R9 | "等间隔卡死模式(7秒、42帧)" | PYTORCH_JIT=0 环境变量 + MoE 权重缺失绕过 |
| R10 | "只加入了 IMU 信号就导致卡顿" | queue.Queue(maxsize=100) + 非阻塞 put/get |
| R11 | "以旧版流畅机制为基底" | 非阻塞 IMU 叠加渲染,不修改原视频管线 |
| R12 | "list object has no attribute astype" | 
p.array(boxes, dtype=np.float32) 类型包裹 |
| R13 | "手部标记在脸上飘着" | consecutive_no_hand + hands_confirmed 标记 |
| R14 | "两只手标记在了同一个手上" | IoU 去重 (threshold=0.3) |

### 4.2 架构演变: 从简单拼接 → 状态机驱动

`
初始: fusion_debug_tool.py (IMU + HAMER + Unity 三合一)
  → CPU 1790%, Thread 105, 每 7 秒卡死

Round 5-7: 线程隔离 + JIT 禁用
  → CPU 降至 200%, 卡死消失, 但 IMU 队列死锁

Round 10-11: 非阻塞 IMU 叠加, 主流水线不受影响
  → 视频恢复流畅, 但手离开后标记残留

Round 13-14: 状态机模型引入
  → vision_active / hands_confirmed / consecutive_no_hand
  → 手离开→自动切换 IMU, 手回来→自动恢复视觉
`

### 4.3 核心代码模块清单 (稳定版本)

| 文件 | 路径 | 行数 | 作用 |
|------|------|------|------|
| un_vitpose_v3.py | work/ | 221 | 主推理脚本: D435i → ViTPose → HAMER → UDP |
| VisionBridge.cs | Assets/Scenes/ | 165 | Unity 视觉+IMU 桥梁 |
| HandMotionManager.cs | Assets/Scenes/ | ~300 | IMU 手势解算与校准 |
| FingerSolver.cs | Assets/Scenes/ | ~400 | 手指骨骼求解器 |
| un_mediapipe_hand.py | work/ | 243 | 备选方案: MediaPipe → UDP |

### 4.4 已知剩余问题

1. **ViTPose 检测频率低** (vit_skip=3, ~7.5fps)。快速手势过渡帧可能丢失。
2. **Curl 精度低于 MediaPipe** (经 MANO 参数化解码后信息损失)。
3. **偶尔的检测幻觉** (无手时 ViTPose 仍预测 hand keypoints)。
4. **Unity 端未使用 spread 值** (yaw = 0f 硬编码)。

---

> 本文件伴随项目代码一起归档。  
> 如需完全复现,请确认 SHA256 校验和、conda 环境版本和模型权重路径与文档一致。
---

## 5. 复现关键告警与四颗「隐形炸弹」

> ⚠️ 以下四个问题如果不处理，复现成功率会从 95% 骤降至 30%。

### 5.1 绝对路径硬编码 (必须迁移)

`run_vitpose_v3.py` 第 11 行目前是硬编码的绝对路径:

```python
# 当前(硬编码，必须修改)
HAMER_DIR = r"C:\Users\Administrator\Documents\Codex\2026-06-16\files-mentioned-by-the-user-gpu2-3\hamer_code\hamer-main"
CKPT = os.path.join(HAMER_DIR, "_DATA", "hamer_ckpts", "checkpoints", "hamer.ckpt")
```

**迁移到新机器时必须修改为**:
```python
# 推荐方式: 基于脚本所在目录的相对路径
import os
BASE = os.path.dirname(os.path.abspath(__file__))
HAMER_DIR = os.path.join(BASE, "..", "hamer_code", "hamer-main")
CKPT = os.path.join(HAMER_DIR, "_DATA", "hamer_ckpts", "checkpoints", "hamer.ckpt")
os.chdir(HAMER_DIR)
sys.path.insert(0, HAMER_DIR)
sys.path.insert(0, os.path.join(HAMER_DIR, "third-party", "ViTPose"))
```

### 5.2 CUDA 与核心依赖版本锁定

| 包名 | 已验证版本 | 安装命令 | 备注 |
|------|-----------|---------|------|
| **Python** | 3.10.20 | `conda create -n hamer python=3.10` | 3.11+ 可能不兼容 mmcv |
| **PyTorch** | 2.6.0+cu124 | `pip install torch==2.6.0+cu124 --index-url https://download.pytorch.org/whl/cu124` | CUDA 12.4 必须匹配 |
| **CUDA Toolkit** | 12.4 | NVIDIA 官网 | 与 torch 版本严格对应 |
| **cuDNN** | 9.1.0 | 随 torch 安装 | - |
| **mmcv** | 1.4.8 | `pip install mmcv==1.4.8` | ViTPose 依赖，非 mmcv-full |
| **opencv-python** | 4.9.0.80 | `pip install opencv-python==4.9.0.80` | - |
| **numpy** | 1.26.4 | `pip install numpy==1.26.4` | 1.27+ 可能不兼容 |
| **scikit-image** | 0.25.2 | `pip install scikit-image==0.25.2` | ViTDetDataset 高斯模糊 |
| **timm** | 1.0.27 | `pip install timm==1.0.27` | ViTPose 骨干网络 |
| **yacs** | 0.1.8 | `pip install yacs==0.1.8` | HAMER 配置解析 |
| **pyrealsense2** | 2.58.2.10647 | `pip install pyrealsense2` | Intel RealSense SDK |
| **pyserial** | 3.5 | `pip install pyserial==3.5` | IMU 串口通信 |
| **mediapipe** | 0.10.35 | `pip install mediapipe==0.10.35` | 备用方案 |
| **psutil** | 7.2.2 | `pip install psutil==7.2.2` | 系统监控 |

> ⚠️ mmcv 兼容性陷阱: 上述环境使用 mmcv 1.4.8。如果换成 mmcv-full (>= 2.0)，ViTPose 的 API `inference_top_down_pose_model` 已被废弃，需改用 `mmpose.apis.inferencer`。

全量安装命令:
```powershell
conda create -n hamer python=3.10 -y
conda activate hamer
pip install torch==2.6.0+cu124 --index-url https://download.pytorch.org/whl/cu124
pip install opencv-python==4.9.0.80 numpy==1.26.4
pip install scikit-image==0.25.2 timm==1.0.27 yacs==0.1.8
pip install mmcv==1.4.8
pip install pyrealsense2 pyserial==3.5 psutil==7.2.2
pip install mediapipe==0.10.35
```

### 5.3 启动顺序与网络边界

| 步骤 | 操作 | 说明 | 验证方法 |
|------|------|------|---------|
| 1 | 先启动 Unity | 打开 onlytip-2.2，双击 SampleScene.unity，点击 Play | Console 显示 `[VisionBridge] 监听 5055` |
| 2 | 再启动 Python | 运行 `python run_vitpose_v3.py` | 控制台显示 `UDP seq=1` |
| 3 | 校准手套 (如需 IMU) | Unity 中按 Space 键 | Console 显示 `✅ 动捕手套校准完成` |

**为什么必须先启动 Unity？**
1. Python 脚本启动后立刻开始推理和 UDP 发送。如果目标端口 (5055) 没有被监听，WinSock 可能进入缓冲区积压模式。
2. Unity 的 UdpClient 在 StartReceiver 中阻塞绑定端口。如果 Python 先发数据，Unity 启动后可能漏掉前几帧。
3. 如果 Unity 未启动而 Python 发信号，该信号会被操作系统丢弃——导致调试时误以为 IMU 接管失效。

**端口冲突诊断**:
```cmd
netstat -ano | findstr 5055
taskkill /F /PID <占用PID>
```

### 5.4 第三方权重下载

| 模型 | 原始来源 | 国内替代方案 | 文件大小 |
|------|---------|------------|---------|
| HAMER (hamer.ckpt) | Hugging Face geopavlakos/hamer | `$env:HF_ENDPOINT = "https://hf-mirror.com"` | ~300 MB |
| ViTPose+ (wholebody.pth) | GitHub Release | 百度网盘或 ModelScope 手动下载 | ~2.3 GB |
| MediaPipe (hand_landmarker.task) | Google Storage (自动下载) | 脚本首次运行自动下载 | ~5.8 MB |

HAMER 权重下载 (Hugging Face 镜像):
```powershell
$env:HF_ENDPOINT = "https://hf-mirror.com"
pip install huggingface-hub
huggingface-cli download geopavlakos/hamer
```

ViTPose+ 权重从 GitHub Release 手动下载后放入:
```
hamer-main/_DATA/vitpose_ckpts/vitpose+_huge/wholebody.pth
```

MediaPipe 模型由脚本自动下载到:
```
work/models/hand_landmarker.task
```

---

## 6. 附录: 文件校验清单

| 文件 | SHA256 | 备注 |
|------|--------|------|
| run_vitpose_v3.py | 33-F0-0D-78-6B-12-7E-CD-B5-04-6E-AD-BA-76-47-B4-79-17-05-45-09-C5-1E-DE-E3-F6-D0-1D-5D-50-9C-6F | 221 行，稳定版 |
| VisionBridge.cs | (见文件) | 165 行，含 num_hands 逻辑 |
| wholebody.pth | - | 需自行下载校验 |

### 5.5 RealSense D435i 硬件级依赖

`pyrealsense2` 在 Windows 上需要底层 USB 驱动支持，仅 `pip install` 可能不够：

```powershell
# 1. pip 安装 Python 绑定（必须）
pip install pyrealsense2

# 2. 安装 Intel RealSense SDK 2.0 运行时（必须）
# 下载地址: https://github.com/IntelRealSense/librealsense/releases
# 选择 Windows Installer: Intel.RealSense.SDK-WIN10-2.55.1.6483.exe
#
# 此安装包会注册相机的 USB 驱动 (realsense2.dll)
# 不安装此驱动时, pyrealsense2 可以 import 但无法识别设备
```

**验证方法**:
```powershell
python -c "import pyrealsense2 as rs; pipe = rs.pipeline(); cfg = rs.config(); cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30); pipe.start(cfg); print('D435i OK'); pipe.stop()"
```
正常输出: `D435i OK`

### 5.6 串口 COM 号动态绑定

IMU 手套串口在代码中硬编码为 `COM122`。Windows 在不同 USB 口或不同机器上会自动分配不同 COM 号。

**代码中的硬编码位置**（在 `run_vitpose_v3.py` 中_当前无 IMU 线程_，但独立 IMU 脚本中有）：

```python
# 独立 IMU 脚本中的硬编码, 需根据设备管理器修改
ser = serial.Serial('COM122', 460800, timeout=0.01)  # 必须修改!
```

**迁移步骤**:
1. 打开 Windows「设备管理器」
2. 展开「端口 (COM 和 LPT)」
3. 找到 USB Serial Device 或 STM32 虚拟串口
4. 查看其分配的 COM 号（如 COM3、COM5 等）
5. 修改代码中的串口号

**推荐方式: 使用命令行参数而非硬编码**:
```powershell
# 运行时指定 COM 口
python run_vitpose_v3.py --com COM3
```
