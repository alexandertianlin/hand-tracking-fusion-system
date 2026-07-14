using UnityEngine;
using System.IO;
using System.Linq;

public enum CameraMode
{
    Default,        // 原始双机位分屏展示
    AutoOrbit,      // 基础自动环绕
    Manual,         // 自由控制
    FistCloseUp,    // 握拳特写
    HeroOrbit,      // 英雄环绕
    ThumbCloseUp,   // 大拇指特写1
    IndexCloseUp,   // 食指特写1
    MiddleCloseUp,  // 中指特写1
    RingCloseUp,    // 无名指特写1
    LittleCloseUp,  // 小指特写1
    ThumbCloseUp2,  // 大拇指特写2
    IndexCloseUp2,  // 食指特写2
    MiddleCloseUp2, // 中指特写2
    RingCloseUp2,   // 无名指特写2
    LittleCloseUp2  // 小指特写2
}

[System.Serializable]
public class CameraPreset
{
    public Vector3 position;
    public Quaternion rotation;
    public float fov = 60f;
}

public class CameraController : MonoBehaviour
{
    [Header("=== 双相机配置 ===")]
    public Camera mainCamera;
    public Camera secondCamera;

    [Header("=== 通用设置 ===")]
    public Transform target;
    public HandMotionManager motionManager;
    public float transitionTime = 0.3f;
    public Canvas uiCanvas;

    [Header("=== 基础自动环绕 ===")]
    public float orbitDistance = 2.0f;
    public float orbitHeightOffset = 0.5f;
    public float orbitRotationSpeed = 30f;
    public float minOrbitSpeed = 5f;
    public float maxOrbitSpeed = 100f;

    [Header("=== 鼠标控制设置 ===")]
    public float rotationSpeed = 200f;
    public float panSpeed = 0.1f;
    public float zoomSpeed = 5f;

    [Header("=== 英雄环绕模式 ===")]
    public float heroBaseDistance = 1.8f;
    public float heroMaxDistance = 2.5f;
    public float heroMinDistance = 1.0f;
    public float heroBaseSpeed = 20f;
    public float heroMaxSpeedMultiplier = 3f;
    public float heroMinFOV = 50f;
    public float heroMaxFOV = 65f;
    public bool autoFocusActiveFinger = true;
    public bool gestureAutoSwitch = true;

    [Header("✨ 特写自动环绕 ===")]
    [Tooltip("切换到特写模式时自动开始英雄环绕")]
    public bool autoOrbitInCloseUp = true;
    [Tooltip("鼠标停止后多久恢复环绕（秒）")]
    public float resumeOrbitDelay = 0.5f;
    [Tooltip("特写环绕的基础速度")]
    public float closeUpOrbitSpeed = 15f;

    [Header("✨ 预设镜头 ===")]
    public CameraPreset fistPreset = new CameraPreset();
    public CameraPreset thumbPreset = new CameraPreset();
    public CameraPreset indexPreset = new CameraPreset();
    public CameraPreset middlePreset = new CameraPreset();
    public CameraPreset ringPreset = new CameraPreset();
    public CameraPreset littlePreset = new CameraPreset();

    [Header("✨ 第二组手指特写 ===")]
    public CameraPreset thumbPreset2 = new CameraPreset();
    public CameraPreset indexPreset2 = new CameraPreset();
    public CameraPreset middlePreset2 = new CameraPreset();
    public CameraPreset ringPreset2 = new CameraPreset();
    public CameraPreset littlePreset2 = new CameraPreset();

    // 内部状态
    private CameraMode currentMode;
    private Vector3 mainDefaultPosition;
    private Quaternion mainDefaultRotation;
    private float mainDefaultFOV;

    private bool secondCameraDefaultEnabled;
    private Rect secondCameraDefaultViewport;

    private float currentAngle;
    private Vector3 positionVelocity;
    private Quaternion rotationVelocity;
    private float fovVelocity;
    private bool isOrbitPaused = false;
    private bool isTransitioning = false;

    // 英雄环绕专用
    private Vector3 heroFocusPoint;
    private Vector3 focusPointVelocity;
    private float lastPalmAngle;
    private float palmAngularVelocity;

    // 特写自动环绕专用
    private float lastMouseInputTime;
    private bool isUserControlling = false;
    private Transform currentOrbitTarget;

    private FingerSolver[] fingersCache;

    void Start()
    {
        // 保存初始状态
        mainDefaultPosition = mainCamera.transform.position;
        mainDefaultRotation = mainCamera.transform.rotation;
        mainDefaultFOV = mainCamera.fieldOfView;

        if (secondCamera != null)
        {
            secondCameraDefaultEnabled = secondCamera.enabled;
            secondCameraDefaultViewport = secondCamera.rect;
        }

        if (motionManager != null)
        {
            fingersCache = new[] {
                motionManager.thumb,
                motionManager.index,
                motionManager.middle,
                motionManager.ring,
                motionManager.little
            };
        }

        currentMode = CameraMode.Default;
        UpdateCameraEnableState();

        Debug.Log("=== 相机控制器使用说明 ===");
        Debug.Log("【模式切换】1:双机位 | 2:环绕 | 3:自由 | 4:握拳 | 5:英雄");
        Debug.Log("【第一组特写】6:大拇指 | 7:食指 | 8:中指 | 9:无名指 | 0:小指");
        Debug.Log("【第二组特写】Shift+6:大拇指 | Shift+7:食指 | Shift+8:中指 | Shift+9:无名指 | Shift+0:小指");
        Debug.Log("✨ 特写模式自动环绕：动鼠标接管，停鼠标自动恢复");
        Debug.Log("【鼠标操作】右键旋转 | 中键平移 | 滚轮缩放");
        Debug.Log("【保存预设】调整好视角后按 Ctrl+S 保存到当前模式");
        Debug.Log("【其他】空格:暂停环绕 | R:重置 | H:隐藏UI | F12:截图");

        // 初始化所有预设的默认值
        if (fistPreset.rotation == Quaternion.identity) fistPreset.rotation = mainDefaultRotation;
        if (thumbPreset.rotation == Quaternion.identity) thumbPreset.rotation = mainDefaultRotation;
        if (indexPreset.rotation == Quaternion.identity) indexPreset.rotation = mainDefaultRotation;
        if (middlePreset.rotation == Quaternion.identity) middlePreset.rotation = mainDefaultRotation;
        if (ringPreset.rotation == Quaternion.identity) ringPreset.rotation = mainDefaultRotation;
        if (littlePreset.rotation == Quaternion.identity) littlePreset.rotation = mainDefaultRotation;

        if (thumbPreset2.rotation == Quaternion.identity) thumbPreset2.rotation = mainDefaultRotation;
        if (indexPreset2.rotation == Quaternion.identity) indexPreset2.rotation = mainDefaultRotation;
        if (middlePreset2.rotation == Quaternion.identity) middlePreset2.rotation = mainDefaultRotation;
        if (ringPreset2.rotation == Quaternion.identity) ringPreset2.rotation = mainDefaultRotation;
        if (littlePreset2.rotation == Quaternion.identity) littlePreset2.rotation = mainDefaultRotation;
    }

    void OnDisable()
    {
        SwitchToDefault();
    }

    void Update()
    {
        // ==================== 全局鼠标控制（永远优先响应）====================
        bool mouseMoved = UpdateGlobalMouseControl();

        if (mouseMoved)
        {
            lastMouseInputTime = Time.time;
            isUserControlling = true;
            isTransitioning = false;
            positionVelocity = Vector3.zero;
            rotationVelocity = Quaternion.identity;
            fovVelocity = 0f;
        }
        // 鼠标停止一段时间后自动恢复环绕
        else if (isUserControlling && Time.time - lastMouseInputTime > resumeOrbitDelay)
        {
            isUserControlling = false;
            // 从当前相机位置重新初始化环绕角度
            currentAngle = Mathf.Atan2(
                mainCamera.transform.position.x - currentOrbitTarget.position.x,
                mainCamera.transform.position.z - currentOrbitTarget.position.z
            ) * Mathf.Rad2Deg;
        }

        // ==================== 模式切换快捷键 ====================
        if (Input.GetKeyDown(KeyCode.Alpha1)) SwitchToDefault();
        if (Input.GetKeyDown(KeyCode.Alpha2)) SwitchToAutoOrbit();
        if (Input.GetKeyDown(KeyCode.Alpha3)) SwitchToManual();
        if (Input.GetKeyDown(KeyCode.Alpha4)) SwitchToCloseUp(CameraMode.FistCloseUp, fistPreset, target);
        if (Input.GetKeyDown(KeyCode.Alpha5)) SwitchToHeroOrbit();

        // 第一组手指特写（6-0）
        if (Input.GetKeyDown(KeyCode.Alpha6) && motionManager?.thumb?.tipBone != null)
            SwitchToCloseUp(CameraMode.ThumbCloseUp, thumbPreset, motionManager.thumb.tipBone);
        if (Input.GetKeyDown(KeyCode.Alpha7) && motionManager?.index?.tipBone != null)
            SwitchToCloseUp(CameraMode.IndexCloseUp, indexPreset, motionManager.index.tipBone);
        if (Input.GetKeyDown(KeyCode.Alpha8) && motionManager?.middle?.tipBone != null)
            SwitchToCloseUp(CameraMode.MiddleCloseUp, middlePreset, motionManager.middle.tipBone);
        if (Input.GetKeyDown(KeyCode.Alpha9) && motionManager?.ring?.tipBone != null)
            SwitchToCloseUp(CameraMode.RingCloseUp, ringPreset, motionManager.ring.tipBone);
        if (Input.GetKeyDown(KeyCode.Alpha0) && motionManager?.little?.tipBone != null)
            SwitchToCloseUp(CameraMode.LittleCloseUp, littlePreset, motionManager.little.tipBone);
        // ✨ 第二组手指特写（Shift+6~Shift+0）
        if (Input.GetKey(KeyCode.LeftShift) && Input.GetKeyDown(KeyCode.Alpha6) && motionManager?.thumb?.tipBone != null)
            SwitchToCloseUp(CameraMode.ThumbCloseUp2, thumbPreset2, motionManager.thumb.tipBone);
        if (Input.GetKey(KeyCode.LeftShift) && Input.GetKeyDown(KeyCode.Alpha7) && motionManager?.index?.tipBone != null)
            SwitchToCloseUp(CameraMode.IndexCloseUp2, indexPreset2, motionManager.index.tipBone);
        if (Input.GetKey(KeyCode.LeftShift) && Input.GetKeyDown(KeyCode.Alpha8) && motionManager?.middle?.tipBone != null)
            SwitchToCloseUp(CameraMode.MiddleCloseUp2, middlePreset2, motionManager.middle.tipBone);
        if (Input.GetKey(KeyCode.LeftShift) && Input.GetKeyDown(KeyCode.Alpha9) && motionManager?.ring?.tipBone != null)
            SwitchToCloseUp(CameraMode.RingCloseUp2, ringPreset2, motionManager.ring.tipBone);
        if (Input.GetKey(KeyCode.LeftShift) && Input.GetKeyDown(KeyCode.Alpha0) && motionManager?.little?.tipBone != null)
            SwitchToCloseUp(CameraMode.LittleCloseUp2, littlePreset2, motionManager.little.tipBone);
        // ==================== 一键保存当前视角 ====================
        if (Input.GetKey(KeyCode.LeftControl) && Input.GetKeyDown(KeyCode.S))
        {
            SaveCurrentPreset();
        }

        // ==================== 其他快捷键 ====================
        if ((currentMode == CameraMode.AutoOrbit || currentMode == CameraMode.HeroOrbit || IsCloseUpMode())
            && Input.GetKeyDown(KeyCode.Space))
        {
            isOrbitPaused = !isOrbitPaused;
            Debug.Log(isOrbitPaused ? "环绕已暂停" : "环绕已继续");
        }

        if (Input.GetKeyDown(KeyCode.R))
        {
            mainCamera.transform.position = mainDefaultPosition;
            mainCamera.transform.rotation = mainDefaultRotation;
            mainCamera.fieldOfView = mainDefaultFOV;
            Debug.Log("相机已重置");
        }

        if (Input.GetKeyDown(KeyCode.H) && uiCanvas != null)
        {
            uiCanvas.enabled = !uiCanvas.enabled;
            Debug.Log(uiCanvas.enabled ? "UI已显示" : "UI已隐藏");
        }

        if (Input.GetKeyDown(KeyCode.F12)) TakeScreenshot();

        // 手势自动切换
        if (gestureAutoSwitch && motionManager != null && motionManager.IsCalibrated)
        {
            if (currentMode == CameraMode.HeroOrbit || currentMode == CameraMode.AutoOrbit)
            {
                bool isFist = fingersCache.All(f => f.CurrentPitch > 70f);
                if (isFist) SwitchToCloseUp(CameraMode.FistCloseUp, fistPreset, target);
            }
        }
    }

    void LateUpdate()
    {
        if (currentMode == CameraMode.Default || currentMode == CameraMode.Manual || isUserControlling)
            return;

        if (currentMode == CameraMode.AutoOrbit)
        {
            UpdateAutoOrbitMode();
        }
        else if (currentMode == CameraMode.HeroOrbit)
        {
            UpdateHeroOrbitMode();
        }
        // 特写模式自动环绕
        else if (IsCloseUpMode() && autoOrbitInCloseUp && !isOrbitPaused)
        {
            UpdateCloseUpOrbitMode();
        }
    }

    // ==================== 全局鼠标控制 ====================
    private bool UpdateGlobalMouseControl()
    {
        bool moved = false;

        // 右键旋转
        if (Input.GetMouseButton(1))
        {
            float mouseX = Input.GetAxis("Mouse X") * rotationSpeed * 0.02f;
            float mouseY = Input.GetAxis("Mouse Y") * rotationSpeed * 0.02f;

            mainCamera.transform.Rotate(Vector3.up, mouseX, Space.World);
            mainCamera.transform.Rotate(Vector3.right, -mouseY, Space.Self);

            Vector3 euler = mainCamera.transform.eulerAngles;
            euler.z = 0f;
            mainCamera.transform.eulerAngles = euler;

            moved = true;
        }

        // 中键平移
        if (Input.GetMouseButton(2))
        {
            float moveX = Input.GetAxis("Mouse X") * panSpeed;
            float moveY = Input.GetAxis("Mouse Y") * panSpeed;

            mainCamera.transform.Translate(-moveX, -moveY, 0, Space.Self);
            moved = true;
        }

        // 滚轮缩放
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(scroll) > 0.001f)
        {
            mainCamera.transform.Translate(0, 0, scroll * zoomSpeed, Space.Self);
            moved = true;
        }

        return moved;
    }

    // ==================== 模式切换方法 ====================
    public void SwitchToDefault()
    {
        currentMode = CameraMode.Default;
        isOrbitPaused = false;
        isTransitioning = false;
        isUserControlling = false;
        UpdateCameraEnableState();

        mainCamera.transform.position = mainDefaultPosition;
        mainCamera.transform.rotation = mainDefaultRotation;
        mainCamera.fieldOfView = mainDefaultFOV;

        Debug.Log("已切换到：双机位默认视图");
    }

    public void SwitchToAutoOrbit()
    {
        currentMode = CameraMode.AutoOrbit;
        currentAngle = Mathf.Atan2(
            mainCamera.transform.position.x - target.position.x,
            mainCamera.transform.position.z - target.position.z
        ) * Mathf.Rad2Deg;
        isOrbitPaused = false;
        isTransitioning = true;
        isUserControlling = false;
        UpdateCameraEnableState();
        Debug.Log("已切换到：基础自动环绕");
    }

    public void SwitchToManual()
    {
        currentMode = CameraMode.Manual;
        isOrbitPaused = false;
        isTransitioning = false;
        isUserControlling = false;
        UpdateCameraEnableState();
        Debug.Log("已切换到：自由控制模式");
    }

    public void SwitchToHeroOrbit()
    {
        currentMode = CameraMode.HeroOrbit;
        currentAngle = Mathf.Atan2(
            mainCamera.transform.position.x - target.position.x,
            mainCamera.transform.position.z - target.position.z
        ) * Mathf.Rad2Deg;
        heroFocusPoint = target.position;
        isOrbitPaused = false;
        isTransitioning = true;
        isUserControlling = false;
        UpdateCameraEnableState();
        Debug.Log("已切换到：✨英雄环绕");
    }

    // 新的特写模式切换方法
    private void SwitchToCloseUp(CameraMode mode, CameraPreset preset, Transform orbitTarget)
    {
        currentMode = mode;
        currentOrbitTarget = orbitTarget;
        isOrbitPaused = false;
        isTransitioning = true;
        isUserControlling = false;
        UpdateCameraEnableState();

        // 平滑过渡到预设位置
        StartCoroutine(SmoothTransitionToPreset(preset));

        // 从预设位置初始化环绕角度
        currentAngle = Mathf.Atan2(
            preset.position.x - orbitTarget.position.x,
            preset.position.z - orbitTarget.position.z
        ) * Mathf.Rad2Deg;

        Debug.Log($"已切换到：{mode}");
    }

    private System.Collections.IEnumerator SmoothTransitionToPreset(CameraPreset preset)
    {
        if (preset.rotation == Quaternion.identity && preset.position == Vector3.zero)
        {
            Debug.LogWarning("当前预设还未保存过，请先调整视角并按Ctrl+S保存");
            isTransitioning = false;
            yield break;
        }

        float t = 0f;
        Vector3 startPos = mainCamera.transform.position;
        Quaternion startRot = mainCamera.transform.rotation;
        float startFOV = mainCamera.fieldOfView;

        if (Vector3.Distance(startPos, preset.position) < 0.001f
            && Quaternion.Angle(startRot, preset.rotation) < 0.1f
            && Mathf.Abs(startFOV - preset.fov) < 0.1f)
        {
            isTransitioning = false;
            yield break;
        }

        while (t < transitionTime && isTransitioning && !isUserControlling)
        {
            t += Time.deltaTime;
            float progress = t / transitionTime;

            mainCamera.transform.position = Vector3.Lerp(startPos, preset.position, progress);
            mainCamera.transform.rotation = Quaternion.Slerp(startRot, preset.rotation, progress);
            mainCamera.fieldOfView = Mathf.Lerp(startFOV, preset.fov, progress);

            yield return null;
        }

        if (isTransitioning && !isUserControlling)
        {
            mainCamera.transform.position = preset.position;
            mainCamera.transform.rotation = preset.rotation;
            mainCamera.fieldOfView = preset.fov;
        }

        isTransitioning = false;
    }

    // ==================== 相机启用状态控制 ====================
    private void UpdateCameraEnableState()
    {
        if (secondCamera == null) return;

        if (currentMode == CameraMode.Default)
        {
            secondCamera.enabled = secondCameraDefaultEnabled;
            secondCamera.rect = secondCameraDefaultViewport;
            mainCamera.rect = new Rect(0.5f, 0, 1, 1);
        }
        else
        {
            secondCamera.enabled = false;
            mainCamera.rect = new Rect(0, 0, 1, 1);
        }
    }

    // ==================== 各模式更新逻辑 ====================
    private void UpdateAutoOrbitMode()
    {
        if (target == null) return;

        if (!isOrbitPaused)
        {
            currentAngle += orbitRotationSpeed * Time.deltaTime;
        }

        Vector3 targetPosition = target.position + new Vector3(
            Mathf.Sin(currentAngle * Mathf.Deg2Rad) * orbitDistance,
            orbitHeightOffset,
            Mathf.Cos(currentAngle * Mathf.Deg2Rad) * orbitDistance
        );

        mainCamera.transform.position = Vector3.SmoothDamp(
            mainCamera.transform.position,
            targetPosition,
            ref positionVelocity,
            transitionTime
        );

        Quaternion targetRotation = Quaternion.LookRotation(
            target.position + Vector3.up * orbitHeightOffset - mainCamera.transform.position
        );

        mainCamera.transform.rotation = Quaternion.Slerp(
            mainCamera.transform.rotation,
            targetRotation,
            1f - Mathf.Exp(-transitionTime * Time.deltaTime * 10f)
        );

        mainCamera.fieldOfView = Mathf.SmoothDamp(
            mainCamera.fieldOfView,
            mainDefaultFOV,
            ref fovVelocity,
            transitionTime
        );
    }

    private void UpdateHeroOrbitMode()
    {
        if (target == null || motionManager == null || !motionManager.IsCalibrated)
        {
            UpdateAutoOrbitMode();
            return;
        }

        float currentPalmAngle = motionManager.palmBone.localEulerAngles.y;
        palmAngularVelocity = Mathf.Abs(Mathf.DeltaAngle(lastPalmAngle, currentPalmAngle)) / Time.deltaTime;
        lastPalmAngle = currentPalmAngle;

        float dynamicSpeed = heroBaseSpeed * Mathf.Lerp(1f, heroMaxSpeedMultiplier, palmAngularVelocity / 100f);

        if (!isOrbitPaused)
        {
            currentAngle += dynamicSpeed * Time.deltaTime;
        }

        float averageBend = fingersCache.Average(f => f.CurrentPitch / f.MaxPitch);
        float dynamicDistance = Mathf.Lerp(heroMaxDistance, heroMinDistance, averageBend);
        float dynamicFOV = Mathf.Lerp(heroMinFOV, heroMaxFOV, palmAngularVelocity / 200f);

        if (autoFocusActiveFinger)
        {
            float maxDelta = 0f;
            Transform activeFinger = motionManager.palmBone;

            foreach (var finger in fingersCache)
            {
                float fingerSpeed = Mathf.Abs(finger.CurrentPitch - (finger.CurrentPitch - Time.deltaTime));
                if (fingerSpeed > maxDelta && finger.tipBone != null)
                {
                    maxDelta = fingerSpeed;
                    activeFinger = finger.tipBone;
                }
            }

            heroFocusPoint = Vector3.SmoothDamp(
                heroFocusPoint,
                activeFinger.position,
                ref focusPointVelocity,
                0.2f
            );
        }
        else
        {
            heroFocusPoint = Vector3.SmoothDamp(
                heroFocusPoint,
                target.position,
                ref focusPointVelocity,
                0.2f
            );
        }

        Vector3 targetPosition = heroFocusPoint + new Vector3(
            Mathf.Sin(currentAngle * Mathf.Deg2Rad) * dynamicDistance,
            orbitHeightOffset,
            Mathf.Cos(currentAngle * Mathf.Deg2Rad) * dynamicDistance
        );

        mainCamera.transform.position = Vector3.SmoothDamp(
            mainCamera.transform.position,
            targetPosition,
            ref positionVelocity,
            transitionTime
        );

        Quaternion targetRotation = Quaternion.LookRotation(heroFocusPoint - mainCamera.transform.position);
        mainCamera.transform.rotation = Quaternion.Slerp(
            mainCamera.transform.rotation,
            targetRotation,
            1f - Mathf.Exp(-transitionTime * Time.deltaTime * 10f)
        );

        mainCamera.fieldOfView = Mathf.SmoothDamp(
            mainCamera.fieldOfView,
            dynamicFOV,
            ref fovVelocity,
            transitionTime
        );
    }

    // 特写模式环绕逻辑（围绕对应手指尖旋转）
    private void UpdateCloseUpOrbitMode()
    {
        if (currentOrbitTarget == null) return;

        float distance = Vector3.Distance(mainCamera.transform.position, currentOrbitTarget.position);

        if (!isOrbitPaused)
        {
            currentAngle += closeUpOrbitSpeed * Time.deltaTime;
        }

        Vector3 targetPosition = currentOrbitTarget.position + new Vector3(
            Mathf.Sin(currentAngle * Mathf.Deg2Rad) * distance,
            mainCamera.transform.position.y - currentOrbitTarget.position.y,
            Mathf.Cos(currentAngle * Mathf.Deg2Rad) * distance
        );

        mainCamera.transform.position = Vector3.SmoothDamp(
            mainCamera.transform.position,
            targetPosition,
            ref positionVelocity,
            0.1f
        );

        Quaternion targetRotation = Quaternion.LookRotation(currentOrbitTarget.position - mainCamera.transform.position);
        mainCamera.transform.rotation = Quaternion.Slerp(
            mainCamera.transform.rotation,
            targetRotation,
            1f - Mathf.Exp(-0.1f * Time.deltaTime * 10f)
        );
    }

    // ==================== 辅助方法 ====================
    private bool IsCloseUpMode()
    {
        return currentMode == CameraMode.FistCloseUp
            || currentMode == CameraMode.ThumbCloseUp
            || currentMode == CameraMode.IndexCloseUp
            || currentMode == CameraMode.MiddleCloseUp
            || currentMode == CameraMode.RingCloseUp
            || currentMode == CameraMode.LittleCloseUp
            || currentMode == CameraMode.ThumbCloseUp2
            || currentMode == CameraMode.IndexCloseUp2
            || currentMode == CameraMode.MiddleCloseUp2
            || currentMode == CameraMode.RingCloseUp2
            || currentMode == CameraMode.LittleCloseUp2;
    }

    private void TakeScreenshot()
    {
        string desktopPath = System.Environment.GetFolderPath(System.Environment.SpecialFolder.Desktop);
        string fileName = $"动捕手套_{System.DateTime.Now:yyyyMMdd_HHmmss}.png";
        string fullPath = Path.Combine(desktopPath, fileName);

        ScreenCapture.CaptureScreenshot(fullPath);
        Debug.Log($"截图已保存到桌面: {fileName}");
    }

    // 一键保存当前视角到当前模式
    private void SaveCurrentPreset()
    {
        CameraPreset targetPreset = null;
        string presetName = "";

        switch (currentMode)
        {
            case CameraMode.FistCloseUp:
                targetPreset = fistPreset;
                presetName = "握拳特写";
                break;
            case CameraMode.ThumbCloseUp:
                targetPreset = thumbPreset;
                presetName = "大拇指特写1";
                break;
            case CameraMode.IndexCloseUp:
                targetPreset = indexPreset;
                presetName = "食指特写1";
                break;
            case CameraMode.MiddleCloseUp:
                targetPreset = middlePreset;
                presetName = "中指特写1";
                break;
            case CameraMode.RingCloseUp:
                targetPreset = ringPreset;
                presetName = "无名指特写1";
                break;
            case CameraMode.LittleCloseUp:
                targetPreset = littlePreset;
                presetName = "小指特写1";
                break;
            case CameraMode.ThumbCloseUp2:
                targetPreset = thumbPreset2;
                presetName = "大拇指特写2";
                break;
            case CameraMode.IndexCloseUp2:
                targetPreset = indexPreset2;
                presetName = "食指特写2";
                break;
            case CameraMode.MiddleCloseUp2:
                targetPreset = middlePreset2;
                presetName = "中指特写2";
                break;
            case CameraMode.RingCloseUp2:
                targetPreset = ringPreset2;
                presetName = "无名指特写2";
                break;
            case CameraMode.LittleCloseUp2:
                targetPreset = littlePreset2;
                presetName = "小指特写2";
                break;
            default:
                Debug.LogWarning("当前模式不支持保存预设");
                return;
        }

        targetPreset.position = mainCamera.transform.position;
        targetPreset.rotation = mainCamera.transform.rotation;
        targetPreset.fov = mainCamera.fieldOfView;

        Debug.Log($"✅ 已保存当前视角为【{presetName}】");
        Debug.Log("⚠️ 请退出运行模式，数值会自动保存到Inspector中");
    }
}