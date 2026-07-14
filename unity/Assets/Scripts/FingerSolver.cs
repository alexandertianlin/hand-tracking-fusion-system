using UnityEngine;

[System.Serializable]
public class FingerSolver
{
    [HideInInspector] public string fingerName;
    [System.NonSerialized] public int deviceId;
    [HideInInspector] public bool isThumb;

    [Header("指节骨骼")]
    public Transform rootBone;
    public Transform midBone;
    public Transform tipBone;

    [Header("四指弯曲比例 (大拇指该项失效)")]
    [Range(-5f, 5f)] public float rootWeight;
    [Range(-5f, 5f)] public float midWeight;
    [Range(-5f, 5f)] public float tipWeight;

    [Header("增量灵敏度")]
    [Tooltip("IMU旋转增量缩放系数。1=原灵敏度，0.5=半速。")]
    [Range(0f, 2f)] public float sensitivity = 1.0f;

    [Header("大拇指专属：生物基准")]
    public float thumbBaseYawOffset = -45f;
    [Range(0f, 1f)] public float oppositionTwistRatio = 0.6f;

    [Header("🔧 运行自由调姿窗口")]
    public bool PoseEditMode = false;
    [Range(-180, 180)] public float AdjRootX, AdjRootY, AdjRootZ;
    [Range(-180, 180)] public float AdjMidX, AdjMidY, AdjMidZ;
    [Range(-180, 180)] public float AdjTipX, AdjTipY, AdjTipZ;

    [Header("✅ 握拳边界姿态")]
    public Vector3 rootFistOffset;
    public Vector3 midFistOffset;
    public Vector3 tipFistOffset;
    public float fistFullPitchThreshold = 80f;

    [Header("模型关节限制")]
    [Range(-30f, 0f)] public float minPitch;
    [Range(0f, 160f)] public float maxPitch = 80f;
    [Range(0f, 100f)] public float maxYaw = 35f;

    // ====================== 力可视化新增 【第1处：类顶部添加字段】 ======================
    [Header("三维力网格")]
    public ForceGridVisualizer forceGridPrefab; // 拖拽预制体到这里
    private ForceGridVisualizer _forceGrid; // 网格实例

    // 增量累积状态（模型当前的弯曲角度）
    public float currentPitch = 0f;
    public float currentYaw = 0f;

    // 上一帧的硬件角度，用于计算增量
    private float lastRawPitch = 0f;
    private float lastRawYaw = 0f;
    // 在类的顶部添加
    public float CurrentPitch => currentPitch;
    public float CurrentYaw => currentYaw;
    public float MaxPitch => maxPitch;
    public float MaxYaw => maxYaw;

    private Quaternion driftBias = Quaternion.identity;
    private Quaternion initialRoot;
    private Quaternion initialMid;
    private Quaternion initialTip;

    public FingerSolver(
        string name,
        int id,
        bool thumb,
        float rw, float mw, float tw,
        float minP, float maxP, float maxY)
    {
        fingerName = name;
        deviceId = id;
        isThumb = thumb;
        rootWeight = rw;
        midWeight = mw;
        tipWeight = tw;
        minPitch = minP;
        maxPitch = maxP;
        maxYaw = maxY;
    }

    //private ForceGridVisualizer _forceGrid;

    public void Init()
    {
        if (rootBone) initialRoot = rootBone.localRotation;
        if (midBone) initialMid = midBone.localRotation;
        if (tipBone) initialTip = tipBone.localRotation;

        // 🔥 强制创建网格，不依赖预制体
        if (tipBone != null)
        {
            // 在指尖骨骼下创建一个空物体
            GameObject gridObj = new GameObject("ForceGrid");
            gridObj.transform.SetParent(tipBone, false);
            gridObj.transform.localPosition = Vector3.zero;
            gridObj.transform.localRotation = Quaternion.identity;

            // 挂上脚本
            _forceGrid = gridObj.AddComponent<ForceGridVisualizer>();
            Debug.LogError($"【调试】{fingerName}：强制创建网格成功！");
        }
        else
        {
            Debug.LogError($"【调试】{fingerName}：tipBone为空，无法创建网格！");
        }
        

    // ✅ 运行时自动加载姿态模板
    // 1. 先临时进入编辑模式
    bool originalEditMode = PoseEditMode;
        PoseEditMode = true;

        // 2. 把你在Inspector里调好的Adj值，应用到骨骼上
        rootFistOffset = new Vector3(AdjRootX, AdjRootY, AdjRootZ);
        midFistOffset = new Vector3(AdjMidX, AdjMidY, AdjMidZ);
        tipFistOffset = new Vector3(AdjTipX, AdjTipY, AdjTipZ);

        rootBone.localRotation = initialRoot * Quaternion.Euler(rootFistOffset);
        midBone.localRotation = initialMid * Quaternion.Euler(midFistOffset);
        tipBone.localRotation = initialTip * Quaternion.Euler(tipFistOffset);

        // 3. 自动退出编辑模式，恢复原来的状态
        PoseEditMode = originalEditMode;

        currentPitch = 0f;
        currentYaw = 0f;
    }

    public void CalibrateZero(Quaternion currentRelRot)
    {
        driftBias = currentRelRot;
        currentPitch = 0f;
        currentYaw = 0f;
        lastRawPitch = 0f;
        lastRawYaw = 0f;
    }

    public void SolveAndApplySOTA(
        Quaternion relativeImuRot,
        Vector3 bendAxis,
        Vector3 spreadAxis)
    {
        if (!rootBone || !midBone || !tipBone) return;

        if (PoseEditMode)
        {
            rootFistOffset = new Vector3(AdjRootX, AdjRootY, AdjRootZ);
            midFistOffset = new Vector3(AdjMidX, AdjMidY, AdjMidZ);
            tipFistOffset = new Vector3(AdjTipX, AdjTipY, AdjTipZ);

            rootBone.localRotation = initialRoot * Quaternion.Euler(rootFistOffset);
            midBone.localRotation = initialMid * Quaternion.Euler(midFistOffset);
            tipBone.localRotation = initialTip * Quaternion.Euler(tipFistOffset);
            return;
        }

        Quaternion compensatedRot = Quaternion.Inverse(driftBias) * relativeImuRot;

        if (isThumb)
            SolveThumbIncremental(compensatedRot, bendAxis, spreadAxis);
        else
            SolveFingerIncremental(compensatedRot, bendAxis, spreadAxis);
    }

    public bool ApplyVisionAngleCorrection(
        float targetPitch,
        float targetYaw,
        float blendAlpha,
        float triggerPitchError,
        float triggerYawError,
        Vector3 bendAxis,
        Vector3 spreadAxis)
    {
        if (!rootBone || !midBone || !tipBone || PoseEditMode) return false;

        targetPitch = Mathf.Clamp(targetPitch, minPitch, maxPitch);
        targetYaw = Mathf.Clamp(targetYaw, -maxYaw, maxYaw);

        float pitchError = Mathf.Abs(Mathf.DeltaAngle(currentPitch, targetPitch));
        float yawError = Mathf.Abs(Mathf.DeltaAngle(currentYaw, targetYaw));
        if (pitchError < triggerPitchError && yawError < triggerYawError) return false;

        blendAlpha = Mathf.Clamp01(blendAlpha);
        currentPitch = Mathf.LerpAngle(currentPitch, targetPitch, blendAlpha);
        currentYaw = Mathf.LerpAngle(currentYaw, targetYaw, blendAlpha);
        currentPitch = Mathf.Clamp(currentPitch, minPitch, maxPitch);
        currentYaw = Mathf.Clamp(currentYaw, -maxYaw, maxYaw);

        if (isThumb)
            ApplyThumbPose(bendAxis, spreadAxis);
        else
            ApplyFingerPose(bendAxis, spreadAxis);

        return true;
    }

    public void ForceVisionAngleAnchor(
        float targetPitch,
        float targetYaw,
        Vector3 bendAxis,
        Vector3 spreadAxis)
    {
        if (!rootBone || !midBone || !tipBone || PoseEditMode) return;

        currentPitch = Mathf.Clamp(targetPitch, minPitch, maxPitch);
        currentYaw = Mathf.Clamp(targetYaw, -maxYaw, maxYaw);

        if (isThumb)
            ApplyThumbPose(bendAxis, spreadAxis);
        else
            ApplyFingerPose(bendAxis, spreadAxis);
    }

    public void ForceVisionAngleAnchorAndSyncImu(
        float targetPitch,
        float targetYaw,
        Quaternion relativeImuRot,
        Vector3 bendAxis,
        Vector3 spreadAxis)
    {
        if (!rootBone || !midBone || !tipBone || PoseEditMode) return;

        Quaternion compensatedRot = Quaternion.Inverse(driftBias) * relativeImuRot;
        if (isThumb)
            ExtractThumbAngles(compensatedRot, bendAxis, spreadAxis, out lastRawPitch, out lastRawYaw);
        else
            ExtractFingerAngles(compensatedRot, bendAxis, spreadAxis, out lastRawPitch, out lastRawYaw);

        ForceVisionAngleAnchor(targetPitch, targetYaw, bendAxis, spreadAxis);
    }

    public void MoveVisionAngleAnchorAndSyncImu(
        float targetPitch,
        float targetYaw,
        float blendAlpha,
        Quaternion relativeImuRot,
        Vector3 bendAxis,
        Vector3 spreadAxis)
    {
        if (!rootBone || !midBone || !tipBone || PoseEditMode) return;

        Quaternion compensatedRot = Quaternion.Inverse(driftBias) * relativeImuRot;
        if (isThumb)
            ExtractThumbAngles(compensatedRot, bendAxis, spreadAxis, out lastRawPitch, out lastRawYaw);
        else
            ExtractFingerAngles(compensatedRot, bendAxis, spreadAxis, out lastRawPitch, out lastRawYaw);

        targetPitch = Mathf.Clamp(targetPitch, minPitch, maxPitch);
        targetYaw = Mathf.Clamp(targetYaw, -maxYaw, maxYaw);
        blendAlpha = Mathf.Clamp01(blendAlpha);

        currentPitch = Mathf.LerpAngle(currentPitch, targetPitch, blendAlpha);
        currentYaw = Mathf.LerpAngle(currentYaw, targetYaw, blendAlpha);
        currentPitch = Mathf.Clamp(currentPitch, minPitch, maxPitch);
        currentYaw = Mathf.Clamp(currentYaw, -maxYaw, maxYaw);

        if (isThumb)
            ApplyThumbPose(bendAxis, spreadAxis);
        else
            ApplyFingerPose(bendAxis, spreadAxis);
    }

    // ====================== 力可视化新增 【第3处：添加公共方法，供外部调用】 ======================
    /// <summary>
    // 修正后的 UpdateForceGrid 方法
    public void UpdateForceGrid(Vector3 force)
    {
        Debug.LogError($"【调试】{fingerName}：调用 UpdateForceGrid！力={force}");

        if (_forceGrid != null)
        {
            _forceGrid.UpdateForce(force);
        }
        else
        {
            Debug.LogError($"【调试】{fingerName}：_forceGrid 为空！无法显示网格");
        }
    }

    private void SolveThumbIncremental(Quaternion compensatedRot, Vector3 bendAxis, Vector3 spreadAxis)
    {
        // 提取当前硬件绝对pitch/yaw
        float rawPitch, rawYaw;
        ExtractThumbAngles(compensatedRot, bendAxis, spreadAxis, out rawPitch, out rawYaw);

        // 计算增量（与上一帧的差值）
        float deltaPitch = rawPitch - lastRawPitch;
        float deltaYaw = rawYaw - lastRawYaw;

        // 处理角度环绕（-180~180 跨变）
        if (deltaPitch > 180f) deltaPitch -= 360f;
        if (deltaPitch < -180f) deltaPitch += 360f;
        if (deltaYaw > 180f) deltaYaw -= 360f;
        if (deltaYaw < -180f) deltaYaw += 360f;

        // 应用灵敏度缩放
        deltaPitch *= sensitivity;
        deltaYaw *= sensitivity;

        // 累积到模型当前状态
        currentPitch += deltaPitch;
        currentYaw += deltaYaw;

        // 模型本地关节限制
        currentPitch = Mathf.Clamp(currentPitch, minPitch, maxPitch);
        currentYaw = Mathf.Clamp(currentYaw, -maxYaw, maxYaw);

        // 更新上一帧记录
        lastRawPitch = rawPitch;
        lastRawYaw = rawYaw;

        // ---- 使用 currentPitch/currentYaw 驱动拇指姿态（与原版逻辑一致） ----
        ApplyThumbPose(bendAxis, spreadAxis);
    }

    private void ApplyThumbPose(Vector3 bendAxis, Vector3 spreadAxis)
    {
        float clampedPitch = currentPitch;
        float clampedYaw = currentYaw;

        // 自动外展基于 clampedPitch 进度
        float thumbBendProgress = Mathf.Clamp01(clampedPitch / maxPitch);
        float autoAbduction = thumbBendProgress * maxYaw * 0.6f;
        float realYaw = clampedYaw + autoAbduction;
        float finalYaw = Mathf.Clamp(realYaw, -maxYaw, maxYaw);

        // 关节弯曲角度
        float ipBendAngle = clampedPitch * 1.8f;
        float mcpBendAngle = clampedPitch * 1.8f;
        float cmcBendAngle = clampedPitch * 0.1f;
        float dynamicTwistAngle = clampedPitch * oppositionTwistRatio * 0.3f;

        // 轴向计算
        Quaternion baseOffset = Quaternion.AngleAxis(thumbBaseYawOffset, spreadAxis);
        Vector3 thumbTrueBendAxis = baseOffset * Vector3.up;
        Vector3 thumbTrueSpreadAxis = baseOffset * spreadAxis;
        Vector3 boneLongAxis = Vector3.Cross(thumbTrueSpreadAxis, thumbTrueBendAxis).normalized;
        if (boneLongAxis == Vector3.zero) boneLongAxis = Vector3.forward;

        Vector3 locSpread = Quaternion.Inverse(initialRoot) * thumbTrueSpreadAxis;
        Vector3 locTwist = Quaternion.Inverse(initialRoot) * boneLongAxis;
        Vector3 locBendRoot = Quaternion.Inverse(initialRoot) * thumbTrueBendAxis;

        Quaternion rootBaseRotation = initialRoot
            * Quaternion.AngleAxis(finalYaw, locSpread)
            * Quaternion.AngleAxis(dynamicTwistAngle, locTwist);

        Quaternion rootFullRotation = rootBaseRotation
            * Quaternion.AngleAxis(cmcBendAngle, locBendRoot);
        rootBone.localRotation = rootFullRotation;

        Vector3 fixedLocalBendAxis = locBendRoot;

        // 握拳边界叠加
        float safeT = Mathf.Max(1f, fistFullPitchThreshold);
        float bendProgress = Mathf.Clamp01(clampedPitch / safeT);
        Quaternion rootAdd = Quaternion.SlerpUnclamped(Quaternion.identity, Quaternion.Euler(rootFistOffset), bendProgress);
        Quaternion midAdd = Quaternion.SlerpUnclamped(Quaternion.identity, Quaternion.Euler(midFistOffset), bendProgress);
        Quaternion tipAdd = Quaternion.SlerpUnclamped(Quaternion.identity, Quaternion.Euler(tipFistOffset), bendProgress);

        midBone.localRotation = initialMid
            * Quaternion.AngleAxis(mcpBendAngle, fixedLocalBendAxis)
            * midAdd;
        tipBone.localRotation = initialTip
            * Quaternion.AngleAxis(ipBendAngle, fixedLocalBendAxis)
            * tipAdd;
    }

    private void SolveFingerIncremental(Quaternion compensatedRot, Vector3 bendAxis, Vector3 spreadAxis)
    {
        float rawPitch, rawYaw;
        ExtractFingerAngles(compensatedRot, bendAxis, spreadAxis, out rawPitch, out rawYaw);

        float deltaPitch = rawPitch - lastRawPitch;
        float deltaYaw = rawYaw - lastRawYaw;

        if (deltaPitch > 180f) deltaPitch -= 360f;
        if (deltaPitch < -180f) deltaPitch += 360f;
        if (deltaYaw > 180f) deltaYaw -= 360f;
        if (deltaYaw < -180f) deltaYaw += 360f;

        deltaPitch *= sensitivity;
        deltaYaw *= sensitivity;

        currentPitch += deltaPitch;
        currentYaw += deltaYaw;

        currentPitch = Mathf.Clamp(currentPitch, minPitch, maxPitch);
        currentYaw = Mathf.Clamp(currentYaw, -maxYaw, maxYaw);

        lastRawPitch = rawPitch;
        lastRawYaw = rawYaw;

        ApplyFingerPose(bendAxis, spreadAxis);
    }

    private void ApplyFingerPose(Vector3 bendAxis, Vector3 spreadAxis)
    {
        float clampedPitch = currentPitch;
        float clampedYaw = currentYaw;

        float rootAngle = clampedPitch * rootWeight;
        float midAngle = clampedPitch * midWeight;
        float tipAngle = clampedPitch * tipWeight;

        // 四指 spread axis 计算
        Vector3 trueSpreadAxis = Vector3.Cross(bendAxis, spreadAxis).normalized;
        if (trueSpreadAxis == Vector3.zero) trueSpreadAxis = Vector3.forward;

        float safeT = Mathf.Max(1f, fistFullPitchThreshold);
        float bendProgress = Mathf.Clamp01(clampedPitch / safeT);
        Quaternion rootBlend = Quaternion.SlerpUnclamped(Quaternion.identity, Quaternion.Euler(rootFistOffset), bendProgress);
        Quaternion midBlend = Quaternion.SlerpUnclamped(Quaternion.identity, Quaternion.Euler(midFistOffset), bendProgress);
        Quaternion tipBlend = Quaternion.SlerpUnclamped(Quaternion.identity, Quaternion.Euler(tipFistOffset), bendProgress);

        rootBone.localRotation = initialRoot
            * Quaternion.AngleAxis(clampedYaw, trueSpreadAxis)
            * Quaternion.AngleAxis(rootAngle, bendAxis)
            * rootBlend;
        midBone.localRotation = initialMid
            * Quaternion.AngleAxis(midAngle, bendAxis)
            * midBlend;
        tipBone.localRotation = initialTip
            * Quaternion.AngleAxis(tipAngle, bendAxis)
            * tipBlend;
    }

    // 提取拇指原始角度（与原版完全一致）
    private void ExtractThumbAngles(Quaternion compensatedRot, Vector3 bendAxis, Vector3 spreadAxis, out float pitch, out float yaw)
    {
        Quaternion baseOffset = Quaternion.AngleAxis(thumbBaseYawOffset, spreadAxis);
        Vector3 thumbTrueBendAxis = baseOffset * Vector3.up;
        Vector3 thumbTrueSpreadAxis = baseOffset * spreadAxis;
        Vector3 boneLongAxis = Vector3.Cross(thumbTrueSpreadAxis, thumbTrueBendAxis).normalized;
        if (boneLongAxis == Vector3.zero) boneLongAxis = Vector3.forward;

        Vector3 currentBoneDir = compensatedRot * boneLongAxis;

        Vector3 pitchProjected = Vector3.ProjectOnPlane(currentBoneDir, thumbTrueBendAxis);
        if (pitchProjected == Vector3.zero) pitchProjected = boneLongAxis;
        pitch = Vector3.SignedAngle(boneLongAxis, pitchProjected, thumbTrueBendAxis);

        Vector3 yawProjected = Vector3.ProjectOnPlane(currentBoneDir, thumbTrueSpreadAxis);
        if (yawProjected == Vector3.zero) yawProjected = boneLongAxis;
        yaw = Vector3.SignedAngle(boneLongAxis, yawProjected, thumbTrueSpreadAxis);
    }

    // 提取四指原始角度（与原版完全一致）
    private void ExtractFingerAngles(Quaternion compensatedRot, Vector3 bendAxis, Vector3 spreadAxis, out float pitch, out float yaw)
    {
        Vector3 boneLongAxis = spreadAxis;
        Vector3 trueSpreadAxis = Vector3.Cross(bendAxis, boneLongAxis).normalized;
        if (trueSpreadAxis == Vector3.zero) trueSpreadAxis = Vector3.forward;

        Vector3 currentBoneDir = compensatedRot * boneLongAxis;

        Vector3 pitchProjected = Vector3.ProjectOnPlane(currentBoneDir, bendAxis);
        if (pitchProjected == Vector3.zero) pitchProjected = boneLongAxis;
        pitch = Vector3.SignedAngle(boneLongAxis, pitchProjected, bendAxis);

        Vector3 yawProjected = Vector3.ProjectOnPlane(currentBoneDir, trueSpreadAxis);
        if (yawProjected == Vector3.zero) yawProjected = boneLongAxis;
        yaw = Vector3.SignedAngle(boneLongAxis, yawProjected, trueSpreadAxis);
    }
}
