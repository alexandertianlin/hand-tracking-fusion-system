using UnityEngine;
using System.Text;
using System.Collections.Generic;

public enum AxisMap { X, Y, Z, NegativeX, NegativeY, NegativeZ }

[RequireComponent(typeof(SerialReceiver))]
public class HandMotionManager : MonoBehaviour
{
    // 在类的顶部添加
    public bool IsCalibrated => isCalibrated;
    private SerialReceiver receiver;

    [Header("硬件与手掌配置")]
    public Transform wristBone;
    public Transform palmBone;
    [System.NonSerialized] public int palmId = 0x30;

    [Header("IMU -> Unity 坐标系映射")]
    public AxisMap imuXToUnity = AxisMap.X;
    public AxisMap imuYToUnity = AxisMap.Y;
    public AxisMap imuZToUnity = AxisMap.Z;

    // 🔥 【新增：大拇指专属IMU坐标系映射，和图里结构完全一致】
    [Header("👍 大拇指 IMU -> Unity 坐标系映射")]
    public AxisMap thumbImuXToUnity = AxisMap.X;
    public AxisMap thumbImuYToUnity = AxisMap.Y;
    public AxisMap thumbImuZToUnity = AxisMap.Z;

    [Header("统一手指关节轴向")]
    public Vector3 fingerBendAxis = Vector3.right;
    public Vector3 fingerSpreadAxis = Vector3.up;

    [Header("🔥 手掌防麻花限制 (Swing-Twist 分解)")]
    [Tooltip("手腕的扭转基准长轴")]
    public Vector3 palmTwistAxis = Vector3.forward;
    [Range(0f, 180f)] public float palmMaxSwingAngle = 70f;
    [Range(0f, 180f)] public float palmMaxTwistAngle = 60f;

    [Header("交互配置")]
    public KeyCode calibrateKey = KeyCode.Space;

    [Header("调试信息输出")]
    public bool printDebugLog = true;
    [Range(0.1f, 2f)] public float printInterval = 0.5f;
    private float printTimer = 0f;

    [Header("--- 手指骨骼绑定 ---")]
    public FingerSolver thumb = new FingerSolver("Thumb", 0x1E, true, 0.5f, 0.3f, 0.2f, -15f, 80f, 35f);
    public FingerSolver index = new FingerSolver("Index", 0x28, false, 0.25f, 0.45f, 0.3f, -10f, 95f, 20f);
    public FingerSolver middle = new FingerSolver("Middle", 0x32, false, 0.2f, 0.5f, 0.3f, -10f, 100f, 15f);
    public FingerSolver ring = new FingerSolver("Ring", 0x3C, false, 0.2f, 0.5f, 0.3f, -10f, 100f, 15f);
    public FingerSolver little = new FingerSolver("Little", 0x46, false, 0.25f, 0.45f, 0.3f, -15f, 90f, 25f);

    private Quaternion palmCalibration = Quaternion.identity;
    private Quaternion palmDriftBias = Quaternion.identity;
    private Quaternion initialPalmLocalRot;
    public bool isCalibrated = false;

    private FingerSolver[] fingersCache;

    void Start()
    {
        receiver = GetComponent<SerialReceiver>();
        if (palmBone) initialPalmLocalRot = palmBone.localRotation;

        fingersCache = new[] { thumb, index, middle, ring, little };
        foreach (var f in fingersCache) f.Init();

        // ✅ 运行时自动关闭所有手指的姿态编辑模式
        // 保留你在Inspector里调好的AdjRoot/AdjMid/AdjTip数值
        foreach (var f in fingersCache)
        {
            f.PoseEditMode = false;
        }

        Debug.Log("✅ 运行时已自动关闭所有手指的姿态编辑模式");
    }

    void Update()
    {
        if (Input.GetKeyDown(calibrateKey)) Calibrate();

        if (printDebugLog)
        {
            printTimer += Time.deltaTime;
            if (printTimer >= printInterval)
            {
                printTimer = 0f;
                PrintHardwareStatusToConsole();
            }
        }

        if (!isCalibrated) return;

        if (receiver.ImuDataDict.TryGetValue(palmId, out Quaternion rawPalmQ))
        {
            Quaternion alignedPalmQ = MapCoordinates(rawPalmQ);
            Quaternion relativePalmRot = Quaternion.Inverse(palmCalibration) * alignedPalmQ;
            Quaternion compensatedPalmRot = Quaternion.Inverse(palmDriftBias) * relativePalmRot;

            DecomposeSwingTwist(compensatedPalmRot, palmTwistAxis, out Quaternion swing, out Quaternion twist);

            twist.ToAngleAxis(out float twistAngle, out Vector3 tAxis);
            if (twistAngle > 180f) twistAngle -= 360f;
            twistAngle = Mathf.Clamp(twistAngle, -palmMaxTwistAngle, palmMaxTwistAngle);
            Quaternion clampedTwist = Quaternion.AngleAxis(twistAngle, tAxis);

            swing.ToAngleAxis(out float swingAngle, out Vector3 sAxis);
            if (swingAngle > 180f) swingAngle -= 360f;
            swingAngle = Mathf.Clamp(swingAngle, -palmMaxSwingAngle, palmMaxSwingAngle);
            Quaternion clampedSwing = Quaternion.AngleAxis(swingAngle, sAxis);

            Quaternion clampedPalmRot = clampedSwing * clampedTwist;

            if (Quaternion.Angle(compensatedPalmRot, clampedPalmRot) > 1.0f)
            {
                Quaternion targetBias = relativePalmRot * Quaternion.Inverse(clampedPalmRot);
                float blendAlpha = 1f - Mathf.Exp(-8f * Time.deltaTime);
                palmDriftBias = Quaternion.Slerp(palmDriftBias, targetBias, blendAlpha);
            }

            if (palmBone) palmBone.localRotation = initialPalmLocalRot * clampedPalmRot;

            // 【仅此处分流：大拇指用独立映射，四指用全局映射】
            foreach (var finger in fingersCache)
            {
                if (receiver.ImuDataDict.TryGetValue(finger.deviceId, out Quaternion rawFingerQ))
                {
                    Quaternion alignedFingerQ;
                    if (finger.isThumb)
                        alignedFingerQ = MapThumbCoordinates(rawFingerQ);
                    else
                        alignedFingerQ = MapCoordinates(rawFingerQ);

                    Quaternion relativeFingerRot = Quaternion.Inverse(alignedPalmQ) * alignedFingerQ;
                    finger.SolveAndApplySOTA(relativeFingerRot, fingerBendAxis, fingerSpreadAxis);
                }
            }
            foreach (var finger in fingersCache)
            {
                if (receiver.ForceDataDict.TryGetValue(finger.deviceId, out Vector3 force))
                {
                    // 调用FingerSolver里新增的方法
                    finger.UpdateForceGrid(force);
                }
            }
        }
    }

    private void DecomposeSwingTwist(Quaternion q, Vector3 twistAxis, out Quaternion swing, out Quaternion twist)
    {
        Vector3 rotationAxis = new Vector3(q.x, q.y, q.z);
        Vector3 projection = Vector3.Project(rotationAxis, twistAxis);
        twist = new Quaternion(projection.x, projection.y, projection.z, q.w).normalized;
        swing = q * Quaternion.Inverse(twist);
    }

    [ContextMenu("一键标定校准 (Calibrate)")]
    public void Calibrate()
    {
        if (receiver.ImuDataDict.TryGetValue(palmId, out Quaternion rawPalmQ))
        {
            palmCalibration = MapCoordinates(rawPalmQ);
            palmDriftBias = Quaternion.identity;
        }

        foreach (var finger in fingersCache)
        {
            if (receiver.ImuDataDict.TryGetValue(finger.deviceId, out Quaternion rawFingerQ))
            {
                // 校准时也分流：大拇指用自己的映射
                Quaternion alignedFingerQ = finger.isThumb ? MapThumbCoordinates(rawFingerQ) : MapCoordinates(rawFingerQ);
                Quaternion relativeFingerRot = Quaternion.Inverse(MapCoordinates(rawPalmQ)) * alignedFingerQ;
                finger.CalibrateZero(relativeFingerRot);
            }
        }
        isCalibrated = true;
        Debug.Log("<color=#00FF00>✅ 动捕手套校准完成！</color>");
    }

    private void PrintHardwareStatusToConsole()
    {
        StringBuilder sb = new StringBuilder();
        sb.AppendLine("<b><color=#00FF00>=== 动捕手套实时状态 ===</color></b>");
        sb.AppendLine(isCalibrated ? "<color=#00FF00>状态: 已校准跟踪中</color>" : "<color=#FF9900>状态: 未校准 (请按空格键)</color>");

        HashSet<int> boundIds = new HashSet<int> { palmId, thumb.deviceId, index.deviceId, middle.deviceId, ring.deviceId, little.deviceId };
        AppendNodeStatus(sb, "手掌 (Palm)", palmId);

        foreach (var f in fingersCache) AppendNodeStatus(sb, f.fingerName, f.deviceId);

        foreach (var kvp in receiver.ImuDataDict)
        {
            if (!boundIds.Contains(kvp.Key))
            {
                receiver.RawHexDict.TryGetValue(kvp.Key, out string hex);
                string shortHex = hex != null && hex.Length > 24 ? hex.Substring(0, 23) + "..." : hex;
                sb.AppendLine($"<color=#FF00FF>[野生数据]</color> 未知节点(0x{kvp.Key:X2}) | 四元数:({kvp.Value.x:F2},{kvp.Value.y:F2},{kvp.Value.z:F2},{kvp.Value.w:F2}) | HEX: {shortHex}");
            }
        }
        Debug.Log(sb.ToString());
    }

    private void AppendNodeStatus(StringBuilder sb, string label, int id)
    {
        if (receiver.ImuDataDict.TryGetValue(id, out Quaternion q))
        {
            receiver.RawHexDict.TryGetValue(id, out string hex);
            string shortHex = hex != null && hex.Length > 24 ? hex.Substring(0, 23) + "..." : hex;
            sb.AppendLine($"<color=#55FFFF>[ON]</color> {label}(0x{id:X2}) | 四元数:({q.x:F2},{q.y:F2},{q.z:F2},{q.w:F2}) | HEX: {shortHex}");
        }
        else sb.AppendLine($"<color=#FF5555>[OFF]</color> {label}(0x{id:X2}) | 未收到报文");
    }

    public bool ApplyVisionAnchorPose(
        bool fist,
        float openPitch,
        float openYaw,
        bool useFingerMaxPitchForFist,
        float fallbackFistPitch,
        float fistYaw,
        float blendAlpha = 1f,
        int fingerIndex = -1)
    {
        if (!isCalibrated) return false;
        if (!receiver.ImuDataDict.TryGetValue(palmId, out Quaternion rawPalmQ)) return false;

        Quaternion alignedPalmQ = MapCoordinates(rawPalmQ);
        bool appliedAny = false;

        for (int i = 0; i < fingersCache.Length; i++)
        {
            if (fingerIndex >= 0 && i != fingerIndex) continue;

            var finger = fingersCache[i];
            if (!receiver.ImuDataDict.TryGetValue(finger.deviceId, out Quaternion rawFingerQ)) continue;

            Quaternion alignedFingerQ = finger.isThumb ? MapThumbCoordinates(rawFingerQ) : MapCoordinates(rawFingerQ);
            Quaternion relativeFingerRot = Quaternion.Inverse(alignedPalmQ) * alignedFingerQ;

            float targetPitch = fist
                ? (useFingerMaxPitchForFist ? finger.MaxPitch : fallbackFistPitch)
                : openPitch;
            float targetYaw = fist ? fistYaw : openYaw;

            if (blendAlpha >= 0.999f)
            {
                finger.ForceVisionAngleAnchorAndSyncImu(
                    targetPitch,
                    targetYaw,
                    relativeFingerRot,
                    fingerBendAxis,
                    fingerSpreadAxis);
            }
            else
            {
                finger.MoveVisionAngleAnchorAndSyncImu(
                    targetPitch,
                    targetYaw,
                    blendAlpha,
                    relativeFingerRot,
                    fingerBendAxis,
                    fingerSpreadAxis);
            }
            appliedAny = true;
        }

        return appliedAny;
    }

    // 【全局映射函数：原有代码100%不动】
    private Quaternion MapCoordinates(Quaternion raw)
    {
        Vector3 iX = GetAxisVector(imuXToUnity);
        Vector3 iY = GetAxisVector(imuYToUnity);
        Vector3 iZ = GetAxisVector(imuZToUnity);

        if (Mathf.Abs(Vector3.Dot(iX, iY)) > 0.1f || Mathf.Abs(Vector3.Dot(iY, iZ)) > 0.1f)
        {
            Debug.LogError("<color=#FF0000>轴映射错误：存在重合轴，请检查 Inspector 设置！</color>");
            return raw;
        }

        Vector3 mappedXYZ = iX * raw.x + iY * raw.y + iZ * raw.z;
        float determinant = Vector3.Dot(Vector3.Cross(iX, iY), iZ);
        float mappedW = determinant < 0f ? -raw.w : raw.w;

        return new Quaternion(mappedXYZ.x, mappedXYZ.y, mappedXYZ.z, mappedW);
    }

    // 【新增：大拇指独立映射函数，和全局逻辑完全一致】
    private Quaternion MapThumbCoordinates(Quaternion raw)
    {
        Vector3 iX = GetAxisVector(thumbImuXToUnity);
        Vector3 iY = GetAxisVector(thumbImuYToUnity);
        Vector3 iZ = GetAxisVector(thumbImuZToUnity);

        if (Mathf.Abs(Vector3.Dot(iX, iY)) > 0.1f || Mathf.Abs(Vector3.Dot(iY, iZ)) > 0.1f)
        {
            Debug.LogError("<color=#FF0000>大拇指轴映射错误：存在重合轴！</color>");
            return raw;
        }

        Vector3 mappedXYZ = iX * raw.x + iY * raw.y + iZ * raw.z;
        float determinant = Vector3.Dot(Vector3.Cross(iX, iY), iZ);
        float mappedW = determinant < 0f ? -raw.w : raw.w;

        return new Quaternion(mappedXYZ.x, mappedXYZ.y, mappedXYZ.z, mappedW);
    }

    private Vector3 GetAxisVector(AxisMap map)
    {
        return map switch
        {
            AxisMap.X => Vector3.right,
            AxisMap.Y => Vector3.up,
            AxisMap.Z => Vector3.forward,
            AxisMap.NegativeX => Vector3.left,
            AxisMap.NegativeY => Vector3.down,
            AxisMap.NegativeZ => Vector3.back,
            _ => Vector3.zero
        };
    }
}
