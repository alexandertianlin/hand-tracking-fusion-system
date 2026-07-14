using UnityEngine;

[RequireComponent(typeof(HandMotionManager))]
public class VisionOpenPalmRefreshModule : MonoBehaviour
{
    [Header("Open palm refresh")]
    public bool enableOpenPalmRefresh = true;
    public bool requirePalmFacing = true;
    public float refreshCooldownSeconds = 3.0f;
    public bool printRefreshLog = true;

    private HandMotionManager handMotion;
    private float lastRefreshTime = -999f;
    private int lastSequenceId = -1;

    private void Awake()
    {
        handMotion = GetComponent<HandMotionManager>();
    }

    public bool TryRefreshFromOpenPalm(int sequenceId, bool isPalmFacing)
    {
        if (!enableOpenPalmRefresh || handMotion == null || !handMotion.IsCalibrated) return false;
        if (sequenceId == lastSequenceId) return false;
        if (requirePalmFacing && !isPalmFacing)
        {
            if (printRefreshLog)
                Debug.Log($"Vision open-palm refresh rejected: palm is not facing camera, seq={sequenceId}");
            return false;
        }
        if (Time.time - lastRefreshTime < refreshCooldownSeconds) return false;

        handMotion.Calibrate();
        lastRefreshTime = Time.time;
        lastSequenceId = sequenceId;

        if (printRefreshLog)
            Debug.Log($"Vision open-palm refresh: recalibrated IMU baseline, seq={sequenceId}, palmFacing={isPalmFacing}");

        return true;
    }
}
