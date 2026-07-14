using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

[RequireComponent(typeof(HandMotionManager))]
public class VisionFingerCorrectionReceiver : MonoBehaviour
{
    [Header("Vision anchor switch")]
    public bool enableVisionCorrection = false;
    public int listenPort = 5055;

    [Header("Anchor validation")]
    [Range(0f, 1f)] public float minConfidence = 0.70f;
    [Range(0f, 1f)] public float visualConfidenceThreshold = 0.75f;
    public int minStableMs = 0;
    public float commandCooldownSeconds = 0.05f;
    public float visionTimeoutSeconds = 0.6f;

    [Header("Visual takeover")]
    public float anchorDriveSeconds = 0.25f;
    [Tooltip("Minimum per-finger vision lock duration. IDLE packets for the same finger are ignored during this window.")]
    [Range(0.1f, 2f)] public float fingerVisualHoldSeconds = 0.8f;
    [Tooltip("Higher means faster movement toward open/fist during visual takeover.")]
    [Range(0.1f, 30f)] public float anchorMoveSpeed = 18f;

    [Header("Open hand anchor")]
    public float openPitch = 0f;
    public float openYaw = 0f;

    [Header("Fist anchor")]
    [Tooltip("Use each finger's maxPitch as the fist target.")]
    public bool useFingerMaxPitchForFist = true;
    public float fallbackFistPitch = 60f;
    public float fistYaw = 0f;

    [Header("Debug")]
    public bool printAnchorLog = true;
    public bool printPacketDiagnostics = true;
    public bool writeDiagnosticFileLog = true;
    public bool clearDiagnosticLogOnStart = true;
    public bool logIdlePackets = false;
    public bool logDriveFrames = true;
    public float driveDiagnosticInterval = 0.25f;
    public string diagnosticLogFileName = "vision_finger_diagnostic.log";

    private const string CommandOpen = "TRIGGER_OPEN";
    private const string CommandFist = "TRIGGER_FIST";
    private const string CommandFingerOpen = "FINGER_OPEN";
    private const string CommandFingerFist = "FINGER_FIST";
    private const string CommandIdle = "IDLE";
    private const int AllFingers = -1;

    private HandMotionManager handMotion;
    private VisionOpenPalmRefreshModule openPalmRefreshModule;
    private UdpClient udpClient;
    private Thread receiveThread;
    private volatile bool running;
    private readonly object packetLock = new object();
    private readonly Queue<string> packetQueue = new Queue<string>();
    private const int MaxQueuedPackets = 64;
    private float lastAnchorTime = -999f;
    private float lastPacketTime = -999f;
    private string activeCommand;
    private float activeAnchorEndTime = -999f;
    private string[] activeFingerCommands;
    private float[] activeFingerEndTimes;
    private float[] lastFingerAnchorTimes;
    private float[] fingerHoldEndTimes;
    private string diagnosticLogPath;
    private readonly object diagnosticLogLock = new object();
    private float nextDriveDiagnosticTime = -999f;

    [Serializable]
    private class VisionPacket
    {
        public long timestampMs;
        public int sequenceId;
        public float confidence;
        public int stableMs;
        public int fingerIndex = -1;
        public string fingerName;
        public string command;
        public string gestureState;
        public float score;
        public float vis_conf;
        public bool isPalmFacing;
    }

    private void Awake()
    {
        handMotion = GetComponent<HandMotionManager>();
        openPalmRefreshModule = GetComponent<VisionOpenPalmRefreshModule>();
        activeFingerCommands = new string[5];
        activeFingerEndTimes = new float[5];
        lastFingerAnchorTimes = new float[5];
        fingerHoldEndTimes = new float[5];
        for (int i = 0; i < lastFingerAnchorTimes.Length; i++)
        {
            lastFingerAnchorTimes[i] = -999f;
            fingerHoldEndTimes[i] = -999f;
        }

        InitializeDiagnosticLog();
    }

    private void OnDisable()
    {
        StopReceiver();
    }

    private void LateUpdate()
    {
        if (!enableVisionCorrection)
        {
            if (udpClient != null) StopReceiver();
            return;
        }

        if (udpClient == null) StartReceiver();
        if (handMotion == null || !handMotion.IsCalibrated) return;

        List<string> packets = null;
        lock (packetLock)
        {
            if (packetQueue.Count > 0)
            {
                packets = new List<string>(packetQueue.Count);
                while (packetQueue.Count > 0)
                    packets.Add(packetQueue.Dequeue());
            }
        }

        int processedPackets = 0;
        if (packets != null)
        {
            foreach (string json in packets)
            {
                lastPacketTime = Time.time;
                TryStartVisualTakeover(json);
                processedPackets++;
            }
        }

        if (printPacketDiagnostics && processedPackets > 1)
        {
            Debug.Log($"Vision packets processed this frame: {processedPackets}");
        }
        if (processedPackets > 0)
        {
            WriteDiagnostic("QUEUE_DRAIN", $"processed={processedPackets} remaining=0");
        }

        if (Time.time - lastPacketTime > visionTimeoutSeconds)
        {
            CancelVisualTakeover(AllFingers, "vision timeout");
        }

        DriveActiveAnchor();
    }

    private void TryStartVisualTakeover(string json)
    {
        VisionPacket packet;
        try
        {
            packet = JsonUtility.FromJson<VisionPacket>(json);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"Vision anchor packet parse failed: {e.Message}");
            WriteDiagnostic("PARSE_FAIL", $"error={e.Message} json={TrimForLog(json)}");
            return;
        }

        string command = GetCommand(packet);
        int packetFingerIndex = GetFingerIndex(packet);
        float visualConfidence = GetVisionConfidence(packet);
        if (command != CommandIdle || logIdlePackets)
        {
            WriteDiagnostic(
                "PARSE_OK",
                $"seq={packet.sequenceId} command={command} finger={packetFingerIndex} score={packet.score:F3} conf={packet.confidence:F3} vis_conf={visualConfidence:F3} stableMs={packet.stableMs} palmFacing={packet.isPalmFacing}");
        }
        if (printPacketDiagnostics && command != CommandIdle)
        {
            Debug.Log($"Vision packet seq={packet.sequenceId} command={command} finger={packetFingerIndex} score={packet.score:F2} conf={packet.confidence:F2} vis_conf={visualConfidence:F2} palmFacing={packet.isPalmFacing}");
        }

        if (visualConfidence < visualConfidenceThreshold)
        {
            WriteDiagnostic(
                "REJECT_BY_CONFIDENCE",
                $"seq={packet.sequenceId} command={command} finger={packetFingerIndex} vis_conf={visualConfidence:F3}<threshold:{visualConfidenceThreshold:F3}");
            if (IsFingerHoldActive(packetFingerIndex))
            {
                WriteDiagnostic(
                    "LOW_CONF_IGNORED_BY_HOLD",
                    $"seq={packet.sequenceId} finger={packetFingerIndex} activeCommand={activeFingerCommands[packetFingerIndex]} holdRemaining={fingerHoldEndTimes[packetFingerIndex] - Time.time:F3}");
            }
            return;
        }

        if (command == CommandIdle)
        {
            WriteDiagnostic("IDLE_PACKET", $"seq={packet.sequenceId} finger={packetFingerIndex}");
            if (IsFingerHoldActive(packetFingerIndex))
            {
                WriteDiagnostic(
                    "IDLE_IGNORED_BY_HOLD",
                    $"seq={packet.sequenceId} finger={packetFingerIndex} activeCommand={activeFingerCommands[packetFingerIndex]} holdRemaining={fingerHoldEndTimes[packetFingerIndex] - Time.time:F3}");
                return;
            }
            CancelVisualTakeover(packetFingerIndex, "IDLE");
            return;
        }

        if (!IsAnchorPacketUsable(packet, out string rejectReason))
        {
            WriteDiagnostic("FILTER_REJECT", $"seq={packet.sequenceId} command={command} finger={packetFingerIndex} reason={rejectReason}");
            return;
        }
        int fingerIndex = IsFingerCommand(command) ? packetFingerIndex : AllFingers;
        if (fingerIndex < AllFingers || fingerIndex > 4)
        {
            WriteDiagnostic("FINGER_REJECT", $"seq={packet.sequenceId} command={command} finger={fingerIndex}");
            return;
        }
        if (IsInCooldown(fingerIndex))
        {
            WriteDiagnostic("COOLDOWN_REJECT", $"seq={packet.sequenceId} command={command} finger={fingerIndex}");
            return;
        }

        StartTakeover(command, fingerIndex);
        WriteDiagnostic("TAKEOVER_START", $"seq={packet.sequenceId} command={command} finger={fingerIndex} driveSeconds={GetDriveSecondsForFinger(fingerIndex):F3} moveSpeed={anchorMoveSpeed:F2}");

        if (command == CommandOpen && fingerIndex == AllFingers && openPalmRefreshModule != null)
            openPalmRefreshModule.TryRefreshFromOpenPalm(packet.sequenceId, packet.isPalmFacing);

        if (printAnchorLog)
            Debug.Log($"Vision takeover started: {command}, finger={fingerIndex}, stable={packet.stableMs}ms, conf={packet.confidence:F2}");
    }

    private void DriveActiveAnchor()
    {
        DriveTakeover(activeCommand, AllFingers, activeAnchorEndTime);

        for (int i = 0; i < activeFingerCommands.Length; i++)
            DriveTakeover(activeFingerCommands[i], i, activeFingerEndTimes[i]);
    }

    private void DriveTakeover(string command, int fingerIndex, float endTime)
    {
        if (!IsDriveCommand(command)) return;

        bool isFinalFrame = Time.time >= endTime;
        float blendAlpha = isFinalFrame
            ? 1f
            : 1f - Mathf.Exp(-anchorMoveSpeed * Time.deltaTime);

        bool applied = handMotion.ApplyVisionAnchorPose(
            command == CommandFist || command == CommandFingerFist,
            openPitch,
            openYaw,
            useFingerMaxPitchForFist,
            fallbackFistPitch,
            fistYaw,
            blendAlpha,
            fingerIndex);

        if (logDriveFrames && Time.time >= nextDriveDiagnosticTime)
        {
            WriteDiagnostic("APPLY_RESULT", $"command={command} finger={fingerIndex} applied={applied} blend={blendAlpha:F3} finalFrame={isFinalFrame}");
            nextDriveDiagnosticTime = Time.time + driveDiagnosticInterval;
        }

        if (!applied)
        {
            WriteDiagnostic("APPLY_FAILED", $"command={command} finger={fingerIndex} finalFrame={isFinalFrame}");
            return;
        }

        if (isFinalFrame)
        {
            if (printAnchorLog) Debug.Log($"Vision takeover finished: {command}, finger={fingerIndex}");
            WriteDiagnostic("TAKEOVER_FINISH", $"command={command} finger={fingerIndex}");
            ClearTakeover(fingerIndex);
        }
    }

    private void CancelVisualTakeover(int fingerIndex, string reason)
    {
        if (fingerIndex == AllFingers)
        {
            if (IsDriveCommand(activeCommand) && printAnchorLog)
                Debug.Log($"Vision takeover cancelled: {reason}, finger={AllFingers}");
            if (IsDriveCommand(activeCommand))
                WriteDiagnostic("CANCEL", $"reason={reason} finger={AllFingers} activeCommand={activeCommand}");

            activeCommand = null;
            for (int i = 0; i < activeFingerCommands.Length; i++)
            {
                if (IsDriveCommand(activeFingerCommands[i]))
                    WriteDiagnostic("CANCEL", $"reason={reason} finger={i} activeCommand={activeFingerCommands[i]}");
                activeFingerCommands[i] = null;
                fingerHoldEndTimes[i] = -999f;
            }
            return;
        }

        if (fingerIndex < 0 || fingerIndex >= activeFingerCommands.Length) return;
        if (!IsDriveCommand(activeFingerCommands[fingerIndex])) return;

        if (printAnchorLog) Debug.Log($"Vision takeover cancelled: {reason}, finger={fingerIndex}");
        WriteDiagnostic("CANCEL", $"reason={reason} finger={fingerIndex} activeCommand={activeFingerCommands[fingerIndex]}");
        activeFingerCommands[fingerIndex] = null;
        fingerHoldEndTimes[fingerIndex] = -999f;
    }

    private bool IsAnchorPacketUsable(VisionPacket packet, out string rejectReason)
    {
        rejectReason = null;
        if (packet == null)
        {
            rejectReason = "packet_null";
            return false;
        }

        string command = GetCommand(packet);
        if (!IsDriveCommand(command))
        {
            rejectReason = $"not_drive_command:{command}";
            return false;
        }
        if (packet.confidence < minConfidence)
        {
            rejectReason = $"low_confidence:{packet.confidence:F3}<min:{minConfidence:F3}";
            return false;
        }
        if (packet.stableMs < minStableMs)
        {
            rejectReason = $"unstable:{packet.stableMs}<min:{minStableMs}";
            return false;
        }
        return true;
    }

    private bool IsDriveCommand(string command)
    {
        return command == CommandOpen
            || command == CommandFist
            || command == CommandFingerOpen
            || command == CommandFingerFist;
    }

    private bool IsFingerCommand(string command)
    {
        return command == CommandFingerOpen || command == CommandFingerFist;
    }

    private bool IsInCooldown(int fingerIndex)
    {
        if (fingerIndex == AllFingers)
            return Time.time - lastAnchorTime < commandCooldownSeconds;

        return Time.time - lastFingerAnchorTimes[fingerIndex] < commandCooldownSeconds;
    }

    private void StartTakeover(string command, int fingerIndex)
    {
        if (fingerIndex == AllFingers)
        {
            activeCommand = command;
            activeAnchorEndTime = Time.time + anchorDriveSeconds;
            lastAnchorTime = Time.time;
            return;
        }

        activeFingerCommands[fingerIndex] = command;
        float driveSeconds = GetDriveSecondsForFinger(fingerIndex);
        activeFingerEndTimes[fingerIndex] = Time.time + driveSeconds;
        fingerHoldEndTimes[fingerIndex] = activeFingerEndTimes[fingerIndex];
        lastFingerAnchorTimes[fingerIndex] = Time.time;
    }

    private void ClearTakeover(int fingerIndex)
    {
        if (fingerIndex == AllFingers)
        {
            activeCommand = null;
            return;
        }

        if (fingerIndex >= 0 && fingerIndex < activeFingerCommands.Length)
        {
            activeFingerCommands[fingerIndex] = null;
            fingerHoldEndTimes[fingerIndex] = -999f;
        }
    }

    private float GetDriveSecondsForFinger(int fingerIndex)
    {
        return fingerIndex == AllFingers
            ? anchorDriveSeconds
            : Mathf.Max(anchorDriveSeconds, fingerVisualHoldSeconds);
    }

    private bool IsFingerHoldActive(int fingerIndex)
    {
        if (fingerIndex < 0 || fingerIndex >= fingerHoldEndTimes.Length) return false;
        return IsDriveCommand(activeFingerCommands[fingerIndex]) && Time.time < fingerHoldEndTimes[fingerIndex];
    }

    private int GetFingerIndex(VisionPacket packet)
    {
        if (packet.fingerIndex >= 0 && packet.fingerIndex <= 4) return packet.fingerIndex;

        switch (packet.fingerName)
        {
            case "Thumb": return 0;
            case "Index": return 1;
            case "Middle": return 2;
            case "Ring": return 3;
            case "Little": return 4;
            default: return AllFingers;
        }
    }

    private string GetCommand(VisionPacket packet)
    {
        if (!string.IsNullOrEmpty(packet.command)) return packet.command;
        return packet.gestureState;
    }

    private float GetVisionConfidence(VisionPacket packet)
    {
        if (packet == null) return 0f;
        return packet.vis_conf > 0f ? packet.vis_conf : packet.confidence;
    }

    private void StartReceiver()
    {
        if (udpClient != null) return;

        try
        {
            udpClient = new UdpClient(listenPort);
            udpClient.Client.ReceiveTimeout = 200;
            running = true;
            receiveThread = new Thread(ReceiveLoop) { IsBackground = true };
            receiveThread.Start();
            Debug.Log($"Vision anchor UDP receiver started: 0.0.0.0:{listenPort}");
            WriteDiagnostic("UDP_START", $"port={listenPort}");
        }
        catch (Exception e)
        {
            Debug.LogWarning($"Vision anchor UDP receiver failed: {e.Message}");
            WriteDiagnostic("UDP_START_FAIL", e.Message);
            StopReceiver();
        }
    }

    private void StopReceiver()
    {
        running = false;

        if (udpClient != null)
        {
            udpClient.Close();
            udpClient = null;
        }

        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Join(300);
        }

        receiveThread = null;
    }

    private void ReceiveLoop()
    {
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
        while (running && udpClient != null)
        {
            try
            {
                byte[] data = udpClient.Receive(ref remoteEndPoint);
                string json = Encoding.UTF8.GetString(data);
                int queueBefore;
                lock (packetLock)
                {
                    queueBefore = packetQueue.Count;
                    if (packetQueue.Count >= MaxQueuedPackets)
                    {
                        packetQueue.Dequeue();
                        WriteDiagnostic("QUEUE_DROP", $"max={MaxQueuedPackets}");
                    }
                    packetQueue.Enqueue(json);
                }
                WriteDiagnostic("UDP_RECV", $"bytes={data.Length} remote={remoteEndPoint.Address}:{remoteEndPoint.Port} queueBefore={queueBefore} json={TrimForLog(json)}");
            }
            catch (SocketException)
            {
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Vision anchor UDP receive error: {e.Message}");
                WriteDiagnostic("UDP_RECV_ERROR", e.Message);
            }
        }
    }

    private void InitializeDiagnosticLog()
    {
        if (!writeDiagnosticFileLog) return;

        diagnosticLogPath = Path.Combine(Application.persistentDataPath, diagnosticLogFileName);
        string header = $"# Vision finger diagnostic log\n# started={DateTime.Now:yyyy-MM-dd HH:mm:ss.fff}\n# sceneObject={gameObject.name}\n";

        lock (diagnosticLogLock)
        {
            try
            {
                if (clearDiagnosticLogOnStart)
                    File.WriteAllText(diagnosticLogPath, header);
                else
                    File.AppendAllText(diagnosticLogPath, "\n" + header);

                Debug.Log($"Vision diagnostic log path: {diagnosticLogPath}");
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Vision diagnostic log init failed: {e.Message}");
                writeDiagnosticFileLog = false;
            }
        }
    }

    private void WriteDiagnostic(string stage, string message)
    {
        string line = $"{DateTime.Now:HH:mm:ss.fff}\t{stage}\t{message}\n";

        if (writeDiagnosticFileLog && !string.IsNullOrEmpty(diagnosticLogPath))
        {
            lock (diagnosticLogLock)
            {
                try
                {
                    File.AppendAllText(diagnosticLogPath, line);
                }
                catch
                {
                    writeDiagnosticFileLog = false;
                }
            }
        }

        if (printPacketDiagnostics && stage != "UDP_RECV" && stage != "QUEUE_DRAIN")
        {
            Debug.Log($"[VisionDiag] {stage}: {message}");
        }
    }

    private string TrimForLog(string value)
    {
        if (string.IsNullOrEmpty(value)) return string.Empty;
        return value.Length <= 240 ? value : value.Substring(0, 240) + "...";
    }
}
