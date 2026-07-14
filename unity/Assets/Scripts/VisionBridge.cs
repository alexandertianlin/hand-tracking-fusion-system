using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

/// <summary>
/// VisionBridge �?视觉+IMU 双模控制桥接
///
/// 视觉（MediaPipe）→ UDP curl/spread �?调用 HandMotionManager.ApplyVisionAnchorPose
/// 视觉超时�?0.5s 无数据）�?IMU（HandMotionManager）自然接�?///
/// 使用方法�?/// 1. 拖到 SampleScene 任意 GameObject �?/// 2. 将场景里�?HandMotionManager 拖到 motionManager 字段
/// 3. �?Play
/// </summary>
[RequireComponent(typeof(HandMotionManager))]
public class VisionBridge : MonoBehaviour
{
    [Header("UDP")]
    public int listenPort = 5055;
    public bool enableLog = true;

    [Header("平滑过渡")]
    [Range(0f, 1f)] public float blendAlpha = 0.85f;  // 0.85 = 平滑过渡
    public float visionTimeout = 0.5f;  // 视觉超时后切�?IMU

    private HandMotionManager motionManager;
    private UdpClient udpClient;
    private Thread recvThread;
    private volatile bool running;
    private readonly Queue<string> packetQueue = new Queue<string>(16);
    private readonly object lockObj = new object();
    private float lastPacketTime = -99f;
    private float[] lastCurls = new float[5];

    void Start()
    {
        motionManager = GetComponent<HandMotionManager>();
        StartReceiver();
    }

    void OnDestroy() { StopReceiver(); }

    void Update()
    {
        // 处理 UDP 队列
        List<string> packets = null;
        lock (lockObj)
        {
            if (packetQueue.Count > 0)
            {
                packets = new List<string>(packetQueue.Count);
                while (packetQueue.Count > 0)
                    packets.Add(packetQueue.Dequeue());
            }
        }

        if (packets != null && packets.Count > 0)
        {
            var pkt = JsonUtility.FromJson<VisionPacket>(packets[packets.Count - 1]);
            if (pkt != null && pkt.type == "hamer_hand")
            {
            if (!string.IsNullOrEmpty(pkt.command) && pkt.command == "calibrate")
            {
                motionManager.Calibrate();
                lastPacketTime = Time.time;
                return;
            }
                if (pkt.num_hands == 0)
                {
                    // No hand detected: force IMU takeover immediately
                    lastPacketTime = Time.time - visionTimeout - 1f;
                    for (int i = 0; i < 5; i++) lastCurls[i] = 0f;
                }
                else
                {
                    lastCurls[0] = pkt.hand_0_curl_thumb;
                    lastCurls[1] = pkt.hand_0_curl_index;
                    lastCurls[2] = pkt.hand_0_curl_middle;
                    lastCurls[3] = pkt.hand_0_curl_ring;
                    lastCurls[4] = pkt.hand_0_curl_little;
                    lastPacketTime = Time.time;
                }
            }
        }

        float elapsed = Time.time - lastPacketTime;
        if (elapsed > visionTimeout || !motionManager.IsCalibrated)
            return;  // 超时→IMU 自然接管

        // 视觉控制：对每根手指�?pitch/yaw
        // curl [0,1] �?pitch [0, maxPitch], spread [0,1] �?yaw [0, maxYaw]
        FingerSolver[] fingers = { motionManager.thumb, motionManager.index,
                                   motionManager.middle, motionManager.ring, motionManager.little };
        for (int i = 0; i < 5; i++)
        {
            float curl = Mathf.Clamp01(lastCurls[i]);
            float pitch = curl * fingers[i].MaxPitch;
            float yaw = 0f;

            motionManager.ApplyVisionAnchorPose(
                fist: false,
                openPitch: pitch,
                openYaw: yaw,
                useFingerMaxPitchForFist: true,
                fallbackFistPitch: 60f,
                fistYaw: 0f,
                blendAlpha: blendAlpha,
                fingerIndex: i
            );
        }

        if (enableLog && Time.frameCount % 120 == 0)
            Debug.Log($"[VisionBridge] curl:[T:{lastCurls[0]:F2} I:{lastCurls[1]:F2} M:{lastCurls[2]:F2} R:{lastCurls[3]:F2} L:{lastCurls[4]:F2}] alpha:{blendAlpha:F2}");
    }

    void StartReceiver()
    {
        if (udpClient != null) return;
        try
        {
            udpClient = new UdpClient(listenPort);
            udpClient.Client.ReceiveTimeout = 200;
            running = true;
            recvThread = new Thread(ReceiveLoop) { IsBackground = true };
            recvThread.Start();
            Debug.Log($"[VisionBridge] 监听 {listenPort}");
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[VisionBridge] 失败: {e.Message}");
            StopReceiver();
        }
    }

    void StopReceiver()
    {
        running = false;
        if (udpClient != null) { udpClient.Close(); udpClient = null; }
        if (recvThread != null && recvThread.IsAlive) recvThread.Join(300);
        recvThread = null;
    }

    void ReceiveLoop()
    {
        IPEndPoint remote = new IPEndPoint(IPAddress.Any, 0);
        while (running && udpClient != null)
        {
            try
            {
                byte[] data = udpClient.Receive(ref remote);
                string json = Encoding.UTF8.GetString(data);
                lock (lockObj) { packetQueue.Enqueue(json); }
            }
            catch (SocketException) { }
            catch (ObjectDisposedException) { break; }
        }
    }

    [Serializable]
    private class VisionPacket
    {
        public string type;
        public int num_hands;
        public string command;
        public float hand_0_curl_thumb;
        public float hand_0_curl_index;
        public float hand_0_curl_middle;
        public float hand_0_curl_ring;
        public float hand_0_curl_little;
    }
