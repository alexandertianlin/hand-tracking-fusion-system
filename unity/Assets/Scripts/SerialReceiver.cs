using System;
using System.IO.Ports;
using System.Threading;
using System.Collections.Concurrent;
using UnityEngine;
using System.Collections.Generic;

public class SerialReceiver : MonoBehaviour
{
    [Header("串口配置")]
    public string portName = "COM122";
    public int baudRate = 460800;

    [Header("手指绑定")]
    public List<FingerSolver> fingerSolvers = new List<FingerSolver>();
    private Dictionary<int, FingerSolver> idToFinger = new Dictionary<int, FingerSolver>();

    private const byte FRAME_HEADER_0 = 0xB5;
    private const byte FRAME_HEADER_1 = 0xA5;
    private const byte FRAME_HEADER_2 = 0x55;
    private const int PACKET_LEN = 35;
    private const float QUAT_SCALE = 10000.0f;
    private const byte FILTER_ID = 0x30;

    private SerialPort serialPort;
    private Thread receiveThread;
    private bool isRunning;
    private byte[] receiveBuffer = new byte[1024];

    public ConcurrentDictionary<int, Quaternion> ImuDataDict = new ConcurrentDictionary<int, Quaternion>();
    public ConcurrentDictionary<int, Vector3> ForceDataDict = new ConcurrentDictionary<int, Vector3>();
    public ConcurrentDictionary<int, string> RawHexDict = new ConcurrentDictionary<int, string>();

    void Start()
    {
        foreach (var finger in fingerSolvers)
        {
            if (!idToFinger.ContainsKey(finger.deviceId))
                idToFinger.Add(finger.deviceId, finger);
        }

        try
        {
            serialPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One) { ReadTimeout = 500 };
            serialPort.Open();
            isRunning = true;
            receiveThread = new Thread(ReceiveThread) { IsBackground = true };
            receiveThread.Start();
            Debug.Log($"<color=#00FF00>✅ 串口 {portName} 打开成功</color>");
        }
        catch (Exception e)
        {
            Debug.LogError($"❌ 串口打开失败: {e.Message}");
        }
    }


    private void ReceiveThread()
    {
        while (isRunning && serialPort != null && serialPort.IsOpen)
        {
            try
            {
                while (serialPort.ReadByte() != FRAME_HEADER_0) { }
                if (serialPort.ReadByte() != FRAME_HEADER_1) continue;
                if (serialPort.ReadByte() != FRAME_HEADER_2) continue;

                receiveBuffer[0] = FRAME_HEADER_0;
                receiveBuffer[1] = FRAME_HEADER_1;
                receiveBuffer[2] = FRAME_HEADER_2;

                int bytesToRead = PACKET_LEN - 3;
                int bytesRead = 0;
                while (bytesRead < bytesToRead && isRunning)
                {
                    int read = serialPort.Read(receiveBuffer, 3 + bytesRead, bytesToRead - bytesRead);
                    if (read <= 0) break;
                    bytesRead += read;
                }

                if (bytesRead == bytesToRead)
                {
                    ParsePacket(receiveBuffer);
                }
            }
            catch { }
        }
    }

    private void ParsePacket(byte[] data)
    {
        int deviceId = data[6];
        string rawHex = BitConverter.ToString(data, 0, PACKET_LEN).Replace("-", " ");
        RawHexDict[deviceId] = rawHex;

        short w = BitConverter.ToInt16(data, 8);
        short x = BitConverter.ToInt16(data, 10);
        short y = BitConverter.ToInt16(data, 12);
        short z = BitConverter.ToInt16(data, 14);

        float qw = w / QUAT_SCALE;
        float qx = x / QUAT_SCALE;
        float qy = y / QUAT_SCALE;
        float qz = z / QUAT_SCALE;

        float norm = Mathf.Sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        Quaternion rot = norm > 0.001f ? new Quaternion(qx / norm, qy / norm, qz / norm, qw / norm) : Quaternion.identity;
        ImuDataDict[deviceId] = rot;

        Vector3 curForce = Vector3.zero;
        if (deviceId != FILTER_ID)
        {
            float forceX = BitConverter.ToSingle(data, 22);
            float forceY = BitConverter.ToSingle(data, 26);
            float forceZ = BitConverter.ToSingle(data, 30);
            curForce = new Vector3(forceX, forceY, forceZ);
        }
        ForceDataDict[deviceId] = curForce;

        string forceInfo = deviceId == FILTER_ID ? "力[屏蔽]" : $"力X:{curForce.x:F2} Y:{curForce.y:F2} Z:{curForce.z:F2}";
        Debug.Log($"ID:{deviceId:X2} | {forceInfo} | 四元数:[{qw:F2},{qx:F2},{qy:F2},{qz:F2}] | 原始:{rawHex}");
    }

    void OnDestroy()
    {
        isRunning = false;
        if (serialPort != null && serialPort.IsOpen) serialPort.Close();
        if (receiveThread != null && receiveThread.IsAlive) receiveThread.Join(200);
    }
}