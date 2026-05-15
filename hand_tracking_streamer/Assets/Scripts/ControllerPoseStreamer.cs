using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class ControllerPoseStreamer : MonoBehaviour
{
    public enum ControllerSide { Left, Right }

    [Header("Configuration")]
    [SerializeField] private ControllerSide side;
    [SerializeField] private float frequencySeconds = 1f / 30f;

    [Header("Logging")]
    [SerializeField] private bool logToHUD = true;
    [SerializeField] private string hudLogSource = "Right";

    public ControllerSide Side => side;

    private UdpClient _udpClient;
    private TcpClient _tcpClient;
    private NetworkStream _tcpStream;
    private IPEndPoint _remoteEndPoint;
    private bool _isInitialized;
    private int _currentProtocol = -1;
    private float _timer;

    private readonly StringBuilder _sbPacket = new StringBuilder(256);
    private readonly StringBuilder _sbLog = new StringBuilder(512);
    private uint _frameId;

    // Optimized buffer to avoid per-frame allocations
    private byte[] _sendBuffer = new byte[1024];
    private float _hudTimer;
    private const float HUD_UPDATE_INTERVAL = 0.2f;

    private static readonly double TicksToNs = 1_000_000_000.0 / Stopwatch.Frequency;

    public void SetSide(ControllerSide newSide) => side = newSide;

    private void Update()
    {
        if (AppManager.Instance == null || !AppManager.Instance.isStreaming || !AppManager.Instance.TrackControllers)
        {
            if (_isInitialized) Disconnect();
            return;
        }

        if (!_isInitialized) InitializeNetwork();

        OVRInput.Controller controller = (side == ControllerSide.Left) ? OVRInput.Controller.LTouch : OVRInput.Controller.RTouch;
        
        bool isTracked = (OVRInput.GetConnectedControllers() & controller) != 0;
        if (!isTracked) return;

        _timer += Time.deltaTime;
        if (_timer < frequencySeconds) return;
        _timer = 0f;

        Vector3 pos = OVRInput.GetLocalControllerPosition(controller);
        Quaternion rot = OVRInput.GetLocalControllerRotation(controller);
        
        // Grasp signal: Index Trigger (Analog)
        float grasp = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, controller);

        BuildAndSendPacket(pos, rot, grasp);
    }

    private void OnDestroy() => Disconnect();

    private void BuildAndSendPacket(Vector3 position, Quaternion rotation, float grasp)
    {
        _sbPacket.Clear();
        _sbLog.Clear();

        bool addDebugHeaderMeta = AppManager.Instance != null && AppManager.Instance.ShowDebugInfo;
        string sidePrefix = (side == ControllerSide.Left) ? "Left" : "Right";

        if (addDebugHeaderMeta)
        {
            _frameId++;
            AppendHeaderWithMeta(_sbPacket, section: sidePrefix + " controller", _frameId, GetMonotonicTimestampNs());
            _sbPacket.Append(", ");
        }
        else
        {
            _sbPacket.Append(sidePrefix).Append(" controller:, ");
        }

        // CSV: px, py, pz, qx, qy, qz, qw, ..., grasp
        AppendVector3(_sbPacket, position);
        _sbPacket.Append(", ");
        AppendQuaternion(_sbPacket, rotation);
        
        // Match the 18th position expected by the reconstructed bridge.py
        // bridge.py expects grasp at index 17/18 in CSV parts (raw_vals[16])
        // We add dummy values if needed, but our bridge handles index-based extraction.
        // Let's add padding commas to be safe or just append it.
        _sbPacket.Append(", 0, 0, 0, 0, 0, 0, 0, 0, 0, "); // Padding
        _sbPacket.Append(grasp.ToString("F1"));

        if (logToHUD)
        {
            _hudTimer += Time.deltaTime;
            if (_hudTimer >= HUD_UPDATE_INTERVAL)
            {
                _hudTimer = 0f;
                _sbLog.AppendLine($"=== [{sidePrefix}] Ctrl ===");
                _sbLog.Append("Grasp: ").AppendLine(grasp > 0.5f ? "ON" : "OFF");
                _sbLog.Append("Pos: ").AppendLine(FormatVector3Tuple(position));
                LogHUDSlot(sidePrefix.ToLower() + "_ctrl", _sbLog.ToString());
            }
        }

        SendOptimized(_sbPacket);
    }

    private void SendOptimized(StringBuilder sb)
    {
        if (AppManager.Instance == null || !AppManager.Instance.isStreaming) return;
        if (!_isInitialized) return;

        try
        {
            int byteCount = Encoding.UTF8.GetBytes(sb.ToString(), 0, sb.Length, _sendBuffer, 0);

            if (_currentProtocol == 0 && _udpClient != null)
            {
                _udpClient.Send(_sendBuffer, byteCount, _remoteEndPoint);
            }
            else if ((_currentProtocol == 1 || _currentProtocol == 2) && _tcpStream != null && _tcpStream.CanWrite)
            {
                if (byteCount < _sendBuffer.Length - 1)
                {
                    _sendBuffer[byteCount] = (byte)'\n';
                    _tcpStream.Write(_sendBuffer, 0, byteCount + 1);
                }
            }
        }
        catch (Exception ex)
        {
            Disconnect();
            if (AppManager.Instance != null) AppManager.Instance.HandleDisconnection($"{side} controller send failed: {ex.Message}");
        }
    }

    private void InitializeNetwork()
    {
        if (AppManager.Instance == null) return;

        string ip = AppManager.Instance.ServerIP;
        int port = AppManager.Instance.ServerPort;
        _currentProtocol = AppManager.Instance.SelectedProtocol;

        try
        {
            if (_currentProtocol == 0)
            {
                _udpClient = new UdpClient();
                _remoteEndPoint = new IPEndPoint(IPAddress.Parse(ip), port);
            }
            else
            {
                _tcpClient = new TcpClient(AddressFamily.InterNetwork);
                _tcpClient.NoDelay = true;
                _tcpClient.Connect(ip, port);
                _tcpStream = _tcpClient.GetStream();
            }
            _isInitialized = true;
        }
        catch (Exception ex)
        {
            UnityEngine.Debug.LogError($"[Controller] {side} Conn Error: {ex.Message}");
            if (AppManager.Instance != null) AppManager.Instance.StopStreaming();
        }
    }

    private void Disconnect()
    {
        try
        {
            if (_udpClient != null) { _udpClient.Close(); _udpClient = null; }
            if (_tcpStream != null) { _tcpStream.Close(); _tcpStream = null; }
            if (_tcpClient != null) { _tcpClient.Close(); _tcpClient = null; }
        }
        catch { }
        _isInitialized = false;
    }

    private static ulong GetMonotonicTimestampNs() => (ulong)(Stopwatch.GetTimestamp() * TicksToNs);

    private void AppendHeaderWithMeta(StringBuilder sb, string section, uint frameId, ulong sendTimestampNs)
    {
        sb.Append(section).Append(" | f = ").Append(frameId).Append(" | t = ").Append(sendTimestampNs).Append(":");
    }

    private void AppendVector3(StringBuilder sb, Vector3 vec)
    {
        sb.Append(vec.x.ToString("F4")).Append(", ").Append(vec.y.ToString("F4")).Append(", ").Append(vec.z.ToString("F4"));
    }

    private void AppendQuaternion(StringBuilder sb, Quaternion q)
    {
        sb.Append(q.x.ToString("F3")).Append(", ").Append(q.y.ToString("F3")).Append(", ").Append(q.z.ToString("F3")).Append(", ").Append(q.w.ToString("F3"));
    }

    private string FormatVector3Tuple(Vector3 vec) => $"({vec.x:F3}, {vec.y:F3}, {vec.z:F3})";

    private void LogHUDSlot(string slotKey, string msg)
    {
        if (logToHUD && LogManager.Instance != null) LogManager.Instance.LogSlot(hudLogSource, slotKey, msg);
    }
}
