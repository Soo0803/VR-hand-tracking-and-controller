using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

/// <summary>
/// Streams Quest controller pose data (position, rotation, direction vectors)
/// over TCP or UDP, following the same architecture as HeadPoseStreamer.
/// Attach one instance per controller side to separate GameObjects.
/// </summary>
public class ControllerPoseStreamer : MonoBehaviour
{
    public enum ControllerSide { Left, Right }

    [Header("Configuration")]
    [SerializeField] private ControllerSide _controllerSide;
    [SerializeField] private float _frequencySeconds = 1f / 30f; // 30 Hz

    [Header("Logging")]
    [SerializeField] private bool _logToHUD = true;
    [SerializeField] private string _hudLogSource = "Right";

    /// <summary>Public accessor for the configured side.</summary>
    public ControllerSide Side => _controllerSide;

    /// <summary>Set the controller side at runtime (used by AppManager auto-creation).</summary>
    public void SetSide(ControllerSide side)
    {
        _controllerSide = side;
        _hudLogSource = side == ControllerSide.Left ? "Left" : "Right";
    }

    // Networking (same pattern as HeadPoseStreamer)
    private UdpClient _udpClient;
    private TcpClient _tcpClient;
    private NetworkStream _tcpStream;
    private IPEndPoint _remoteEndPoint;
    private bool _isInitialized;
    private int _currentProtocol = -1;
    private float _timer;

    // Packet building
    private readonly StringBuilder _sbPacket = new StringBuilder(512);
    private readonly StringBuilder _sbLog = new StringBuilder(256);
    private uint _frameId;

    private static readonly double TicksToNs = 1_000_000_000.0 / Stopwatch.Frequency;

    // Tracking debounce — prevents rapid on/off blinking
    private bool _debouncedIsTracked = false;
    private float _trackingLostTimer = 0f;
    private const float TRACKING_GRACE_PERIOD = 0.15f; // seconds to wait before declaring "lost"

    // Landmark visualization
    [Header("Visualization")]
    [SerializeField] private GameObject _axisPrefab;
    private ControllerLandmarkVisualizer _landmarkVisualizer;

    /// <summary>The OVRInput.Controller enum for the selected side.</summary>
    private OVRInput.Controller OvrController =>
        _controllerSide == ControllerSide.Left
            ? OVRInput.Controller.LTouch
            : OVRInput.Controller.RTouch;

    private void Update()
    {
        // 1. Gate on AppManager streaming state + controller toggle
        if (AppManager.Instance == null
            || !AppManager.Instance.isStreaming
            || !AppManager.Instance.TrackControllers)
        {
            if (_isInitialized) Disconnect();
            return;
        }

        // 2. Ensure network is initialised
        if (!_isInitialized) InitializeNetwork();

        // 3. Rate limiting
        _timer += Time.deltaTime;
        if (_timer < _frequencySeconds) return;
        _timer = 0f;

        BuildAndSendPacket();
    }

    private void OnDestroy()
    {
        Disconnect();
    }

    // ─────────────────────────  PACKET BUILDING  ─────────────────────────

    private void BuildAndSendPacket()
    {
        _sbPacket.Clear();
        _sbLog.Clear();

        OVRInput.Controller ctrl = OvrController;

        // Raw tracking state
        bool rawTracked = OVRInput.GetControllerPositionTracked(ctrl)
                       && OVRInput.GetControllerOrientationTracked(ctrl);

        // Debounce: immediate ON, delayed OFF (prevents blinking)
        if (rawTracked)
        {
            _debouncedIsTracked = true;
            _trackingLostTimer = 0f;
        }
        else
        {
            _trackingLostTimer += _frequencySeconds;
            if (_trackingLostTimer >= TRACKING_GRACE_PERIOD)
            {
                _debouncedIsTracked = false;
            }
        }

        bool isTracked = _debouncedIsTracked;

        // Pose
        Vector3 position = OVRInput.GetLocalControllerPosition(ctrl);
        Quaternion rotation = OVRInput.GetLocalControllerRotation(ctrl);

        // Direction vectors derived from rotation
        Vector3 forward = rotation * Vector3.forward;
        Vector3 up      = rotation * Vector3.up;
        Vector3 right   = rotation * Vector3.right;

        // button state
        bool isButtonPressed = OVRInput.Get(OVRInput.Button.One, ctrl);

        // ── Header ──
        bool addDebugMeta = AppManager.Instance != null && AppManager.Instance.ShowDebugInfo;
        if (addDebugMeta)
        {
            _frameId++;
            ulong ts = GetMonotonicTimestampNs();
            AppendHeaderWithMeta(_sbPacket, "controller", _frameId, ts);
            _sbPacket.Append(", ");
        }
        else
        {
            _sbPacket.Append(_controllerSide).Append(" controller:, ");
        }

        // ── Payload ──
        // tracked flag
        _sbPacket.Append(isTracked ? "1" : "0");

        // position
        _sbPacket.Append(", ");
        AppendVector3(_sbPacket, position);

        // rotation quaternion
        _sbPacket.Append(", ");
        AppendQuaternion(_sbPacket, rotation);

        // forward vector
        _sbPacket.Append(", ");
        AppendVector3(_sbPacket, forward);

        // up vector
        _sbPacket.Append(", ");
        AppendVector3(_sbPacket, up);

        // right vector
        _sbPacket.Append(", ");
        AppendVector3(_sbPacket, right);

        // button state (Grasp signal)
        _sbPacket.Append(", ");
        _sbPacket.Append(isButtonPressed ? "1" : "0");

        // ── HUD Log ──
        if (_logToHUD)
        {
            _sbLog.AppendLine($"=== [{_controllerSide}] Controller ===");
            _sbLog.AppendLine($"Tracked: {isTracked}");
            _sbLog.AppendLine($"Pos: {FormatVec3(position)}");
            _sbLog.AppendLine($"Rot: {FormatQuat(rotation)}");
            _sbLog.AppendLine($"Button: {(isButtonPressed ? "PRESSED" : "Released")}");
            LogHUDSlot("controller", _sbLog.ToString());
        }

        SendData(_sbPacket.ToString());
    }

    // ─────────────────────────  NETWORK HELPERS  ─────────────────────────
    // (Identical to HeadPoseStreamer — each streamer owns its own socket)

    private void InitializeNetwork()
    {
        if (AppManager.Instance == null) return;

        string ip = AppManager.Instance.ServerIP;
        int port = AppManager.Instance.ServerPort;
        _currentProtocol = AppManager.Instance.SelectedProtocol;

        try
        {
            if (_currentProtocol == 0) // UDP
            {
                _udpClient = new UdpClient();
                _udpClient.Client.SendBufferSize = 0;
                _remoteEndPoint = new IPEndPoint(IPAddress.Parse(ip), port);
                LogHUD($"Controller UDP Ready: {ip}:{port}");
            }
            else // TCP (Wired=1 or Wireless=2)
            {
                _tcpClient = new TcpClient(AddressFamily.InterNetwork);
                _tcpClient.NoDelay = true;
                _tcpClient.SendTimeout = 1000;
                _tcpClient.ReceiveTimeout = 1000;
                _tcpClient.Connect(ip, port);
                _tcpStream = _tcpClient.GetStream();
                string type = _currentProtocol == 1 ? "Wired" : "WiFi";
                LogHUD($"Controller TCP({type}) Connected: {ip}:{port}");
            }
            _isInitialized = true;
        }
        catch (Exception ex)
        {
            LogHUD($"Controller Conn Error: {ex.Message}");
            if (AppManager.Instance != null)
            {
                AppManager.Instance.StopStreaming();
            }
        }
    }

    private void SendData(string message)
    {
        if (AppManager.Instance != null && !AppManager.Instance.isStreaming) return;

        try
        {
            if (_currentProtocol == 0 && _udpClient != null)
            {
                byte[] data = Encoding.UTF8.GetBytes(message);
                _udpClient.Send(data, data.Length, _remoteEndPoint);
            }
            else if ((_currentProtocol == 1 || _currentProtocol == 2) && _tcpStream != null && _tcpStream.CanWrite)
            {
                byte[] data = Encoding.UTF8.GetBytes(message + "\n");
                _tcpStream.Write(data, 0, data.Length);
            }
        }
        catch (Exception ex)
        {
            Disconnect();
            if (AppManager.Instance != null)
            {
                AppManager.Instance.HandleDisconnection("Controller send failed: " + ex.Message);
            }
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

    // ─────────────────────────  FORMATTING HELPERS  ─────────────────────────

    private static ulong GetMonotonicTimestampNs()
    {
        return (ulong)(Stopwatch.GetTimestamp() * TicksToNs);
    }

    private void AppendHeaderWithMeta(StringBuilder sb, string section, uint frameId, ulong sendTimestampNs)
    {
        sb.Append(_controllerSide)
          .Append(" ")
          .Append(section)
          .Append(" | f = ")
          .Append(frameId)
          .Append(" | t = ")
          .Append(sendTimestampNs)
          .Append(":");
    }

    private static void AppendVector3(StringBuilder sb, Vector3 vec)
    {
        sb.Append(vec.x.ToString("F4")).Append(", ")
          .Append(vec.y.ToString("F4")).Append(", ")
          .Append(vec.z.ToString("F4"));
    }

    private static void AppendQuaternion(StringBuilder sb, Quaternion q)
    {
        sb.Append(q.x.ToString("F3")).Append(", ")
          .Append(q.y.ToString("F3")).Append(", ")
          .Append(q.z.ToString("F3")).Append(", ")
          .Append(q.w.ToString("F3"));
    }

    private static string FormatVec3(Vector3 v) => $"({v.x:F3}, {v.y:F3}, {v.z:F3})";
    private static string FormatQuat(Quaternion q) => $"({q.x:F3}, {q.y:F3}, {q.z:F3}, {q.w:F3})";

    private void LogHUD(string msg)
    {
        if (_logToHUD && LogManager.Instance != null)
        {
            LogManager.Instance.Log(_hudLogSource, msg);
        }
    }

    private void LogHUDSlot(string slotKey, string msg)
    {
        if (_logToHUD && LogManager.Instance != null)
        {
            LogManager.Instance.LogSlot(_hudLogSource, slotKey, msg);
        }
    }
}
