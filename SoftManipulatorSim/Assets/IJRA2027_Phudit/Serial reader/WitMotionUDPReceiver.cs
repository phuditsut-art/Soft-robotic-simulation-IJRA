using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class WitMotionUDPReceiver : MonoBehaviour
{
    // Singleton instance for easy scene-wide access
    public static WitMotionUDPReceiver Instance { get; private set; }

    [Header("Network Configuration")]
    [Tooltip("Port matching the UDP_PORT in your Python BLE script.")]
    public int listenPort = 5005;

    [Header("Live Sensor Data (Read-Only)")]
    public float roll = 0f;
    public float pitch = 0f;
    public float yaw = 0f;

    public float accX = 0f;
    public float accY = 0f;
    public float accZ = 0f;

    [Header("Processed Outputs")]
    [Tooltip("Raw Euler angles received from sensor (X=Roll, Y=Pitch, Z=Yaw).")]
    public Vector3 eulerAngles;

    [Tooltip("Calculated rotation Quaternion ready to apply directly to Unity GameObjects.")]
    public Quaternion sensorRotation = Quaternion.identity;

    [Tooltip("Linear acceleration values (g).")]
    public Vector3 acceleration;

    // Threading & Network variables
    private UdpClient _udpClient;
    private Thread _receiveThread;
    private bool _isRunning = false;

    // Buffer variables to safely cross thread boundaries
    private float _threadRoll, _threadPitch, _threadYaw;
    private float _threadAccX, _threadAccY, _threadAccZ;
    private readonly object _lock = new object();

    private void Awake()
    {
        // Singleton initialization
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }
        Instance = this;
    }

    private void Start()
    {
        _isRunning = true;
        
        // Start background thread for non-blocking network socket listening
        _receiveThread = new Thread(ReceiveDataThread)
        {
            IsBackground = true
        };
        _receiveThread.Start();
        
        Debug.Log($"[WitMotionUDPReceiver] Listening for Python BLE stream on port {listenPort}...");
    }

    private void ReceiveDataThread()
    {
        try
        {
            _udpClient = new UdpClient(listenPort);
            IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);

            while (_isRunning)
            {
                // Blocks until a packet arrives
                byte[] data = _udpClient.Receive(ref remoteEndPoint);
                string message = Encoding.UTF8.GetString(data);

                if (!string.IsNullOrEmpty(message))
                {
                    ParsePayload(message);
                }
            }
        }
        catch (SocketException ex)
        {
            // Socket closed cleanly on app quit or error
            if (_isRunning)
            {
                Debug.LogWarning($"[WitMotionUDPReceiver] Socket Exception: {ex.Message}");
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"[WitMotionUDPReceiver] Thread Error: {ex.Message}");
        }
    }

    private void ParsePayload(string payload)
    {
        // Expected CSV format: roll,pitch,yaw,acc_x,acc_y,acc_z
        string[] tokens = payload.Split(',');
        if (tokens.Length == 6)
        {
            if (float.TryParse(tokens[0], NumberStyles.Float, CultureInfo.InvariantCulture, out float r) &&
                float.TryParse(tokens[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float p) &&
                float.TryParse(tokens[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float y) &&
                float.TryParse(tokens[3], NumberStyles.Float, CultureInfo.InvariantCulture, out float ax) &&
                float.TryParse(tokens[4], NumberStyles.Float, CultureInfo.InvariantCulture, out float ay) &&
                float.TryParse(tokens[5], NumberStyles.Float, CultureInfo.InvariantCulture, out float az))
            {
                lock (_lock)
                {
                    _threadRoll = r;
                    _threadPitch = p;
                    _threadYaw = y;
                    _threadAccX = ax;
                    _threadAccY = ay;
                    _threadAccZ = az;
                }
            }
        }
    }

    private void Update()
    {
        // Sync values safely from the background thread onto the Unity main thread
        lock (_lock)
        {
            roll = _threadRoll;
            pitch = _threadPitch;
            yaw = _threadYaw;

            accX = _threadAccX;
            accY = _threadAccY;
            accZ = _threadAccZ;
        }

        // Map values to Unity coordinate system
        eulerAngles = new Vector3(roll, pitch, yaw);
        acceleration = new Vector3(accX, accY, accZ);

        // Convert Euler angles to Quaternion rotation (adjust axes if needed for your model origin)
        sensorRotation = Quaternion.Euler(-pitch, -yaw, roll);
    }

    private void OnDisable()
    {
        CleanUpThread();
    }

    private void OnApplicationQuit()
    {
        CleanUpThread();
    }

    private void CleanUpThread()
    {
        _isRunning = false;
        
        if (_udpClient != null)
        {
            _udpClient.Close();
            _udpClient = null;
        }

        if (_receiveThread != null && _receiveThread.IsAlive)
        {
            _receiveThread.Join(100);
            _receiveThread = null;
        }
    }
}