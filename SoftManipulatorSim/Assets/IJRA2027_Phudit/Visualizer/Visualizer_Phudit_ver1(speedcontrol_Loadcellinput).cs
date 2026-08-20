using UnityEngine;
using System.Collections.Generic;
using System.IO.Ports;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Unity equivalent of PART B of SoftRobot_Simulation.py.
///
/// IMPLEMENTS BASE ROTATION: Segment 1 computes its bend in a fixed 2D plane
/// (internal phi = 0), and a rigid rotation matrix Rz(phi1) is applied.
/// 
/// IMPLEMENTS ANALOG SPEED CONTROL: Reads float values from load cells. 
/// Higher pressure = faster bending/spinning.
/// </summary>
[ExecuteAlways]
public class Visualizer_ver1_speedcontrol : MonoBehaviour
{
    // ------------------------------------------------------------ configuration
    [Header("Segment 1 (degrees)")]
    [Range(-360f, 360f)] public float theta1Deg = 0f;
    [Range(-180f, 180f)] public float phi1Deg   = 0f;
    [Range(-180f, 180f)] public float beta1Deg  = 0f;
    [Min(0.001f)]        public float L1        = 25f;

    [Header("Segment 2 (degrees)")]
    [Range(-360f, 360f)] public float theta2Deg = 0f;
    [Range(-180f, 180f)] public float phi2Deg   = 0f;
    [Range(-180f, 180f)] public float beta2Deg  = 0f;
    [Min(0.001f)]        public float L2        = 25f;

    [Header("Chain geometry (centimetres)")]
    [Min(0f)] public float sensorLength  = SoftRobotKinematics.DefaultSensorLength;
    [Min(0f)] public float diskThickness = SoftRobotKinematics.DefaultDiskThickness;

    [Header("Rendering")]
    public float worldScale = 0.01f;
    [Range(4, 200)] public int pointsPerSegment = 30;
    public float tubeRadius = 5.5f;
    [Range(3, 32)] public int tubeSides = 16;

    public Material tubeMaterial;
    public Material diskMaterial;
    public Material tipMaterial;

    public bool showDisks = true;
    public bool showEndEffector = true;

    [Header("Mounting plate (centimetres)")]
    public bool showBasePlate = true;
    [Min(1f)] public float basePlateRadius = 65f;
    [Min(0.1f)] public float basePlateThickness = 2f;
    public Material basePlateMaterial;

    [Header("Interaction (Analog Speed Control)")]
    public bool useArduinoControl = true; 
    public ArduinoSerialReader_Float serialReader; 
    
    [Tooltip("Maximum expected raw value from the load cell to normalize math")]
    public float maxSensorSignal = 1200f;
    
    [Tooltip("Adjusts how fast the robot moves relative to the pressure applied. Higher = more sensitive.")]
    public float analogSensitivityMultiplier = 1.0f;
    
    [Tooltip("Ignores tiny sensor fluctuations to prevent jittering.")]
    public float inputDeadzone = 0.05f;

    [Tooltip("Maximum base speed for bending (Up/Down) when fully pressed")]
    public float maxBendSpeedDegPerSec  = 90f;
    [Tooltip("Maximum base speed for spinning (Left/Right) when fully pressed")]
    public float maxPhiSpeedDegPerSec   = 120f;
    
    [Header("Auto-Return")]
    [Tooltip("Check this to make the robot smoothly return to a straight line when you release the controls.")]
    public bool autoReturnToZero = false;
    [Tooltip("How long to wait (in seconds) after releasing controls before auto-returning.")]
    public float autoReturnDelay = 0.75f;
    [Tooltip("How fast the robot springs back to straight (degrees per second).")]
    public float returnSpeedDegPerSec = 45f;

    [Header("XIAO Serial Control (Segment + FSR)")]
    public bool useXiaoSlider = true;
    public int xiaoPortNumber = 8;
    public int xiaoBaudRate = 9600;
    private SerialPort _xiaoStream;

    [Header("Elevation Control (On/Off Switch)")]
    [Tooltip("The FSR reading needed to trigger elevation.")]
    public float triggerThreshold = 1000f;
    public float elevationSpeed = 15f;
    public float maxElevationLimit = 50f;
    public float retractSpeed = 20f;
    public float currentInsertionCm = 0f;

    [Header("Active State")]
    [Range(1, 2)] public int activeSegment = 1;

    // ------------------------------------------------------------ runtime state
    private MeshFilter   _meshFilter;
    private MeshRenderer _meshRenderer;
    private Mesh         _mesh;
    private Transform[]  _diskMarkers;
    private Transform    _tipMarker;
    private Transform    _basePlate;

    private readonly float[] _theta = new float[2];
    private readonly float[] _phi   = new float[2];
    private readonly float[] _beta  = new float[2];
    private readonly float[] _len   = new float[2];

    private float _timeSinceLastInput = 0f;
    private float _continuousInputTime = 0f;
    private float _currentFsrReading = 0f;

    public Vector3 TipPosition { get; private set; }
    public Vector3 TipPositionMatlab { get; private set; }
    public IReadOnlyList<Vector3> BackboneMatlab => _backboneMatlab;
    private List<Vector3> _backboneMatlab;
    private bool _needsRebuild;

    // ------------------------------------------------------------------ lifecycle
    private void Start()
    {
        if (Application.isPlaying && useXiaoSlider)
        {
            string portName = "COM" + xiaoPortNumber;
            _xiaoStream = new SerialPort(portName, xiaoBaudRate);
            _xiaoStream.ReadTimeout = 15; 
            
            try
            {
                _xiaoStream.Open();
                Debug.Log($"Visualizer: Connected to XIAO on {portName}");
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Could not open XIAO port {portName}: {e.Message}");
            }
        }
    }

    private void OnEnable()
    {
        EnsureRenderers();
        _needsRebuild = true;
        Rebuild();
    }

    private void Update()
    {
        if (Application.isPlaying)
        {
            if (useArduinoControl) HandleInput(); 
            Rebuild();
        }
        else if (_needsRebuild)
        {
            _needsRebuild = false;
            Rebuild();
        }
    }

    private void OnValidate()
    {
        _needsRebuild = true;
    }

    private void OnDestroy()
    {
        if (_xiaoStream != null && _xiaoStream.IsOpen)
        {
            _xiaoStream.Close();
        }
    }

    // ---------------------------------------------------------------- the core
    private void Rebuild()
    {
        EnsureRenderers();

        _theta[0] = theta1Deg * Mathf.Deg2Rad * 0.5f; 
        _theta[1] = theta2Deg * Mathf.Deg2Rad * 0.5f;
        
        // --- THE KINEMATICS FIX ---
        // Lock internal phi1 to 0 so the math generates a pure, non-twisting 2D curve.
        _phi[0]   = 0f; 
        _phi[1]   = phi2Deg  * Mathf.Deg2Rad;
        
        _beta[0]  = beta1Deg * Mathf.Deg2Rad;
        _beta[1]  = beta2Deg * Mathf.Deg2Rad;
        _len[0]   = L1;
        _len[1]   = L2;

        List<Vector3> backboneMatlab = SoftRobotKinematics.ComputeBackbone(
            _theta, _phi, _beta, _len,
            Vector3.zero, Vector3.zero,
            out Matrix4x4 tip, out Matrix4x4[] diskFrames,
            pointsPerSegment, sensorLength, diskThickness);

        // --- APPLY RIGID BASE ROTATION Rz(phi1) AND ELEVATION ---
        float rad = phi1Deg * Mathf.Deg2Rad;
        float cosP = Mathf.Cos(rad);
        float sinP = Mathf.Sin(rad);

        for (int i = 0; i < backboneMatlab.Count; i++)
        {
            Vector3 p = backboneMatlab[i];
            p.z += currentInsertionCm; // Add FSR Elevation
            backboneMatlab[i] = new Vector3(p.x * cosP - p.y * sinP, p.x * sinP + p.y * cosP, p.z);
        }
        _backboneMatlab = backboneMatlab;

        Matrix4x4 Rz = Matrix4x4.identity;
        Rz.m00 = cosP;  Rz.m01 = -sinP;
        Rz.m10 = sinP;  Rz.m11 = cosP;

        tip.m23 += currentInsertionCm;
        tip = Rz * tip;

        for (int i = 0; i < diskFrames.Length; i++)
        {
            diskFrames[i].m23 += currentInsertionCm;
            diskFrames[i] = Rz * diskFrames[i];
        }

        // --- RENDER AS NORMAL ---
        var pts = new List<Vector3>(_backboneMatlab.Count);
        for (int i = 0; i < _backboneMatlab.Count; i++)
            pts.Add(SoftRobotKinematics.ToUnity(_backboneMatlab[i], worldScale));

        BuildTubeMesh(pts, tubeRadius * worldScale);

        if (showDisks)
        {
            EnsureDiskMarkers(diskFrames.Length);
            for (int i = 0; i < diskFrames.Length; i++)
            {
                _diskMarkers[i].gameObject.SetActive(true);

                Quaternion frameRot = SoftRobotKinematics.ToUnityRotation(diskFrames[i]);
                Vector3 axis = frameRot * Vector3.forward;
                float thick = diskThickness * worldScale;

                Vector3 farFace = SoftRobotKinematics.ToUnity(
                    SoftRobotKinematics.GetPosition(diskFrames[i]), worldScale);
                _diskMarkers[i].localPosition = farFace - axis * (thick * 0.5f);
                _diskMarkers[i].localRotation = frameRot * Quaternion.Euler(90f, 0f, 0f);

                float diameter = tubeRadius * worldScale * 2.2f;
                _diskMarkers[i].localScale = new Vector3(diameter, thick * 0.5f, diameter);
            }
        }
        else if (_diskMarkers != null)
        {
            foreach (var d in _diskMarkers)
                if (d != null) d.gameObject.SetActive(false);
        }

        TipPositionMatlab = SoftRobotKinematics.GetPosition(tip);
        Vector3 tipLocal = SoftRobotKinematics.ToUnity(TipPositionMatlab, worldScale);
        TipPosition = transform.TransformPoint(tipLocal);

        if (showEndEffector)
        {
            EnsureTipMarker();
            _tipMarker.gameObject.SetActive(true);
            _tipMarker.localPosition = tipLocal;
            _tipMarker.localRotation = SoftRobotKinematics.ToUnityRotation(tip);
            float s = tubeRadius * worldScale * 1.6f;
            _tipMarker.localScale = new Vector3(s, s, s);
        }
        else if (_tipMarker != null)
        {
            _tipMarker.gameObject.SetActive(false);
        }

        if (showBasePlate)
        {
            EnsureBasePlate();
            _basePlate.gameObject.SetActive(true);
            float thick = basePlateThickness * worldScale;
            
            // Baseplate moves up with the elevation
            float yPos = (currentInsertionCm * worldScale) - (thick * 0.5f);
            
            _basePlate.localPosition = new Vector3(0f, yPos, 0f);
            _basePlate.localRotation = Quaternion.identity;
            
            float d = basePlateRadius * worldScale * 2f;
            _basePlate.localScale = new Vector3(d, thick * 0.5f, d);
        }
        else if (_basePlate != null)
        {
            _basePlate.gameObject.SetActive(false);
        }
    }

    // --------------------------------------------------- procedural tube mesh
    private void BuildTubeMesh(List<Vector3> path, float radius)
    {
        var p = new List<Vector3>(path.Count);
        for (int i = 0; i < path.Count; i++)
            if (i == 0 || (path[i] - p[p.Count - 1]).sqrMagnitude > 1e-12f)
                p.Add(path[i]);

        if (_mesh == null)
        {
            _mesh = new Mesh { name = "SoftRobotTube" };
            _mesh.MarkDynamic();
        }

        if (p.Count < 2)
        {
            _mesh.Clear();
            _meshFilter.sharedMesh = _mesh;
            return;
        }

        int rings = p.Count;
        int sides = Mathf.Max(3, tubeSides);

        var verts   = new Vector3[rings * sides];
        var normals = new Vector3[rings * sides];
        var uvs     = new Vector2[rings * sides];

        Vector3 t0 = (p[1] - p[0]).normalized;
        Vector3 refUp = Mathf.Abs(Vector3.Dot(t0, Vector3.up)) > 0.9f ? Vector3.right : Vector3.up;
        Vector3 n = Vector3.Normalize(Vector3.Cross(refUp, t0));
        Vector3 prevTangent = t0;

        for (int i = 0; i < rings; i++)
        {
            Vector3 tangent = (i == 0)          ? (p[1] - p[0]).normalized
                            : (i == rings - 1)  ? (p[i] - p[i - 1]).normalized
                                                : (p[i + 1] - p[i - 1]).normalized;

            Quaternion swing = Quaternion.FromToRotation(prevTangent, tangent);
            n = swing * n;
            n = Vector3.Normalize(n - Vector3.Dot(n, tangent) * tangent);
            Vector3 b = Vector3.Cross(tangent, n);
            prevTangent = tangent;

            float v = i / (rings - 1f);
            for (int j = 0; j < sides; j++)
            {
                float a = (j / (float)sides) * Mathf.PI * 2f;
                Vector3 dir = Mathf.Cos(a) * n + Mathf.Sin(a) * b;
                int idx = i * sides + j;
                verts[idx]   = p[i] + dir * radius;
                normals[idx] = dir;
                uvs[idx]     = new Vector2(j / (float)sides, v);
            }
        }

        var tris = new int[(rings - 1) * sides * 6];
        int k = 0;
        for (int i = 0; i < rings - 1; i++)
        {
            for (int j = 0; j < sides; j++)
            {
                int a = i * sides + j;
                int bb = i * sides + (j + 1) % sides;
                int c = (i + 1) * sides + j;
                int d = (i + 1) * sides + (j + 1) % sides;

                tris[k++] = a; tris[k++] = c; tris[k++] = bb;
                tris[k++] = bb; tris[k++] = c; tris[k++] = d;
            }
        }

        _mesh.Clear();
        _mesh.indexFormat = verts.Length > 65000
            ? UnityEngine.Rendering.IndexFormat.UInt32
            : UnityEngine.Rendering.IndexFormat.UInt16;
        _mesh.vertices  = verts;
        _mesh.normals   = normals;
        _mesh.uv        = uvs;
        _mesh.triangles = tris;
        _mesh.RecalculateBounds();

        _meshFilter.sharedMesh = _mesh;
    }

    // -------------------------------------------------------------- input
    private void HandleInput()
    {
        float dt = Time.deltaTime;

        // 1. Read Serial Data for Segment Selection & FSR
        if (useXiaoSlider && _xiaoStream != null && _xiaoStream.IsOpen)
        {
            try
            {
                while (_xiaoStream.BytesToRead > 0)
                {
                    string incoming = _xiaoStream.ReadLine().Trim();
                    string[] parts = incoming.Split(',');

                    if (parts.Length == 2)
                    {
                        if (int.TryParse(parts[0], out int sliderState))
                        {
                            if (sliderState >= 1 && sliderState <= 2) activeSegment = sliderState;
                        }

                        if (float.TryParse(parts[1], out float fsr))
                        {
                            _currentFsrReading = fsr; 
                        }
                    }
                }
            }
            catch (System.TimeoutException) { }
            catch (System.Exception e) { Debug.LogWarning("XIAO Serial Error: " + e.Message); }
        }

        // 2. FSR Elevation Math
        if (_currentFsrReading >= triggerThreshold)
        {
            currentInsertionCm = Mathf.MoveTowards(currentInsertionCm, maxElevationLimit, elevationSpeed * dt);
        }
        else 
        {
            currentInsertionCm = Mathf.MoveTowards(currentInsertionCm, 0f, retractSpeed * dt);
        }

        // 3. Keyboard overrides
        if (KeyDown(KeyKind.Alpha1)) activeSegment = 1;
        if (KeyDown(KeyKind.Alpha2)) activeSegment = 2;
        if (KeyDown(KeyKind.R))      ResetToStraightPose();

        float inputThetaSpeedMultiplier = 0f;
        float inputPhiSpeedMultiplier   = 0f;

        // 4. Read Analog Float Data
        if (serialReader != null)
        {
            // Calculate a ratio from -1.0 to 1.0 based on sensor strength
            float upFloat = serialReader.loadCellUpValue / maxSensorSignal;
            float downFloat = serialReader.loadCellDownValue / maxSensorSignal;
            float rightFloat = serialReader.loadCellRightValue / maxSensorSignal;
            float leftFloat = serialReader.loadCellLeftValue / maxSensorSignal;

            inputThetaSpeedMultiplier = (upFloat - downFloat) * analogSensitivityMultiplier;
            inputPhiSpeedMultiplier   = (rightFloat - leftFloat) * analogSensitivityMultiplier;
        }

        // Apply deadzone to ignore tiny sensor noise
        if (Mathf.Abs(inputThetaSpeedMultiplier) < inputDeadzone) inputThetaSpeedMultiplier = 0f;
        if (Mathf.Abs(inputPhiSpeedMultiplier) < inputDeadzone) inputPhiSpeedMultiplier = 0f;

        // Clamp to prevent moving faster than the set maximums if pushed incredibly hard
        inputThetaSpeedMultiplier = Mathf.Clamp(inputThetaSpeedMultiplier, -2f, 2f);
        inputPhiSpeedMultiplier = Mathf.Clamp(inputPhiSpeedMultiplier, -2f, 2f);

        bool hasAnalogInput = (inputThetaSpeedMultiplier != 0f || inputPhiSpeedMultiplier != 0f);

        // 5. Apply the variable speed to the active segment
        if (hasAnalogInput)
        {
            _continuousInputTime += dt; 

            // DEBOUNCE FILTER: Ignore signals shorter than 0.05 seconds (noise)
            if (_continuousInputTime > 0.05f) 
            {
                _timeSinceLastInput = 0f; // Reset the auto-return delay timer

                float frameThetaMovement = inputThetaSpeedMultiplier * maxBendSpeedDegPerSec * dt;
                float framePhiMovement   = inputPhiSpeedMultiplier * maxPhiSpeedDegPerSec * dt;

                if (activeSegment == 1)
                {
                    theta1Deg = Mathf.Clamp(theta1Deg + frameThetaMovement, -360f, 360f);
                    phi1Deg   = Mathf.Repeat(phi1Deg + framePhiMovement + 180f, 360f) - 180f;
                }
                else
                {
                    theta2Deg = Mathf.Clamp(theta2Deg + frameThetaMovement, -360f, 360f);
                    phi2Deg   = Mathf.Repeat(phi2Deg + framePhiMovement + 180f, 360f) - 180f;
                }
            }
        }
        else 
        {
            _continuousInputTime = 0f; 
            _timeSinceLastInput += dt; 

            // Wait until the delay has passed before auto-returning
            if (autoReturnToZero && _timeSinceLastInput >= autoReturnDelay)
            {
                // Smoothly unbend back to straight (0 degrees)
                theta1Deg = Mathf.MoveTowards(theta1Deg, 0f, returnSpeedDegPerSec * dt);
                theta2Deg = Mathf.MoveTowards(theta2Deg, 0f, returnSpeedDegPerSec * dt);

                // Spin the base back to 0 as well using the shortest path
                phi1Deg = Mathf.MoveTowardsAngle(phi1Deg, 0f, returnSpeedDegPerSec * dt);
                phi2Deg = Mathf.MoveTowardsAngle(phi2Deg, 0f, returnSpeedDegPerSec * dt);
            }
        }
    }

    public void ResetToStraightPose()
    {
        theta1Deg = 0f; phi1Deg = 0f; beta1Deg = 0f; L1 = 25f;
        theta2Deg = 0f; phi2Deg = 0f; beta2Deg = 0f; L2 = 25f;
        sensorLength = 5f;
        currentInsertionCm = 0f;
    }

    private enum KeyKind { R, Alpha1, Alpha2 }

    private static bool KeyDown(KeyKind k)
    {
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current;
        if (kb == null) return false;
        switch (k)
        {
            case KeyKind.R:      return kb.rKey.wasPressedThisFrame;
            case KeyKind.Alpha1: return kb.digit1Key.wasPressedThisFrame;
            case KeyKind.Alpha2: return kb.digit2Key.wasPressedThisFrame;
        }
        return false;
#else
        switch (k)
        {
            case KeyKind.R:      return Input.GetKeyDown(KeyCode.R);
            case KeyKind.Alpha1: return Input.GetKeyDown(KeyCode.Alpha1);
            case KeyKind.Alpha2: return Input.GetKeyDown(KeyCode.Alpha2);
        }
        return false;
#endif
    }

    private void EnsureRenderers()
    {
        if (_meshFilter == null)
        {
            _meshFilter = GetComponent<MeshFilter>();
            if (_meshFilter == null) _meshFilter = gameObject.AddComponent<MeshFilter>();
        }
        if (_meshRenderer == null)
        {
            _meshRenderer = GetComponent<MeshRenderer>();
            if (_meshRenderer == null) _meshRenderer = gameObject.AddComponent<MeshRenderer>();
        }
        if (_meshRenderer.sharedMaterial == null)
            _meshRenderer.sharedMaterial = tubeMaterial != null ? tubeMaterial : DefaultMaterial();
        else if (tubeMaterial != null && _meshRenderer.sharedMaterial != tubeMaterial)
            _meshRenderer.sharedMaterial = tubeMaterial;
    }

    private void EnsureDiskMarkers(int count)
    {
        if (_diskMarkers != null && _diskMarkers.Length == count) return;
        _diskMarkers = new Transform[count];
        for (int i = 0; i < count; i++)
        {
            string nm = "SR_Disk_" + i;
            Transform t = transform.Find(nm);
            if (t == null)
            {
                var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                go.name = nm;
                SafeDestroy(go.GetComponent<Collider>());
                go.transform.SetParent(transform, false);
                t = go.transform;
            }
            var r = t.GetComponent<MeshRenderer>();
            if (r != null) r.sharedMaterial = diskMaterial != null ? diskMaterial : DefaultMaterial();
            _diskMarkers[i] = t;
        }
    }

    private void EnsureTipMarker()
    {
        if (_tipMarker != null) return;
        Transform t = transform.Find("SR_Tip");
        if (t == null)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.name = "SR_Tip";
            SafeDestroy(go.GetComponent<Collider>());
            go.transform.SetParent(transform, false);
            t = go.transform;
        }
        var r = t.GetComponent<MeshRenderer>();
        if (r != null) r.sharedMaterial = tipMaterial != null ? tipMaterial : DefaultMaterial();
        _tipMarker = t;
    }

    private void EnsureBasePlate()
    {
        if (_basePlate != null) return;
        Transform t = transform.Find("SR_BasePlate");
        if (t == null)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            go.name = "SR_BasePlate";
            SafeDestroy(go.GetComponent<Collider>());
            go.transform.SetParent(transform, false);
            t = go.transform;
        }
        var r = t.GetComponent<MeshRenderer>();
        if (r != null)
        {
            if (basePlateMaterial != null) r.sharedMaterial = basePlateMaterial;
            else if (r.sharedMaterial == null || r.sharedMaterial.name != "SR_BasePlate_mat")
            {
                var m = DefaultMaterial();
                m.name = "SR_BasePlate_mat";
                m.color = new Color(0.3f, 0.3f, 0.34f, 1f);
                r.sharedMaterial = m;
            }
        }
        _basePlate = t;
    }

    private static Material DefaultMaterial()
    {
        Shader s = Shader.Find("Universal Render Pipeline/Lit");
        if (s == null) s = Shader.Find("Standard");
        if (s == null) s = Shader.Find("Sprites/Default");
        return new Material(s);
    }

    private static void SafeDestroy(Object o)
    {
        if (o == null) return;
        if (Application.isPlaying) Destroy(o);
        else DestroyImmediate(o);
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(TipPosition, tubeRadius * worldScale * 0.8f);
    }
}