using UnityEngine;
using System.Collections.Generic;
using System.IO.Ports;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Unity equivalent of PART B of SoftRobot_Simulation.py.
/// Uses an On/Off Switch elevation system for the FSR.
/// </summary>
[ExecuteAlways]
public class Visualizer_ver2_positioncontrol : MonoBehaviour
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

    [Header("Interaction (Load Cell Control)")]
    public bool useArduinoControl = true; 
    public ArduinoSerialReader_Float serialReader; 
    
    [Header("Position Control Settings")]
    public float maxBendAngle = 180f;  
    public float maxSpinAngle = 180f;  
    public float positionMoveSpeed = 150f;
    public float maxSensorSignal = 1200f; 

    [Header("XIAO Serial Control (Segment + FSR)")]
    public bool useXiaoSlider = true;
    public int xiaoPortNumber = 8;
    public int xiaoBaudRate = 9600;
    private SerialPort _xiaoStream;

    [Header("Elevation Control (On/Off Switch)")]
    [Tooltip("The FSR reading needed to trigger elevation. Used 1000 instead of 1023 to prevent analog flickering.")]
    public float triggerThreshold = 1000f;
    [Tooltip("How fast the robot climbs upward (cm per second) when button is held")]
    public float elevationSpeed = 15f;
    [Tooltip("Maximum height it can reach so it doesn't fly away infinitely")]
    public float maxElevationLimit = 50f;
    [Tooltip("How fast it falls back to 0 (cm per second) when button is released")]
    public float retractSpeed = 20f;
    
    [Tooltip("Read-only view of the current elevation depth")]
    public float currentInsertionCm = 0f;

    [Header("Active State")]
    [Range(0, 2)] public int activeSegment = 1;

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

    public Vector3 TipPosition { get; private set; }
    public Vector3 TipPositionMatlab { get; private set; }
    public IReadOnlyList<Vector3> BackboneMatlab => _backboneMatlab;
    private List<Vector3> _backboneMatlab;
    private bool _needsRebuild;
    private float _currentFsrReading = 0f; 

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

        float rad = phi1Deg * Mathf.Deg2Rad;
        float cosP = Mathf.Cos(rad);
        float sinP = Mathf.Sin(rad);

        for (int i = 0; i < backboneMatlab.Count; i++)
        {
            Vector3 p = backboneMatlab[i];
            p.z += currentInsertionCm; 
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

                Vector3 farFace = SoftRobotKinematics.ToUnity(SoftRobotKinematics.GetPosition(diskFrames[i]), worldScale);
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

        // 1. Read Serial Data
        if (useXiaoSlider && _xiaoStream != null && _xiaoStream.IsOpen)
        {
            try
            {
                if (_xiaoStream.BytesToRead > 0)
                {
                    string incoming = _xiaoStream.ReadLine().Trim();
                    string[] parts = incoming.Split(',');

                    if (parts.Length == 2)
                    {
                        if (int.TryParse(parts[0], out int sliderState))
                        {
                            if (sliderState >= 0 && sliderState <= 2) activeSegment = sliderState;
                        }

                        if (float.TryParse(parts[1], out float fsr))
                        {
                            _currentFsrReading = fsr; // Save reading for the switch math below
                        }
                    }
                }
            }
            catch (System.TimeoutException) { /* Ignore read timeouts */ }
            catch (System.Exception e) { Debug.LogWarning("XIAO Serial Error: " + e.Message); }
        }

        // 2. On/Off Switch Elevation Math
        if (_currentFsrReading >= triggerThreshold)
        {
            // SWITCH ON: Elevate steadily until hitting the max limit
            currentInsertionCm = Mathf.MoveTowards(currentInsertionCm, maxElevationLimit, elevationSpeed * dt);
        }
        else 
        {
            // SWITCH OFF: Retract steadily back down to 0
            currentInsertionCm = Mathf.MoveTowards(currentInsertionCm, 0f, retractSpeed * dt);
        }

        // 3. Keyboard overrides
        if (KeyDown(KeyKind.Alpha1)) activeSegment = 1;
        if (KeyDown(KeyKind.Alpha2)) activeSegment = 2;
        if (KeyDown(KeyKind.R))      ResetToStraightPose();

        float inputTheta = 0f;
        float inputPhi   = 0f;

        // 4. Load cell proportional reading
        if (serialReader != null)
        {
            inputTheta += (serialReader.loadCellUpValue / maxSensorSignal);
            inputTheta -= (serialReader.loadCellDownValue / maxSensorSignal);

            inputPhi += (serialReader.loadCellRightValue / maxSensorSignal);
            inputPhi -= (serialReader.loadCellLeftValue / maxSensorSignal);
        }

        float deadzone = 0.02f;
        if (Mathf.Abs(inputTheta) < deadzone) inputTheta = 0f;
        if (Mathf.Abs(inputPhi) < deadzone)   inputPhi = 0f;

        inputTheta = Mathf.Clamp(inputTheta, -1f, 1f);
        inputPhi   = Mathf.Clamp(inputPhi, -1f, 1f);

        float targetTheta = inputTheta * maxBendAngle;
        float targetPhi   = inputPhi * maxSpinAngle;

        // 5. Apply bending to the active segment
        if (activeSegment == 1)
        {
            theta1Deg = Mathf.MoveTowards(theta1Deg, targetTheta, positionMoveSpeed * dt);
            phi1Deg   = Mathf.MoveTowardsAngle(phi1Deg, targetPhi, positionMoveSpeed * dt);
        }
        else if (activeSegment == 2)
        {
            theta2Deg = Mathf.MoveTowards(theta2Deg, targetTheta, positionMoveSpeed * dt);
            phi2Deg   = Mathf.MoveTowardsAngle(phi2Deg, targetPhi, positionMoveSpeed * dt);
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