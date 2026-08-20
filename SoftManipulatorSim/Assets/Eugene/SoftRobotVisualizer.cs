using UnityEngine;
using System.Collections.Generic;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Unity equivalent of PART B of SoftRobot_Simulation.py.
///
/// Drives SoftRobotKinematics and rebuilds the backbone every frame as a
/// procedural tube mesh (similar to Blender's bevelled curve). Also
/// places markers at each disk frame and at the end-effector tip.
///
/// CONTROLS (mirroring the Blender build)
///   1 / 2        select active segment
///   W / S        increase / decrease theta (bend) of the active segment
///   A / D        rotate phi (bending plane) of the active segment
///   Q / E        twist beta of the active segment
///   R            reset to the MATLAB example configuration
///
[ExecuteAlways]
public class SoftRobotVisualizer : MonoBehaviour
{
    // ------------------------------------------------------------ configuration
    [Header("Segment 1 (degrees)")]
    [Range(-180f, 180f)] public float theta1Deg = 90f;
    [Range(-180f, 180f)] public float phi1Deg   = 0f;
    [Range(-180f, 180f)] public float beta1Deg  = 45f;
    [Min(0.001f)]        public float L1        = 25f;

    [Header("Segment 2 (degrees)")]
    [Range(-180f, 180f)] public float theta2Deg = 90f;
    [Range(-180f, 180f)] public float phi2Deg   = 45f;
    [Range(-180f, 180f)] public float beta2Deg  = 0f;
    [Min(0.001f)]        public float L2        = 25f;

    [Header("Chain geometry (centimetres)")]
    [Min(0f)] public float sensorLength  = SoftRobotKinematics.DefaultSensorLength;
    [Min(0f)] public float diskThickness = SoftRobotKinematics.DefaultDiskThickness;

    [Header("Rendering")]
    [Tooltip("MATLAB units -> Unity metres. The model is in centimetres, so " +
             "0.01 gives true scale: L=25 becomes a 0.25 m segment and the " +
             "whole manipulator spans ~0.61 m when straight. Increase only if " +
             "you want an oversized display model.")]
    public float worldScale = 0.01f;
    [Range(4, 200)] public int pointsPerSegment = 30;
    [Tooltip("Tube radius in centimetres (Blender used 5.5 = 11 cm diameter). Check this against the physical prototype -- it may have been chosen for on-screen visibility rather than true geometry.")]
    public float tubeRadius = 5.5f;
    [Range(3, 32)] public int tubeSides = 16;

    public Material tubeMaterial;
    public Material diskMaterial;
    public Material tipMaterial;

    public bool showDisks = true;
    public bool showEndEffector = true;

    [Header("Mounting plate (centimetres)")]
    [Tooltip("Flat disc at the robot's origin that the robot (and obstacle) stand on. Its top face is flush with model z = 0.")]
    public bool showBasePlate = true;
    [Tooltip("Plate radius in centimetres. 65 covers the full ~61 cm straight reach.")]
    [Min(1f)] public float basePlateRadius = 65f;
    [Min(0.1f)] public float basePlateThickness = 2f;
    public Material basePlateMaterial;

    [Header("Interaction")]
    public bool enableKeyboardControl = true;
    public float bendSpeedDegPerSec  = 60f;
    public float phiSpeedDegPerSec   = 90f;
    public float twistSpeedDegPerSec = 60f;
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

    /// <summary>Tip position in Unity world space -- useful for targets/collisions.</summary>
    public Vector3 TipPosition { get; private set; }

    /// <summary>Tip position in MATLAB space (right-handed Z-up, centimetres).
    /// The target simulation measures reach distances in this frame so
    /// tolerances stay in cm regardless of worldScale.</summary>
    public Vector3 TipPositionMatlab { get; private set; }

    /// <summary>Latest backbone polyline in MATLAB space (Z-up, centimetres).
    /// The obstacle course runs its contact test against these points so the
    /// port stays a direct translation of backbone_contacts_obstacle().</summary>
    public IReadOnlyList<Vector3> BackboneMatlab => _backboneMatlab;
    private List<Vector3> _backboneMatlab;

    // ------------------------------------------------------------------ lifecycle
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
            // At runtime the configuration can change every frame, so always rebuild.
            if (enableKeyboardControl) HandleInput();
            Rebuild();
        }
        else if (_needsRebuild)
        {
            // In edit mode only rebuild when an inspector value actually changed.
            _needsRebuild = false;
            Rebuild();
        }
    }

    private void OnValidate()
    {
        // Unity doesn't allow creating/destroying objects inside OnValidate, so
        // flag it and let the next Update fix.
        _needsRebuild = true;
    }

    private bool _needsRebuild;

    // ---------------------------------------------------------------- the core
    private void Rebuild()
    {
        EnsureRenderers();

        _theta[0] = theta1Deg * Mathf.Deg2Rad * 0.5f;   // MATLAB uses theta/2 as xi
        _theta[1] = theta2Deg * Mathf.Deg2Rad * 0.5f;
        _phi[0]   = phi1Deg  * Mathf.Deg2Rad;
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
        _backboneMatlab = backboneMatlab;

        // Convert the whole polyline into Unity space (Y-up, left-handed).
        var pts = new List<Vector3>(backboneMatlab.Count);
        for (int i = 0; i < backboneMatlab.Count; i++)
            pts.Add(SoftRobotKinematics.ToUnity(backboneMatlab[i], worldScale));

        BuildTubeMesh(pts, tubeRadius * worldScale);

        // Disk frame markers.
        if (showDisks)
        {
            EnsureDiskMarkers(diskFrames.Length);
            for (int i = 0; i < diskFrames.Length; i++)
            {
                _diskMarkers[i].gameObject.SetActive(true);

                Quaternion frameRot = SoftRobotKinematics.ToUnityRotation(diskFrames[i]);
                Vector3 axis = frameRot * Vector3.forward;          // backbone tangent
                float thick = diskThickness * worldScale;

                // TF[i] sits at the FAR face of the disk (T_disk already applied),
                // so step back half a thickness to centre the marker on the disk.
                Vector3 farFace = SoftRobotKinematics.ToUnity(
                    SoftRobotKinematics.GetPosition(diskFrames[i]), worldScale);
                _diskMarkers[i].localPosition = farFace - axis * (thick * 0.5f);

                // Unity's cylinder primitive runs along its local Y and is 2 units
                // tall by 1 unit wide, so rotate Y onto the tangent and halve the height.
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

        // End-effector tip.
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

        // Mounting plate. The robot's base frame sits at the model origin, so a
        // flat disc there reads as the bench both the robot and the obstacle
        // are bolted to. Model Z is Unity local Y: the cylinder primitive's
        // height axis is already the plate normal, no rotation needed.
        if (showBasePlate)
        {
            EnsureBasePlate();
            _basePlate.gameObject.SetActive(true);
            float thick = basePlateThickness * worldScale;
            // Centre half a thickness below the origin: top face flush with z = 0.
            _basePlate.localPosition = new Vector3(0f, -thick * 0.5f, 0f);
            _basePlate.localRotation = Quaternion.identity;
            float d = basePlateRadius * worldScale * 2f;
            _basePlate.localScale = new Vector3(d, thick * 0.5f, d);   // cylinder is 2 units tall
        }
        else if (_basePlate != null)
        {
            _basePlate.gameObject.SetActive(false);
        }
    }

    // --------------------------------------------------- procedural tube mesh
    /// <summary>
    /// Sweeps a circular cross-section along the backbone using parallel-transport
    /// frames, which avoids the sudden flips a naive LookRotation would produce.
    /// This is the Unity stand-in for Blender's bevelled curve.
    /// </summary>
    private void BuildTubeMesh(List<Vector3> path, float radius)
    {
        // Remove duplicate consecutive points -- they would give zero-length tangents.
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

        // Seed an arbitrary normal perpendicular to the first tangent.
        Vector3 t0 = (p[1] - p[0]).normalized;
        Vector3 refUp = Mathf.Abs(Vector3.Dot(t0, Vector3.up)) > 0.9f ? Vector3.right : Vector3.up;
        Vector3 n = Vector3.Normalize(Vector3.Cross(refUp, t0));
        Vector3 prevTangent = t0;

        for (int i = 0; i < rings; i++)
        {
            Vector3 tangent = (i == 0)          ? (p[1] - p[0]).normalized
                            : (i == rings - 1)  ? (p[i] - p[i - 1]).normalized
                                                : (p[i + 1] - p[i - 1]).normalized;

            // Parallel transport: rotate the reference normal by the same rotation
            // that carries the previous tangent onto this one.
            Quaternion swing = Quaternion.FromToRotation(prevTangent, tangent);
            n = swing * n;
            n = Vector3.Normalize(n - Vector3.Dot(n, tangent) * tangent);   // re-orthogonalise
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

        if (KeyDown(KeyKind.Alpha1)) activeSegment = 1;
        if (KeyDown(KeyKind.Alpha2)) activeSegment = 2;
        if (KeyDown(KeyKind.R))      ResetToMatlabExample();

        float dTheta = (Held(KeyKind.W) ? 1f : 0f) - (Held(KeyKind.S) ? 1f : 0f);
        float dPhi   = (Held(KeyKind.D) ? 1f : 0f) - (Held(KeyKind.A) ? 1f : 0f);
        float dBeta  = (Held(KeyKind.E) ? 1f : 0f) - (Held(KeyKind.Q) ? 1f : 0f);

        if (activeSegment == 1)
        {
            theta1Deg = Mathf.Clamp(theta1Deg + dTheta * bendSpeedDegPerSec * dt, -180f, 180f);
            phi1Deg   = Mathf.Repeat(phi1Deg + dPhi * phiSpeedDegPerSec * dt + 180f, 360f) - 180f;
            beta1Deg  = Mathf.Repeat(beta1Deg + dBeta * twistSpeedDegPerSec * dt + 180f, 360f) - 180f;
        }
        else
        {
            theta2Deg = Mathf.Clamp(theta2Deg + dTheta * bendSpeedDegPerSec * dt, -180f, 180f);
            phi2Deg   = Mathf.Repeat(phi2Deg + dPhi * phiSpeedDegPerSec * dt + 180f, 360f) - 180f;
            beta2Deg  = Mathf.Repeat(beta2Deg + dBeta * twistSpeedDegPerSec * dt + 180f, 360f) - 180f;
        }
    }

    public void ResetToMatlabExample()
    {
        theta1Deg = 90f; phi1Deg = 0f;  beta1Deg = 45f; L1 = 25f;
        theta2Deg = 90f; phi2Deg = 45f; beta2Deg = 0f;  L2 = 25f;
        sensorLength = 5f;
    }

    // Small abstraction so the script compiles under either input backend.
    private enum KeyKind { W, A, S, D, Q, E, R, Alpha1, Alpha2 }

    private static bool Held(KeyKind k)
    {
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current;
        if (kb == null) return false;
        switch (k)
        {
            case KeyKind.W: return kb.wKey.isPressed;
            case KeyKind.A: return kb.aKey.isPressed;
            case KeyKind.S: return kb.sKey.isPressed;
            case KeyKind.D: return kb.dKey.isPressed;
            case KeyKind.Q: return kb.qKey.isPressed;
            case KeyKind.E: return kb.eKey.isPressed;
            case KeyKind.R: return kb.rKey.isPressed;
            case KeyKind.Alpha1: return kb.digit1Key.isPressed;
            case KeyKind.Alpha2: return kb.digit2Key.isPressed;
        }
        return false;
#else
        switch (k)
        {
            case KeyKind.W: return Input.GetKey(KeyCode.W);
            case KeyKind.A: return Input.GetKey(KeyCode.A);
            case KeyKind.S: return Input.GetKey(KeyCode.S);
            case KeyKind.D: return Input.GetKey(KeyCode.D);
            case KeyKind.Q: return Input.GetKey(KeyCode.Q);
            case KeyKind.E: return Input.GetKey(KeyCode.E);
            case KeyKind.R: return Input.GetKey(KeyCode.R);
            case KeyKind.Alpha1: return Input.GetKey(KeyCode.Alpha1);
            case KeyKind.Alpha2: return Input.GetKey(KeyCode.Alpha2);
        }
        return false;
#endif
    }

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

    // ------------------------------------------------------- object plumbing
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
                t = go.transform;   // orientation is set every frame in Rebuild()

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
                m.color = new Color(0.3f, 0.3f, 0.34f, 1f);   // dark bench grey
                r.sharedMaterial = m;
            }
        }
        _basePlate = t;
    }

    private static Material DefaultMaterial()
    {
        // Note: do NOT use ?? here. UnityEngine.Object overloads ==, and the
        // null-coalescing operator bypasses that overload.
        Shader s = Shader.Find("Universal Render Pipeline/Lit");
        if (s == null) s = Shader.Find("Standard");
        if (s == null) s = Shader.Find("Sprites/Default");
        return new Material(s);
    }

    /// <summary>DestroyImmediate is illegal at runtime; Destroy is illegal in edit mode.</summary>
    private static void SafeDestroy(Object o)
    {
        if (o == null) return;
        if (Application.isPlaying) Destroy(o);
        else DestroyImmediate(o);
    }

    // Handy in the editor: draw the backbone even when not playing.
    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(TipPosition, tubeRadius * worldScale * 0.8f);
    }
}
