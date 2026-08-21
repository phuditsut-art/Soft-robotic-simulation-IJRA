using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Analytic wall-contact detection for the nasal cavity canal.
/// </summary>
[ExecuteAlways]
[RequireComponent(typeof(Visualizer_Rotating_Bending_Dof_Ver1_canal))]
public class SoftRobotCanal : MonoBehaviour
{
    [Header("Active")]
    public bool canalActive = true;

    [Header("Renderer to tint")]
    public MeshRenderer canalRenderer;

    [Header("Scale")]
    [Min(0.01f)] public float meshScale = 2f;

    [Header("Lumen geometry (TRUE millimetres, before Mesh Scale)")]
    public Vector2 lumenAxisXY = new Vector2(0f, 0.9f);
    [Min(0.1f)] public float entryRadius = 11.2f;
    public float snoutEndZ = 14f;
    [Min(0.1f)] public float chamberRadius = 18.6f;
    public float chamberStartZ = 19f;
    public float canalStartZ = 0f;
    public float canalEndZ = 90f;

    [Header("Placement")]
    public bool autoPlaceFromTransform = true;
    public float canalOriginZ = -35f;
    public Vector2 canalOriginXY = Vector2.zero;

    [Header("Contact")]
    [Min(0f)] public float contactClearance = 0f;
    public Color contactColor = new Color(0.95f, 0.35f, 0.1f);

    [Header("Read-only")]
    public int contactEvents = 0;
    public bool InContact => _inContact;

    [Header("Debug")]
    [Tooltip("Spawns actual GameObjects (LineRenderers) to show the collision bounds.")]
    public bool showInvisibleWall = true;

    private static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
    private Visualizer_Rotating_Bending_Dof_Ver1_canal _robot;
    private Transform _canalRoot;
    private Material _mat;
    private Color _clearColor;
    private bool _inContact, _wasInContact, _ready;

    private GameObject _debugContainer;

    private void OnEnable()
    {
        _robot = GetComponent<Visualizer_Rotating_Bending_Dof_Ver1_canal>();
        Bind();
    }

    private void Bind()
    {
        if (canalRenderer == null)
        {
            foreach (var r in GetComponentsInChildren<MeshRenderer>(true))
            {
                string nm = r.gameObject.name.ToLower();
                string pa = r.transform.parent != null ? r.transform.parent.name.ToLower() : "";
                if (nm.Contains("cavity") || pa.Contains("cavity") || nm.Contains("nasal") || pa.Contains("nasal"))
                { canalRenderer = r; break; }
            }
        }
        
        if (canalRenderer != null)
        {
            _canalRoot = canalRenderer.transform;
            while (_canalRoot.parent != null && _canalRoot.parent != transform)
                _canalRoot = _canalRoot.parent;

            if (Application.isPlaying)
            {
                _mat = canalRenderer.material;      
                _clearColor = _mat.HasProperty(BaseColorId) ? _mat.GetColor(BaseColorId) : _mat.color;
                _ready = true;
            }
        }
    }

    private void Update()
    {
        UpdateDebugVisuals();

        if (!Application.isPlaying || _robot == null) return;
        if (!_ready) Bind();

        _inContact = canalActive && BackboneTouchesWall();

        if (canalActive)
        {
            if (_inContact && !_wasInContact)
            {
                contactEvents++;
                Debug.Log($"[sim] body touched canal wall; contact events = {contactEvents}");
            }
            _wasInContact = _inContact;
        }
        else _wasInContact = false;

        Tint();
    }

    private void OnDisable()
    {
        if (_ready && _mat != null) _mat.color = _clearColor;
        _wasInContact = false;
        
        if (_debugContainer != null)
        {
            SafeDestroy(_debugContainer);
        }
    }

    private float LumenRadiusAt(float z)
    {
        float s = meshScale;
        float zs = snoutEndZ * s, zc = chamberStartZ * s;
        float rE = entryRadius * s, rC = chamberRadius * s;

        if (z <= zs) return rE;
        if (z >= zc) return rC;
        float t = (z - zs) / Mathf.Max(1e-4f, zc - zs);
        return Mathf.Lerp(rE, rC, t);
    }

    private void SyncPlacement()
    {
        if (_robot == null) _robot = GetComponent<Visualizer_Rotating_Bending_Dof_Ver1_canal>();
        if (_robot == null) return;

        if (autoPlaceFromTransform && _canalRoot != null)
        {
            float ws = Mathf.Max(1e-6f, _robot.worldScale);
            Vector3 lp = _canalRoot.localPosition;
            canalOriginXY = new Vector2(lp.x / ws, lp.z / ws);
            canalOriginZ  = lp.y / ws;
            meshScale = Mathf.Max(0.01f, _canalRoot.localScale.x);
        }
    }

    private bool BackboneTouchesWall()
    {
        var pts = _robot.BackboneMatlab;
        if (pts == null) return false;

        SyncPlacement();
        float s = meshScale;
        float z0 = canalStartZ * s, z1 = canalEndZ * s;
        float ax = lumenAxisXY.x * s + canalOriginXY.x;
        float ay = lumenAxisXY.y * s + canalOriginXY.y;
        float margin = _robot.tubeRadius + contactClearance;

        for (int i = 0; i < pts.Count; i++)
        {
            Vector3 p = pts[i];
            float zc = p.z - canalOriginZ;
            if (zc < z0 || zc > z1) continue;

            float dx = p.x - ax, dy = p.y - ay;
            float dAxis = Mathf.Sqrt(dx * dx + dy * dy);
            if (dAxis + margin > LumenRadiusAt(zc)) return true;
        }
        return false;
    }

    private void Tint()
    {
        if (!_ready || _mat == null) return;
        Color want = _inContact ? contactColor : _clearColor;
        want.a = _clearColor.a;                
        if (_mat.color != want) _mat.color = want;
        if (_mat.HasProperty(BaseColorId)) _mat.SetColor(BaseColorId, want);
    }

    // --------------------------------------------------- GameObject Visualization
    private void UpdateDebugVisuals()
    {
        if (!showInvisibleWall)
        {
            if (_debugContainer != null) _debugContainer.SetActive(false);
            return;
        }

        if (_debugContainer == null)
        {
            Transform existing = transform.Find("SR_Canal_DebugWall");
            if (existing != null) _debugContainer = existing.gameObject;
            else
            {
                _debugContainer = new GameObject("SR_Canal_DebugWall");
                _debugContainer.transform.SetParent(transform, false);
            }
        }
        _debugContainer.SetActive(true);

        if (_robot == null) _robot = GetComponent<Visualizer_Rotating_Bending_Dof_Ver1_canal>();
        if (_robot == null) return;

        if (_canalRoot == null) Bind();
        SyncPlacement();

        float s = meshScale;
        float z0 = canalStartZ * s;
        float z1 = canalEndZ * s;
        float ax = lumenAxisXY.x * s + canalOriginXY.x;
        float ay = lumenAxisXY.y * s + canalOriginXY.y;
        float ws = _robot.worldScale;

        // 1. Draw Start Sphere
        Transform sphereTransform = _debugContainer.transform.Find("StartSphere");
        if (sphereTransform == null)
        {
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.name = "StartSphere";
            go.transform.SetParent(_debugContainer.transform, false);
            SafeDestroy(go.GetComponent<Collider>());
            var mr = go.GetComponent<MeshRenderer>();
            if (mr != null) mr.sharedMaterial = GetDebugMaterial(Color.yellow, "SR_Debug_Yellow");
            sphereTransform = go.transform;
        }

        Vector3 startMatlab = new Vector3(ax, ay, z0 + canalOriginZ);
        Vector3 startLocal = new Vector3(startMatlab.x, startMatlab.z, startMatlab.y) * ws;
        sphereTransform.localPosition = startLocal;
        sphereTransform.localScale = Vector3.one * 0.02f;

        // 2. Draw Rings using LineRenderers
        int ringCount = 20;
        float step = (z1 - z0) / ringCount;
        int segments = 24;

        List<LineRenderer> rings = new List<LineRenderer>(_debugContainer.GetComponentsInChildren<LineRenderer>(true));
        
        while (rings.Count <= ringCount)
        {
            GameObject go = new GameObject($"Ring_{rings.Count}");
            go.transform.SetParent(_debugContainer.transform, false);
            LineRenderer lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace = false;
            lr.loop = true;
            lr.startWidth = 0.003f;
            lr.endWidth = 0.003f;
            lr.sharedMaterial = GetDebugMaterial(Color.cyan, "SR_Debug_Cyan");
            rings.Add(lr);
        }

        for (int i = 0; i < rings.Count; i++)
        {
            if (i > ringCount)
            {
                rings[i].gameObject.SetActive(false);
                continue;
            }

            rings[i].gameObject.SetActive(true);
            float zc = z0 + i * step; 
            float radius = LumenRadiusAt(zc); 
            float actualZ = zc + canalOriginZ; 

            Vector3 centerMatlab = new Vector3(ax, ay, actualZ);
            Vector3 centerLocal = new Vector3(centerMatlab.x, centerMatlab.z, centerMatlab.y) * ws;
            
            rings[i].transform.localPosition = centerLocal;
            rings[i].positionCount = segments;

            for (int j = 0; j < segments; j++)
            {
                float angle = j * Mathf.PI * 2f / segments;
                // Draw circle flat on the XZ plane (which is Matlab's XY plane)
                Vector3 pt = new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle)) * (radius * ws);
                rings[i].SetPosition(j, pt);
            }
        }
    }

    private Material GetDebugMaterial(Color color, string matName)
    {
        Shader s = Shader.Find("Universal Render Pipeline/Unlit");
        if (s == null) s = Shader.Find("Unlit/Color");
        if (s == null) s = Shader.Find("Standard");

        Material m = new Material(s);
        m.name = matName;
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", color);
        if (m.HasProperty("_Color")) m.SetColor("_Color", color);
        m.color = color;
        return m;
    }

    private static void SafeDestroy(Object o)
    {
        if (o == null) return;
        if (Application.isPlaying) Destroy(o);
        else DestroyImmediate(o);
    }
}