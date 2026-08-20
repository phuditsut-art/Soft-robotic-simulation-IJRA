using UnityEngine;

/// <summary>
/// Rigid insertion rod for the transsphenoidal canal scenario.
///
/// In this updated version, keyboard WASD controls have been removed. 
/// The physical hardware load cells strictly control the tilt angles directly 
/// through the Visualizer_XY_Bending_Dof_Ver1_canal script. 
/// </summary>
[RequireComponent(typeof(Visualizer_XY_Bending_Dof_Ver1_canal))]
public class SoftRobotExtender : MonoBehaviour
{
    [Header("Mode")]
    public bool canalMode = false;

    [Tooltip("Lay the whole assembly on its side while in canal mode. This rotates the GameObject ONCE, but does not tilt it dynamically.")]
    public bool horizontalInCanalMode = true;
    public Vector3 horizontalEuler = new Vector3(0f, 0f, -90f);

    [Header("Insertion (model space, centimetres)")]
    [Tooltip("Read-only. Insertion is driven strictly by the XIAO FSR hardware in the Visualizer.")]
    public float extenderDistance = 0f;
    public float rodLength = 120f;
    public bool anchorRodAtEntry = true;
    public float rodAnchorZ = -120f;

    [Header("Rod appearance")]
    [Min(0.1f)] public float rodRadius = 4f;
    public Material rodMaterial;

    private Visualizer_XY_Bending_Dof_Ver1_canal _robot;
    private Transform _plate;
    private Transform _rod;
    private const string RodName = "SR_Extender";
    private const string PlateName = "SR_BasePlate";

    private void OnEnable()
    {
        _robot = GetComponent<Visualizer_XY_Bending_Dof_Ver1_canal>();
        FindPlate();
    }

    private void OnDisable()
    {
        SetPlateVisible(true);
        DestroyRod();
        ApplyStaticOrientation(false);
        if (_robot != null) 
        { 
            _robot.currentInsertionCm = 0f; 
            _robot.tiltAngleXDeg = 0f;
            _robot.tiltAngleYDeg = 0f; 
        }
    }

    private void Update()
    {
        if (_robot == null) return;

        if (!canalMode)
        {
            SetPlateVisible(true);
            DestroyRod();
            ApplyStaticOrientation(false);
            _robot.currentInsertionCm = 0f;
            _robot.tiltAngleXDeg = 0f;
            _robot.tiltAngleYDeg = 0f;
            return;
        }

        // ---- canal mode ----
        SetPlateVisible(false);

        // Sync distance entirely from the FSR in the visualizer script
        extenderDistance = _robot.currentInsertionCm;

        // Note: We no longer handle WASD input here. 
        // The tiltAngleXDeg and tiltAngleYDeg are updated internally by _robot via load cell analog speed control.

        // Lay the whole system horizontally if required, but DO NOT apply tilt to the GameObject.
        ApplyStaticOrientation(horizontalInCanalMode);

        UpdateRod();
    }

    /// <summary>
    /// Only applies the fixed horizontal layout.
    /// Notice: tilt angles are NOT used here, preventing the cavity from moving.
    /// </summary>
    private void ApplyStaticOrientation(bool horizontal)
    {
        Quaternion want = horizontal ? Quaternion.Euler(horizontalEuler) : Quaternion.identity;
        if (transform.localRotation != want) 
            transform.localRotation = want;
    }

    // -------------------------------------------------------------- rod render
    private void UpdateRod()
    {
        if (_rod == null)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            go.name = RodName;
            var col = go.GetComponent<Collider>();
            if (col != null) { if (Application.isPlaying) Destroy(col); else DestroyImmediate(col); }
            go.transform.SetParent(transform, false);
            _rod = go.transform;

            var r = go.GetComponent<MeshRenderer>();
            if (r != null) r.sharedMaterial = rodMaterial != null ? rodMaterial : DefaultRodMaterial();
        }

        float ws = _robot.worldScale;

        // Pull the tilt directly from the Visualizer since it manages it via analog load cells now
        Vector3 ax = Quaternion.Euler(-_robot.tiltAngleXDeg, _robot.tiltAngleYDeg, 0f) * Vector3.forward;
        
        Vector3 baseM  = ax * extenderDistance;
        Vector3 entryM = anchorRodAtEntry
            ? ax * rodAnchorZ
            : ax * (extenderDistance - rodLength);

        Vector3 baseU  = SoftRobotKinematics.ToUnity(baseM,  ws);
        Vector3 entryU = SoftRobotKinematics.ToUnity(entryM, ws);

        Vector3 mid = (baseU + entryU) * 0.5f;
        float length = Vector3.Distance(baseU, entryU);

        _rod.localPosition = mid;
        
        Vector3 dir = (baseU - entryU).normalized;
        if (dir != Vector3.zero)
        {
            _rod.localRotation = Quaternion.FromToRotation(Vector3.up, dir);
        }
        
        float d = rodRadius * 2f * ws;
        _rod.localScale = new Vector3(d, length * 0.5f, d);
    }

    private void FindPlate()
    {
        if (_plate != null) return;
        Transform t = transform.Find(PlateName);
        if (t != null) _plate = t;
    }

    private void SetPlateVisible(bool on)
    {
        if (_plate == null) FindPlate();
        if (_plate != null && _plate.gameObject.activeSelf != on)
            _plate.gameObject.SetActive(on);
    }

    private void DestroyRod()
    {
        if (_rod == null) return;
        if (Application.isPlaying) Destroy(_rod.gameObject);
        else DestroyImmediate(_rod.gameObject);
        _rod = null;
    }

    private static Material DefaultRodMaterial()
    {
        Shader s = Shader.Find("Universal Render Pipeline/Lit");
        if (s == null) s = Shader.Find("Standard");
        if (s == null) s = Shader.Find("Sprites/Default");
        var m = new Material(s);
        m.color = new Color(0.7f, 0.72f, 0.78f, 1f);   
        return m;
    }
}