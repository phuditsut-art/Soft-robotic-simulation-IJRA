using UnityEngine;
using System.Collections.Generic;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Unity port of the OBSTACLE COURSE section of PART B of
/// SoftRobot_Simulation.py.
///
/// Universal Version: Inherits Reflection tools from Targetsim_Phudit_Universal 
/// to automatically detect ANY active Visualizer script.
/// </summary>
public class OBSTACLE_Phudit_Universal : Targetsim_Phudit_Universal
{
    public enum ObstacleShape { Cylinder, Cuboid }

    // ------------------------------------------------------------ configuration
    [Header("Obstacle (model space, centimetres)")]
    public bool obstacleActive = false;
    public ObstacleShape shape = ObstacleShape.Cylinder;
    [Tooltip("Centre of the obstacle in the model's XY plane (cm).")]
    public float obstacleX = 15f;
    public float obstacleY = 0f;
    [Tooltip("Keep the obstacle's base on the mounting plate. When on, the centre " +
             "height is DERIVED as height/2 so the base stays at model z = 0 even " +
             "if the height changes, and Obstacle Z below is ignored.")]
    public bool restOnPlate = true;

    [Tooltip("CENTRE height along model Z (cm) -- Unity's vertical. Ignored while " +
             "Rest On Plate is on. The Blender source treats this as a free " +
             "coordinate, which is why it is still here.")]
    public float obstacleZ = 25f;
    [Tooltip("Cylinder radius, or cuboid half-width in both X and Y (cm).")]
    [Min(0.1f)] public float obstacleRadius = 6f;
    [Tooltip("FULL extent along model Z, centred on obstacle z (cm).")]
    [Min(0.1f)] public float obstacleHeight = 30f;
    [Tooltip("Extra safety gap added to the tube radius when testing contact (cm).")]
    [Min(0f)] public float contactClearance = 0f;

    public float ObstacleCentreZ => restOnPlate ? obstacleHeight * 0.5f : obstacleZ;

    [Header("Read-only")]
    public int contactEvents = 0;

    // ------------------------------------------------------------ runtime state
    private Transform _obstacle;
    private ObstacleShape _builtShape;
    private bool _inContact;      
    private bool _wasInContact;
    private Material _clearMat;
    private Material _contactMat;

    private const string ObstacleName = "SR_Obstacle";

    // ------------------------------------------------------------------ lifecycle

    protected override void Update()
    {
        if (Application.isPlaying)
        {
            if (enableKeyboardControl && ObstacleToggleKeyPressed())
            {
                obstacleActive = !obstacleActive;
                if (obstacleActive) contactEvents = 0;
            }

            _inContact = BackboneContactsObstacle();
            if (obstacleActive)
            {
                UpdateObstacleVisual();
                if (_inContact && !_wasInContact)
                {
                    contactEvents++;
                    Debug.Log($"[sim] body entered obstacle volume; contact events = {contactEvents}");
                }
                _wasInContact = _inContact;
            }
            else
            {
                DestroyObstacle();
                _wasInContact = false;
            }
        }

        // base.Update() handles the target simulation AND finding the active visualizer!
        base.Update();
    }

    protected override void OnDisable()
    {
        DestroyObstacle();
        _wasInContact = false;
        base.OnDisable();
    }

    // ------------------------------------------------- point_in_obstacle (PART B)
    private bool PointInObstacle(Vector3 p, float extra)
    {
        float dz = Mathf.Abs(p.z - ObstacleCentreZ);
        if (dz > obstacleHeight * 0.5f + extra) return false;

        if (shape == ObstacleShape.Cylinder)
        {
            float dx = p.x - obstacleX;
            float dy = p.y - obstacleY;
            return Mathf.Sqrt(dx * dx + dy * dy) <= obstacleRadius + extra;
        }
        
        return Mathf.Abs(p.x - obstacleX) <= obstacleRadius + extra
            && Mathf.Abs(p.y - obstacleY) <= obstacleRadius + extra;
    }

    // ------------------------------------- backbone_contacts_obstacle (PART B)
    private bool BackboneContactsObstacle()
    {
        // _activeVisualizer is inherited from Targetsim_Phudit_Universal
        if (!obstacleActive || _activeVisualizer == null) return false;
        
        // GetValue is inherited from Targetsim_Phudit_Universal
        IReadOnlyList<Vector3> pts = GetValue<IReadOnlyList<Vector3>>("BackboneMatlab");
        float radius = GetValue<float>("tubeRadius", 5.5f);

        if (pts == null) return false;

        float margin = radius + contactClearance;
        for (int i = 0; i < pts.Count; i++)
        {
            if (PointInObstacle(pts[i], margin)) return true;
        }
        
        return false;
    }

    // ------------------------------------------- Targetsim_Phudit_Universal hooks
    protected override bool IsPointBlocked(Vector3 pointMatlab, float extra = 0f)
        => obstacleActive && PointInObstacle(pointMatlab, extra);

    protected override bool IsBodyInContact() => _inContact;

    // ------------------------------------------------ update_obstacle (PART B)
    private void UpdateObstacleVisual()
    {
        if (_obstacle != null && _builtShape != shape)
            DestroyObstacle();

        if (_obstacle == null)
        {
            var go = GameObject.CreatePrimitive(shape == ObstacleShape.Cylinder
                ? PrimitiveType.Cylinder : PrimitiveType.Cube);
            go.name = ObstacleName;
            Destroy(go.GetComponent<Collider>());
            go.transform.SetParent(transform, false);
            _obstacle = go.transform;
            _builtShape = shape;
        }

        // GetValue is inherited from Targetsim_Phudit_Universal
        float ws = GetValue<float>("worldScale", 0.01f);

        _obstacle.localPosition = SoftRobotKinematics.ToUnity(
            new Vector3(obstacleX, obstacleY, ObstacleCentreZ), ws);
        
        _obstacle.localRotation = Quaternion.identity;

        float d = obstacleRadius * 2f * ws;
        _obstacle.localScale = (shape == ObstacleShape.Cylinder)
            ? new Vector3(d, obstacleHeight * 0.5f * ws, d)
            : new Vector3(d, obstacleHeight * ws, d);

        var r = _obstacle.GetComponent<MeshRenderer>();
        if (r != null)
            r.sharedMaterial = _inContact ? ContactMaterial() : ClearMaterial();
    }

    // ------------------------------------------------------- object plumbing
    private void DestroyObstacle()
    {
        if (_obstacle == null) return;
        if (Application.isPlaying) Destroy(_obstacle.gameObject);
        else DestroyImmediate(_obstacle.gameObject);
        _obstacle = null;
    }

    private Material ClearMaterial()
    {
        if (_clearMat == null)
            _clearMat = MakeMaterial(new Color(0.5f, 0.5f, 0.55f, 1f));
        return _clearMat;
    }

    private Material ContactMaterial()
    {
        if (_contactMat == null)
            _contactMat = MakeMaterial(new Color(0.95f, 0.55f, 0.1f, 1f));
        return _contactMat;
    }

    // -------------------------------------------------------------- input
    private static bool ObstacleToggleKeyPressed()
    {
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current;
        return kb != null && kb.oKey.wasPressedThisFrame;
#else
        return Input.GetKeyDown(KeyCode.O);
#endif
    }

    private static Material MakeMaterial(Color c)
    {
        Shader s = Shader.Find("Universal Render Pipeline/Lit");
        if (s == null) s = Shader.Find("Standard");
        if (s == null) s = Shader.Find("Sprites/Default");
        var mat = new Material(s);
        mat.color = c;
        return mat;
    }
}