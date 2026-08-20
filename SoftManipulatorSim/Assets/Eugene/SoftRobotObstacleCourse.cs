using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Unity port of the OBSTACLE COURSE section of PART B of
/// SoftRobot_Simulation.py (point_in_obstacle / backbone_contacts_obstacle /
/// update_obstacle and the contact-event tally in update_soft_robot).
///
/// Extends SoftRobotTargetSim through its obstacle hooks, so the target
/// simulation is inherited: use THIS component instead of SoftRobotTargetSim
/// on the "SoftRobot" GameObject.
///
/// The obstacle is a single upright, axis-aligned primitive (cylinder or
/// cuboid) the robot must be steered around. All obstacle parameters live in
/// MODEL space (right-handed Z-up, centimetres) and the contact test runs on
/// the model-space backbone -- Unity coordinates appear only at the render
/// step, exactly like the rest of the port. "Upright" (along model Z) is
/// therefore vertical (along Y) in Unity.
///
/// Contact means the VISIBLE tube touches the VISIBLE obstacle: the volume is
/// inflated by (tube radius + clearance) before testing the centreline points.
/// The contact counter increments on each rising edge (clear -> contact), not
/// per frame -- this is telemetry, not a score.
///
/// The test is analytic. No Collider or Rigidbody is involved.
///
/// CONTROLS
///   O    show / hide the obstacle (resets the contact tally on each enable)
/// </summary>
public class SoftRobotObstacleCourse : SoftRobotTargetSim
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

    /// <summary>
    /// Effective centre height along model Z. Both the analytic contact test and
    /// the rendered mesh read this, so the volume and the visible obstacle can
    /// never disagree about where the obstacle actually is.
    /// </summary>
    public float ObstacleCentreZ => restOnPlate ? obstacleHeight * 0.5f : obstacleZ;

    // Running count of times the body newly entered the obstacle volume.
    [Header("Read-only")]
    public int contactEvents = 0;

    // ------------------------------------------------------------ runtime state
    private Transform _obstacle;
    private ObstacleShape _builtShape;
    private bool _inContact;      // cached once per frame; IsBodyInContact() reads it
    private bool _wasInContact;
    private Material _clearMat;
    private Material _contactMat;

    private const string ObstacleName = "SR_Obstacle";

    // ------------------------------------------------------------------ lifecycle
    protected override void Update()
    {
        if (Application.isPlaying)
        {
            // O toggles the obstacle. enableKeyboardControl is inherited from
            // SoftRobotTargetSim, so the same switch mutes T and O together.
            if (enableKeyboardControl && ObstacleToggleKeyPressed())
            {
                obstacleActive = !obstacleActive;
                // Fresh tally each time it is switched on, mirroring the way
                // SoftRobotTargetSim resets targetsReached on a fresh start.
                if (obstacleActive) contactEvents = 0;
            }

            // Mirror update_soft_robot(): test contact first, draw the obstacle,
            // THEN let the base run the targets (which reads IsBodyInContact()).
            _inContact = BackboneContactsObstacle();
            if (obstacleActive)
            {
                UpdateObstacleVisual();
                // Count a "contact event" only on the rising edge (clear ->
                // contact), not every frame the body stays inside.
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

        base.Update();
    }

    protected override void OnDisable()
    {
        DestroyObstacle();
        _wasInContact = false;
        base.OnDisable();
    }

    // ------------------------------------------------- point_in_obstacle (PART B)
    /// <summary>True if model-space point `p` lies inside the obstacle volume
    /// expanded by `extra` centimetres.</summary>
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
        // CUBOID: radius is the half-width in both X and Y.
        return Mathf.Abs(p.x - obstacleX) <= obstacleRadius + extra
            && Mathf.Abs(p.y - obstacleY) <= obstacleRadius + extra;
    }

    // ------------------------------------- backbone_contacts_obstacle (PART B)
    /// <summary>Does ANY backbone point lie inside the obstacle volume inflated
    /// by (tube radius + clearance)? The inflation makes "contact" mean the
    /// visible tube touches the visible obstacle, not just the centreline.</summary>
    private bool BackboneContactsObstacle()
    {
        if (!obstacleActive || _robot == null) return false;
        var pts = _robot.BackboneMatlab;
        if (pts == null) return false;

        float margin = _robot.tubeRadius + contactClearance;
        for (int i = 0; i < pts.Count; i++)
            if (PointInObstacle(pts[i], margin)) return true;
        return false;
    }

    // ------------------------------------------- SoftRobotTargetSim hooks
    /// <summary>Target sampling rejects candidates inside the obstacle (the base
    /// passes extra = targetRadius so targets stay clear by their own size).</summary>
    protected override bool IsPointBlocked(Vector3 pointMatlab, float extra = 0f)
        => obstacleActive && PointInObstacle(pointMatlab, extra);

    /// <summary>Cached result of this frame's backbone test; with
    /// requireClearance on, the base refuses to register a reach while true.</summary>
    protected override bool IsBodyInContact() => _inContact;

    // ------------------------------------------------ update_obstacle (PART B)
    private void UpdateObstacleVisual()
    {
        // If the shape enum changed since last time, remove the old object so
        // the correct primitive (cylinder vs box) is rebuilt.
        if (_obstacle != null && _builtShape != shape)
            DestroyObstacle();

        if (_obstacle == null)
        {
            var go = GameObject.CreatePrimitive(shape == ObstacleShape.Cylinder
                ? PrimitiveType.Cylinder : PrimitiveType.Cube);
            go.name = ObstacleName;
            Destroy(go.GetComponent<Collider>());   // the contact test is analytic
            go.transform.SetParent(transform, false);
            _obstacle = go.transform;
            _builtShape = shape;
        }

        float ws = _robot.worldScale;
        _obstacle.localPosition = SoftRobotKinematics.ToUnity(
            new Vector3(obstacleX, obstacleY, ObstacleCentreZ), ws);
        // Upright along model Z == along Unity Y, which is already the height
        // axis of both primitives -- no rotation needed.
        _obstacle.localRotation = Quaternion.identity;

        float d = obstacleRadius * 2f * ws;
        _obstacle.localScale = (shape == ObstacleShape.Cylinder)
            // Unity's cylinder primitive is 2 units tall x 1 wide, so halve the height.
            ? new Vector3(d, obstacleHeight * 0.5f * ws, d)
            // The cube is 1x1x1: model X/Y half-widths map to Unity X/Z, height to Y.
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
            _clearMat = MakeMaterial(new Color(0.5f, 0.5f, 0.55f, 1f));    // neutral grey
        return _clearMat;
    }

    private Material ContactMaterial()
    {
        if (_contactMat == null)
            _contactMat = MakeMaterial(new Color(0.95f, 0.55f, 0.1f, 1f)); // amber
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
        // Note: do NOT use ?? on UnityEngine.Object -- it bypasses the == overload.
        Shader s = Shader.Find("Universal Render Pipeline/Lit");
        if (s == null) s = Shader.Find("Standard");
        if (s == null) s = Shader.Find("Sprites/Default");
        var mat = new Material(s);
        mat.color = c;
        return mat;
    }
}
