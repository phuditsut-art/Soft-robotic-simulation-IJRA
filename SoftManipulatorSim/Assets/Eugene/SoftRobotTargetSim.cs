using UnityEngine;
using System.Collections.Generic;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Unity port of the TARGET SIMULATION section of PART B of
/// SoftRobot_Simulation.py (random_reachable_tip / update_targets and the
/// Start/Stop operators).
///
/// Green spheres appear at random reachable points in the workspace. Steer the
/// tip to a target; getting within the reach tolerance registers it and a new
/// target is placed. Positions come from forward kinematics on a random valid
/// joint configuration (the same idea as Example_SoftManipulator_model.m), so
/// every target is guaranteed reachable.
///
/// SETUP: add this next to SoftRobotVisualizer on the "SoftRobot" GameObject
/// and press Play. Targets only run in Play mode -- in the editor the robot
/// still renders via the visualizer's [ExecuteAlways], but random targets
/// churning in the Scene view would just be noise.
///
/// CONTROLS
///   T    start / stop the target simulation (resets the reached tally on start)
///
/// The obstacle-course hooks (IsPointBlocked / IsBodyInContact) mirror the
/// Python structure and are overridden by SoftRobotObstacleCourse: targets
/// are rejected inside the obstacle volume, and "target needs clear path"
/// refuses to register a reach while the body is in contact. Use that
/// subclass instead of this component when you want the obstacle.
/// </summary>
[RequireComponent(typeof(SoftRobotVisualizer))]
public class SoftRobotTargetSim : MonoBehaviour
{
    // ------------------------------------------------------------ configuration
    [Header("Target simulation")]
    public bool simulationActive = false;
    [Range(1, 10)] public int targetCount = 1;
    [Tooltip("Visual sphere radius in centimetres (Blender default 3).")]
    [Min(0.1f)] public float targetRadius = 3f;
    [Tooltip("Tip-to-target distance in centimetres that counts as reached (Blender default 8).")]
    [Min(0.1f)] public float reachTolerance = 8f;

    [Tooltip("If true, a target only registers as reached while the robot body is clear of the obstacle. No effect until the obstacle course is ported.")]
    public bool requireClearance = true;

    public Material targetMaterial;

    [Header("Timer Challenge")]
    [Tooltip("Enable to time how long it takes to reach a specific number of targets.")]
    public bool useTimerChallenge = false;
    [Tooltip("Number of targets you want to reach to stop the timer.")]
    [Min(1)] public int targetGoal = 5;

    // Neutral running tally of how many targets the tip has reached (not a game score).
    [Header("Read-only")]
    public int targetsReached = 0;
    
    [Tooltip("Time elapsed since the simulation started.")]
    public float elapsedTime = 0f;
    public bool isChallengeComplete = false;

    [Header("Interaction")]
    public bool enableKeyboardControl = true;

    // ------------------------------------------------------------ runtime state
    protected SoftRobotVisualizer _robot;

    // Target positions live in MATLAB space (Z-up, centimetres) like the
    // Python `_targets` list; they are converted to Unity only when the
    // marker transforms are updated.
    private readonly List<Vector3> _targets = new List<Vector3>();
    private readonly List<Transform> _markers = new List<Transform>();

    private const string TargetPrefix = "SR_Target_";
    private bool _wasActive;
    private bool _timerRunning = false;

    // ------------------------------------------------------------------ lifecycle
    // virtual so SoftRobotObstacleCourse can extend them; a subclass declaring
    // its own Update/OnEnable/OnDisable would silently SHADOW these otherwise.
    protected virtual void OnEnable()
    {
        _robot = GetComponent<SoftRobotVisualizer>();
    }

    protected virtual void OnDisable()
    {
        ClearTargets();
        _wasActive = false;
        _timerRunning = false;
    }

    protected virtual void Update()
    {
        if (!Application.isPlaying) return;

        if (enableKeyboardControl && ToggleKeyPressed())
            simulationActive = !simulationActive;

        // SR_OT_StartTargetSim resets the tally on every fresh start.
        if (simulationActive && !_wasActive)
        {
            targetsReached = 0;
            elapsedTime = 0f;
            isChallengeComplete = false;
            _timerRunning = true;
            _targets.Clear();
            Debug.Log(useTimerChallenge ? $"[sim] Timer started! Goal: {targetGoal} targets." : "[sim] Target simulation started.");
        }
        
        _wasActive = simulationActive;

        // Run the timer if the simulation is active and the challenge isn't over yet
        if (simulationActive && _timerRunning && (!useTimerChallenge || !isChallengeComplete))
        {
            elapsedTime += Time.deltaTime;
        }

        if (simulationActive) UpdateTargets();
        else ClearTargets();
    }

    // --------------------------------------------------- ON-SCREEN UI
    private void OnGUI()
    {
        if (!Application.isPlaying) return;

        // Only draw the UI if the simulation is running or we just finished the challenge
        if (simulationActive || isChallengeComplete)
        {
            // Set up font styling for the overlay
            GUIStyle style = new GUIStyle();
            style.fontSize = 24;
            style.fontStyle = FontStyle.Bold;
            style.normal.textColor = Color.white;

            // Draw a subtle dark background box for readability
            GUI.Box(new Rect(10, 10, 300, 100), "");

            // Display time
            string timeText = $"Time: {elapsedTime:F2} s";
            GUI.Label(new Rect(20, 20, 280, 30), timeText, style);

            // Display score
            string scoreText = useTimerChallenge 
                ? $"Targets: {targetsReached} / {targetGoal}" 
                : $"Targets Reached: {targetsReached}";
            GUI.Label(new Rect(20, 50, 280, 30), scoreText, style);

            // If challenge is complete, show a big green success message
            if (useTimerChallenge && isChallengeComplete)
            {
                GUIStyle successStyle = new GUIStyle();
                successStyle.fontSize = 32;
                successStyle.fontStyle = FontStyle.Bold;
                successStyle.normal.textColor = Color.green;
                
                GUI.Label(new Rect(20, 120, 400, 50), "CHALLENGE COMPLETE!", successStyle);
            }
        }
    }

    // ---------------------------------------------- random_reachable_tip (PART B)
    /// <summary>
    /// Sample a random valid joint config and return its tip in MATLAB space
    /// (always reachable). If an obstacle is active, resample up to
    /// <paramref name="maxTries"/> times so the target is not placed inside the
    /// obstacle volume; falls back to the last sample if all attempts fail.
    /// </summary>
    public Vector3 RandomReachableTip(bool avoidObstacle = true, int maxTries = 25)
    {
        // Same sampling ranges as the Python build (and the MATLAB example
        // script): theta is HALF the bend angle, hence the extra * 0.5.
        var th = new float[2];
        var ph = new float[2];
        var bt = new float[2];
        var len = new float[] { _robot.L1, _robot.L2 };

        Vector3 tip = Vector3.zero;
        for (int attempt = 0; attempt < maxTries; attempt++)
        {
            for (int i = 0; i < 2; i++)
            {
                th[i] = Random.Range(0f, 90f) * Mathf.Deg2Rad * 0.5f;
                ph[i] = Random.Range(-180f, 180f) * Mathf.Deg2Rad;
                bt[i] = Random.Range(-45f, 45f) * Mathf.Deg2Rad;
            }

            SoftRobotKinematics.FwdSRM(
                th, ph, bt, len,
                Vector3.zero, Vector3.zero,
                out Matrix4x4 tipT, out _,
                _robot.sensorLength, _robot.diskThickness);
            tip = SoftRobotKinematics.GetPosition(tipT);

            if (!avoidObstacle) return tip;
            // Keep the target clear of the obstacle by at least its own radius.
            if (!IsPointBlocked(tip, extra: targetRadius)) return tip;
        }
        return tip;
    }

    // --------------------------------------------------- update_targets (PART B)
    /// <summary>
    /// Keep `targetCount` targets alive; register any the tip reaches; redraw.
    /// If "requireClearance" is on, a reach only registers while the body is
    /// clear of the obstacle -- this is what forces maneuvering AROUND the
    /// obstacle rather than pushing through it (inactive until that port lands).
    /// </summary>
    private void UpdateTargets()
    {
        // If the challenge is complete, don't spawn new targets
        if (useTimerChallenge && isChallengeComplete)
        {
            ClearTargets();
            return;
        }

        while (_targets.Count < targetCount) _targets.Add(RandomReachableTip());
        while (_targets.Count > targetCount) _targets.RemoveAt(_targets.Count - 1);

        Vector3 tip = _robot.TipPositionMatlab;
        bool blocked = requireClearance && IsBodyInContact();
        for (int i = 0; i < _targets.Count; i++)
        {
            if (Vector3.Distance(tip, _targets[i]) > reachTolerance) continue;
            if (blocked)
            {
                // Tip is at the target but the body is inside the obstacle:
                // do not register this reach; the path was not clear.
                continue;
            }
            
            _targets[i] = RandomReachableTip();
            targetsReached++;
            
            if (useTimerChallenge && targetsReached >= targetGoal)
            {
                _timerRunning = false;
                isChallengeComplete = true;
                Debug.Log($"<color=green>[sim] CHALLENGE COMPLETE! Reached {targetGoal} targets in {elapsedTime:F2} seconds.</color>");
            }
            else
            {
                Debug.Log($"[sim] target reached; total reached = {targetsReached}");
            }
        }

        RedrawMarkers();
    }

    // -------------------------------------------------------- obstacle hooks
    // Stubs mirroring point_in_obstacle / the contact test in the Python build.
    // The obstacle-course port overrides these; until then nothing is blocked.
    protected virtual bool IsPointBlocked(Vector3 pointMatlab, float extra = 0f) => false;
    protected virtual bool IsBodyInContact() => false;

    // ------------------------------------------------------- object plumbing
    private void RedrawMarkers()
    {
        while (_markers.Count < _targets.Count)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.name = TargetPrefix + _markers.Count;
            Destroy(go.GetComponent<Collider>());
            go.transform.SetParent(transform, false);

            var r = go.GetComponent<MeshRenderer>();
            if (r != null)
                r.sharedMaterial = targetMaterial != null ? targetMaterial : DefaultTargetMaterial();
            _markers.Add(go.transform);
        }
        while (_markers.Count > _targets.Count)
        {
            var last = _markers[_markers.Count - 1];
            _markers.RemoveAt(_markers.Count - 1);
            if (last != null) Destroy(last.gameObject);
        }

        float scale = _robot.worldScale;
        for (int i = 0; i < _markers.Count; i++)
        {
            _markers[i].localPosition = SoftRobotKinematics.ToUnity(_targets[i], scale);
            float d = targetRadius * scale * 2f;   // sphere primitive is 1 unit in diameter
            _markers[i].localScale = new Vector3(d, d, d);
        }
    }

    private void ClearTargets()
    {
        _targets.Clear();
        foreach (var m in _markers)
            if (m != null)
            {
                if (Application.isPlaying) Destroy(m.gameObject);
                else DestroyImmediate(m.gameObject);
            }
        _markers.Clear();
    }

    private static Material DefaultTargetMaterial()
    {
        // Note: do NOT use ?? on UnityEngine.Object -- it bypasses the == overload.
        Shader s = Shader.Find("Universal Render Pipeline/Lit");
        if (s == null) s = Shader.Find("Standard");
        if (s == null) s = Shader.Find("Sprites/Default");
        var mat = new Material(s);
        mat.color = new Color(0.1f, 0.9f, 0.2f, 1f);   // SR_target_mat green
        return mat;
    }

    // -------------------------------------------------------------- input
    private static bool ToggleKeyPressed()
    {
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current;
        return kb != null && kb.tKey.wasPressedThisFrame;
#else
        return Input.GetKeyDown(KeyCode.T);
#endif
    }
}