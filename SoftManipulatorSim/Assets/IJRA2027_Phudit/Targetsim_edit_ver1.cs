using UnityEngine;
using System.Collections.Generic;
using System.Reflection;
using System.IO; // Added for CSV Logging

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Unity port of the TARGET SIMULATION section of PART B of SoftRobot_Simulation.py.
/// Universal Version with integrated Unity-Space CSV Logging.
/// </summary>
public class Targetsim_Phudit_Universal : MonoBehaviour
{
    // ------------------------------------------------------------ configuration
    [Header("Target simulation")]
    public bool simulationActive = false;
    
    [Tooltip("How many targets should appear on screen at once.")]
    [Range(1, 20)] public int targetCount = 1;
    
    [Tooltip("Visual sphere radius in centimetres (Blender default 3).")]
    [Min(0.1f)] public float targetRadius = 3f;
    
    [Tooltip("Tip-to-target distance in centimetres that counts as reached (Blender default 8).")]
    [Min(0.1f)] public float reachTolerance = 8f;

    [Tooltip("How long (in seconds) the tip must stay inside the target to register a hit.")]
    [Range(0f, 5f)] public float requiredHoldTime = 1.0f;

    [Tooltip("If true, a target only registers as reached while the robot body is clear of the obstacle.")]
    public bool requireClearance = true;

    [Header("Timer Challenge")]
    [Tooltip("Enable to time how long it takes to reach a specific number of targets.")]
    public bool useTimerChallenge = false;
    [Tooltip("Number of targets you want to reach to stop the timer.")]
    [Min(1)] public int targetGoal = 5;

    [Header("Target Generation Mode")]
    [Tooltip("If TRUE, Segment 1 is locked to the robot's current live angles, making the targets spawn centered around the second segment's joint.")]
    public bool centerOnSecondSegmentJoint = true;

    [Tooltip("Multiplies how far the target is placed from the origin. 1.0 = actual robot reach, >1.0 = further away.")]
    [Range(0.5f, 5.0f)] public float distanceMultiplier = 1.0f;

    [Header("Target Generation Ranges (Degrees)")]
    [Tooltip("CHECKED = Fisher-Yates Deck Shuffle (guarantees NO repeats until every angle is used).\nUNCHECKED = Pure Random (can pick the same angle twice).")]
    public bool useDeckShuffle = true;

    [Tooltip("If true (and Center On Joint is false), both segments bend in the exact same direction. If false, segments bend independently.")]
    public bool syncSegments = true;

    [Header("Theta (Bend)")]
    public Vector2 thetaRange = new Vector2(0f, 90f);
    [Min(0f)] public float thetaStep = 0f;
    
    [Header("Phi (Spin)")]
    public Vector2 phiRange = new Vector2(-180f, 180f);
    [Min(0f)] public float phiStep = 45f;
    
    [Header("Beta (Twist)")]
    public Vector2 betaRange = new Vector2(-45f, 45f);
    [Min(0f)] public float betaStep = 0f;

    [Header("Visuals & UI")]
    public Material targetMaterial;
    [Tooltip("Scales the size of the on-screen timer and score text.")]
    [Range(0.5f, 5f)] public float uiScaleMultiplier = 1.0f;

    [Header("CSV Target Logging")]
    public bool enableTargetLogging = true;
    public string targetLogFileName = "TargetSpawnLog.csv";

    // Neutral running tally of how many targets the tip has reached
    [Header("Read-only")]
    public int targetsReached = 0;
    public float elapsedTime = 0f;
    public bool isChallengeComplete = false;

    [Header("Interaction")]
    public bool enableKeyboardControl = true;

    // ------------------------------------------------------------ runtime state
    
    [System.NonSerialized] protected MonoBehaviour _activeVisualizer;

    // Helper properties to dynamically grab data via Reflection
    private float RobotL1 => GetValue<float>("L1", 25f);
    private float RobotL2 => GetValue<float>("L2", 25f);
    private float RobotSensorLength => GetValue<float>("sensorLength", 5f);
    private float RobotDiskThickness => GetValue<float>("diskThickness", 2f);
    private Vector3 RobotTipPositionMatlab => GetValue<Vector3>("TipPositionMatlab", Vector3.zero);
    private float RobotWorldScale => GetValue<float>("worldScale", 0.01f);
    
    // Arrays for live joint states
    private float[] RobotTheta => GetValue<float[]>("theta");
    private float[] RobotPhi => GetValue<float[]>("phi");
    private float[] RobotBeta => GetValue<float[]>("beta");

    // Target tracking lists
    private readonly List<Vector3> _targets = new List<Vector3>();
    private readonly List<Transform> _markers = new List<Transform>();
    private readonly List<float> _targetTimers = new List<float>(); 

    // Shuffle Bags (Deck of Cards)
    private List<float> _thetaBag = new List<float>();
    private List<float> _phiBag = new List<float>();
    private List<float> _betaBag = new List<float>();

    private float _lastDrawnTheta = -999f;
    private float _lastDrawnPhi = -999f;
    private float _lastDrawnBeta = -999f;

    private Vector2 _prevThetaRange; private float _prevThetaStep = -1f;
    private Vector2 _prevPhiRange;   private float _prevPhiStep = -1f;
    private Vector2 _prevBetaRange;  private float _prevBetaStep = -1f;

    private const string TargetPrefix = "SR_Target_";
    private bool _wasActive;
    private bool _timerRunning = false;

    // CSV Logging State
    private StreamWriter _targetLogWriter;
    private int _absoluteTargetId = 0; 

    // ------------------------------------------------------------------ lifecycle
    protected virtual void OnEnable()
    {
        FindActiveVisualizer();
    }

    protected virtual void OnDisable()
    {
        ClearTargets();
        CloseCSV();
        _wasActive = false;
        _timerRunning = false;
    }

    protected virtual void OnDestroy()
    {
        CloseCSV(); 
    }

    protected virtual void Update()
    {
        if (!Application.isPlaying) return;

        FindActiveVisualizer();

        if (enableKeyboardControl && ToggleKeyPressed())
            simulationActive = !simulationActive;

        if (!simulationActive && _wasActive)
        {
            CloseCSV();
        }

        if (simulationActive && !_wasActive)
        {
            targetsReached = 0;
            elapsedTime = 0f;
            isChallengeComplete = false;
            _timerRunning = true;
            _targets.Clear();
            _targetTimers.Clear();
            _absoluteTargetId = 0;

            if (enableTargetLogging)
            {
                string path = Path.Combine(Application.dataPath, targetLogFileName);
                _targetLogWriter = new StreamWriter(path, false); 
                _targetLogWriter.WriteLine("Timestamp,AbsoluteSpawnID,ObjectName,World_X,World_Y,World_Z");
                Debug.Log($"Logging target positions to: {path}");
            }

            Debug.Log(useTimerChallenge ? $"[sim] Timer started! Goal: {targetGoal} targets." : "[sim] Target simulation started.");
        }
        
        _wasActive = simulationActive;

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

        if (simulationActive || isChallengeComplete)
        {
            float s = uiScaleMultiplier;

            GUIStyle style = new GUIStyle();
            style.fontSize = Mathf.RoundToInt(24 * s);
            style.fontStyle = FontStyle.Bold;
            style.normal.textColor = Color.white;

            GUI.Box(new Rect(10 * s, 10 * s, 300 * s, 100 * s), "");

            string timeText = $"Time: {elapsedTime:F2} s";
            GUI.Label(new Rect(20 * s, 20 * s, 280 * s, 30 * s), timeText, style);

            string scoreText = useTimerChallenge 
                ? $"Targets: {targetsReached} / {targetGoal}" 
                : $"Targets Reached: {targetsReached}";
            GUI.Label(new Rect(20 * s, 50 * s, 280 * s, 30 * s), scoreText, style);

            if (useTimerChallenge && isChallengeComplete)
            {
                GUIStyle successStyle = new GUIStyle();
                successStyle.fontSize = Mathf.RoundToInt(32 * s);
                successStyle.fontStyle = FontStyle.Bold;
                successStyle.normal.textColor = Color.green;
                
                GUI.Label(new Rect(20 * s, 120 * s, 400 * s, 50 * s), "CHALLENGE COMPLETE!", successStyle);
            }
        }
    }

    // ------------------------------------------------- Reflection Magic
    private void FindActiveVisualizer()
    {
        if (_activeVisualizer != null && _activeVisualizer.enabled) return;

        _activeVisualizer = null;

        MonoBehaviour[] localBehaviours = GetComponents<MonoBehaviour>();
        foreach (var mb in localBehaviours)
        {
            if (mb != null && mb.GetType().Name.StartsWith("Visualizer_") && mb.enabled)
            {
                _activeVisualizer = mb;
                return;
            }
        }

        MonoBehaviour[] allBehaviours = FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Exclude);
        foreach (var mb in allBehaviours)
        {
            if (mb != null && mb.GetType().Name.StartsWith("Visualizer_") && mb.enabled)
            {
                _activeVisualizer = mb;
                return;
            }
        }
    }

    protected T GetValue<T>(string name, T defaultVal = default)
    {
        if (_activeVisualizer == null) return defaultVal;
        
        var type = _activeVisualizer.GetType();
        
        var field = type.GetField(name, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
        if (field != null && typeof(T).IsAssignableFrom(field.FieldType))
            return (T)field.GetValue(_activeVisualizer);

        var prop = type.GetProperty(name, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
        if (prop != null && typeof(T).IsAssignableFrom(prop.PropertyType))
            return (T)prop.GetValue(_activeVisualizer);

        return defaultVal;
    }

    // ---------------------------------------------- Deck Shuffle Logic
    private void CheckSettingsChanged()
    {
        if (Vector2.Distance(thetaRange, _prevThetaRange) > 0.01f || Mathf.Abs(thetaStep - _prevThetaStep) > 0.01f)
        {
            _thetaBag.Clear(); _lastDrawnTheta = -999f;
            _prevThetaRange = thetaRange; _prevThetaStep = thetaStep;
        }
        if (Vector2.Distance(phiRange, _prevPhiRange) > 0.01f || Mathf.Abs(phiStep - _prevPhiStep) > 0.01f)
        {
            _phiBag.Clear(); _lastDrawnPhi = -999f;
            _prevPhiRange = phiRange; _prevPhiStep = phiStep;
        }
        if (Vector2.Distance(betaRange, _prevBetaRange) > 0.01f || Mathf.Abs(betaStep - _prevBetaStep) > 0.01f)
        {
            _betaBag.Clear(); _lastDrawnBeta = -999f;
            _prevBetaRange = betaRange; _prevBetaStep = betaStep;
        }
    }

    private float GetNextBagAngle(List<float> bag, Vector2 range, float step, ref float lastDrawn)
    {
        if (step <= 0.001f) return Random.Range(range.x, range.y);

        if (bag.Count == 0)
        {
            int stepCount = Mathf.FloorToInt((range.y - range.x) / step);
            
            for (int i = 0; i <= stepCount; i++)
            {
                float val = range.x + (i * step);
                if (i > 0 && Mathf.Abs(val - (range.x + 360f)) < 1f) continue;
                bag.Add(val);
            }

            for (int i = bag.Count - 1; i > 0; i--)
            {
                int randomIndex = Random.Range(0, i + 1);
                float temp = bag[i];
                bag[i] = bag[randomIndex];
                bag[randomIndex] = temp;
            }

            if (lastDrawn != -999f && bag.Count > 1)
            {
                int topCardIndex = bag.Count - 1;
                if (Mathf.Abs(Mathf.DeltaAngle(bag[topCardIndex], lastDrawn)) < 1f)
                {
                    float temp = bag[0];
                    bag[0] = bag[topCardIndex];
                    bag[topCardIndex] = temp;
                }
            }
        }

        int lastIndex = bag.Count - 1;
        float drawn = bag[lastIndex];
        bag.RemoveAt(lastIndex);
        lastDrawn = drawn;

        return drawn;
    }

    private float GetRandomAngle(Vector2 range, float step)
    {
        if (step <= 0.001f) return Random.Range(range.x, range.y);

        int stepCount = Mathf.FloorToInt((range.y - range.x) / step);
        int randomStep = Random.Range(0, stepCount + 1);
        float snappedValue = range.x + (randomStep * step);
        
        if (randomStep > 0 && Mathf.Abs(snappedValue - (range.x + 360f)) < 1f)
        {
            snappedValue = range.x;
        }

        return Mathf.Min(snappedValue, range.y); 
    }

    // ---------------------------------------------- random_reachable_tip (PART B)
    public Vector3 RandomReachableTip(bool avoidObstacle = true, int maxTries = 25)
    {
        var th = new float[2];
        var ph = new float[2];
        var bt = new float[2];
        var len = new float[] { RobotL1, RobotL2 };

        // Pull the live state of the robot to find where the joint currently is
        float[] currentTh = RobotTheta;
        float[] currentPh = RobotPhi;
        float[] currentBt = RobotBeta;

        CheckSettingsChanged();

        Vector3 tip = Vector3.zero;
        for (int attempt = 0; attempt < maxTries; attempt++)
        {
            if (centerOnSecondSegmentJoint)
            {
                // Lock segment 1 to the actual current live robot angles
                th[0] = (currentTh != null && currentTh.Length > 0) ? currentTh[0] : 0f;
                ph[0] = (currentPh != null && currentPh.Length > 0) ? currentPh[0] : 0f;
                bt[0] = (currentBt != null && currentBt.Length > 0) ? currentBt[0] : 0f;
                
                // Randomize segment 2 to create a workspace centered on the joint
                th[1] = (useDeckShuffle ? GetNextBagAngle(_thetaBag, thetaRange, thetaStep, ref _lastDrawnTheta) : GetRandomAngle(thetaRange, thetaStep)) * Mathf.Deg2Rad;
                ph[1] = (useDeckShuffle ? GetNextBagAngle(_phiBag, phiRange, phiStep, ref _lastDrawnPhi) : GetRandomAngle(phiRange, phiStep)) * Mathf.Deg2Rad;
                bt[1] = (useDeckShuffle ? GetNextBagAngle(_betaBag, betaRange, betaStep, ref _lastDrawnBeta) : GetRandomAngle(betaRange, betaStep)) * Mathf.Deg2Rad;
            }
            else if (syncSegments)
            {
                // Legacy: Both segments bend identically around the origin
                float sharedTh = (useDeckShuffle ? GetNextBagAngle(_thetaBag, thetaRange, thetaStep, ref _lastDrawnTheta) : GetRandomAngle(thetaRange, thetaStep)) * Mathf.Deg2Rad;
                float sharedPh = (useDeckShuffle ? GetNextBagAngle(_phiBag, phiRange, phiStep, ref _lastDrawnPhi) : GetRandomAngle(phiRange, phiStep)) * Mathf.Deg2Rad;
                float sharedBt = (useDeckShuffle ? GetNextBagAngle(_betaBag, betaRange, betaStep, ref _lastDrawnBeta) : GetRandomAngle(betaRange, betaStep)) * Mathf.Deg2Rad;

                for (int i = 0; i < 2; i++)
                {
                    th[i] = sharedTh;
                    ph[i] = sharedPh;
                    bt[i] = sharedBt;
                }
            }
            else
            {
                // Legacy: Both segments bend independently around the origin
                for (int i = 0; i < 2; i++)
                {
                    th[i] = (useDeckShuffle ? GetNextBagAngle(_thetaBag, thetaRange, thetaStep, ref _lastDrawnTheta) : GetRandomAngle(thetaRange, thetaStep)) * Mathf.Deg2Rad;
                    ph[i] = (useDeckShuffle ? GetNextBagAngle(_phiBag, phiRange, phiStep, ref _lastDrawnPhi) : GetRandomAngle(phiRange, phiStep)) * Mathf.Deg2Rad;
                    bt[i] = (useDeckShuffle ? GetNextBagAngle(_betaBag, betaRange, betaStep, ref _lastDrawnBeta) : GetRandomAngle(betaRange, betaStep)) * Mathf.Deg2Rad;
                }
            }

            // Calculate the tip based on the kinematics model
            SoftRobotKinematics.FwdSRM(
                th, ph, bt, len,
                Vector3.zero, Vector3.zero,
                out Matrix4x4 tipT, out _,
                RobotSensorLength, RobotDiskThickness);
            
            tip = SoftRobotKinematics.GetPosition(tipT);
            
            // Multiply it by the slider value to push it further out radially
            tip *= distanceMultiplier;

            if (!avoidObstacle) return tip;
            if (!IsPointBlocked(tip, extra: targetRadius)) return tip;
        }
        return tip;
    }

    // --------------------------------------------------- update_targets (PART B)
    private void UpdateTargets()
    {
        if (useTimerChallenge && isChallengeComplete)
        {
            ClearTargets();
            return;
        }

        while (_targets.Count < targetCount) 
        {
            Vector3 newTarget = RandomReachableTip();
            _targets.Add(newTarget);
            _targetTimers.Add(0f);
            
            LogTargetToCSV(_absoluteTargetId, _targets.Count - 1, newTarget);
            _absoluteTargetId++;
        }
        while (_targets.Count > targetCount) 
        {
            _targets.RemoveAt(_targets.Count - 1);
            _targetTimers.RemoveAt(_targetTimers.Count - 1);
        }

        Vector3 tip = RobotTipPositionMatlab;
        bool blocked = requireClearance && IsBodyInContact();
        
        for (int i = 0; i < _targets.Count; i++)
        {
            if (blocked || Vector3.Distance(tip, _targets[i]) > reachTolerance)
            {
                _targetTimers[i] = 0f;
                continue;
            }

            _targetTimers[i] += Time.deltaTime;

            if (_targetTimers[i] >= requiredHoldTime)
            {
                targetsReached++;
                
                if (useTimerChallenge && targetsReached >= targetGoal)
                {
                    _timerRunning = false;
                    isChallengeComplete = true;
                    Debug.Log($"<color=green>[sim] CHALLENGE COMPLETE! Reached {targetGoal} targets in {elapsedTime:F2} seconds.</color>");
                    
                    ClearTargets();
                    return;
                }
                else
                {
                    Vector3 newTarget = RandomReachableTip();
                    _targets[i] = newTarget;
                    _targetTimers[i] = 0f; 
                    
                    LogTargetToCSV(_absoluteTargetId, i, newTarget);
                    _absoluteTargetId++;
                    
                    Debug.Log($"[sim] target reached (held for {requiredHoldTime}s); total reached = {targetsReached}");
                }
            }
        }

        RedrawMarkers();
    }

    // -------------------------------------------------------- CSV Methods
    private void LogTargetToCSV(int absId, int listIndex, Vector3 matlabPos)
    {
        if (_targetLogWriter != null)
        {
            Vector3 localUnityPos = SoftRobotKinematics.ToUnity(matlabPos, RobotWorldScale);
            Vector3 worldUnityPos = transform.TransformPoint(localUnityPos);

            string timestamp = Time.time.ToString("F3");
            string objName = TargetPrefix + listIndex;
            
            _targetLogWriter.WriteLine($"{timestamp},{absId},{objName},{worldUnityPos.x},{worldUnityPos.y},{worldUnityPos.z}");
        }
    }

    private void CloseCSV()
    {
        if (_targetLogWriter != null)
        {
            _targetLogWriter.Close();
            _targetLogWriter = null;
        }
    }

    // -------------------------------------------------------- obstacle hooks
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

        float scale = RobotWorldScale;
        for (int i = 0; i < _markers.Count; i++)
        {
            _markers[i].localPosition = SoftRobotKinematics.ToUnity(_targets[i], scale);
            float d = targetRadius * scale * 2f;   
            _markers[i].localScale = new Vector3(d, d, d);
        }
    }

    private void ClearTargets()
    {
        _targets.Clear();
        _targetTimers.Clear(); 
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
        Shader s = Shader.Find("Universal Render Pipeline/Lit");
        if (s == null) s = Shader.Find("Standard");
        if (s == null) s = Shader.Find("Sprites/Default");
        var mat = new Material(s);
        mat.color = new Color(0.1f, 0.9f, 0.2f, 1f);   
        return mat;
    }

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