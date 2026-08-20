using UnityEngine;
using System.Reflection;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Universal Telemetry + controls panel for the soft manipulator.
/// Runs in edit-mode to instantly start with the Visualizer!
/// </summary>
[ExecuteAlways]
public class HUD_Phudit_Universal : MonoBehaviour
{
    [Header("Source Visualizer")]
    [Tooltip("Leave empty. It will automatically find any script starting with 'Visualizer_'")]
    public MonoBehaviour activeVisualizer;

    [Tooltip("Leave empty to find the target simulation automatically.")]
    public SoftRobotTargetSim sim;

    [Header("Size")]
    [Tooltip("Overall size multiplier. Bump this if the panel is hard to read.")]
    [Range(0.5f, 4f)] public float uiScale = 1.5f;

    [Tooltip("Baseline screen height. 1080 suits most displays.")]
    public int referenceHeight = 1080;

    [Header("Panel")]
    public bool visible = true;
    public bool showControls = true;
    public Corner corner = Corner.TopRight;
    public int panelWidth = 330; 
    public int margin = 10;

    [Header("Behaviour")]
    public bool escapeQuits = true;

    public enum Corner { TopLeft, TopRight, BottomLeft, BottomRight }

    private static readonly string[,] ControlRows =
    {
        { "1 / 2",       "select active segment" },
        { "Load U/D",    "bend  \u03B8 (X-axis)" },
        { "Load L/R",    "bend  \u03C6 (Z-axis)" },
        { "FSR Press",   "elevate Z-insertion" },
        { "R",           "reset pose" },
        { "T",           "target sim on/off" },
        { "O",           "obstacle on/off" },
        { "right-drag",  "orbit camera" },
        { "middle-drag", "pan camera" },
        { "scroll",      "zoom" },
        { "F",           "re-centre camera" },
        { "F1",          "hide this panel" },
    };

    private SoftRobotObstacleCourse _obstacle;
    private GUIStyle _box, _header, _label, _value, _key, _on, _off;
    private float _builtAtScale = -1f;

    private void Start()
    {
        FindActiveVisualizer();

        if (sim == null) 
        {
            sim = FindAnyObjectByType<SoftRobotTargetSim>(); 
        }
        _obstacle = sim as SoftRobotObstacleCourse;
    }

    private void Update()
    {
        // Only check inputs if the game is actually playing
        if (Application.isPlaying)
        {
            if (TogglePressed()) visible = !visible;
            if (escapeQuits && QuitPressed() && !Application.isEditor) Application.Quit();
        }
        
        // Always try to link up if we lose the visualizer (even in Edit mode)
        if (activeVisualizer == null)
        {
            FindActiveVisualizer();
        }
    }

   private void FindActiveVisualizer()
    {
        // 1. If we already have one, make sure it is still active/enabled. 
        // If it got disabled (checkbox unticked), we clear it so we can find the new one.
        if (activeVisualizer != null && activeVisualizer.enabled)
        {
            return;
        }

        activeVisualizer = null;

        // 2. FIRST PRIORITY: Check the exact same GameObject
        MonoBehaviour[] localBehaviours = GetComponents<MonoBehaviour>();
        foreach (var mb in localBehaviours)
        {
            // Only lock on if the script name matches AND the checkbox is enabled!
            if (mb != null && mb.GetType().Name.StartsWith("Visualizer_") && mb.enabled)
            {
                activeVisualizer = mb;
                return;
            }
        }

        // 3. SECOND PRIORITY: If not found locally, search the entire scene
        MonoBehaviour[] allBehaviours = FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Exclude);
        foreach (var mb in allBehaviours)
        {
            if (mb != null && mb.GetType().Name.StartsWith("Visualizer_") && mb.enabled)
            {
                activeVisualizer = mb;
                return;
            }
        }
    }

    // --- MAGIC REFLECTION HELPER ---
    // Extracts variables regardless of what version is currently running
    private T GetValue<T>(string name, T defaultVal = default)
    {
        if (activeVisualizer == null) return defaultVal;
        
        var type = activeVisualizer.GetType();
        
        var field = type.GetField(name, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
        if (field != null && field.FieldType == typeof(T))
            return (T)field.GetValue(activeVisualizer);

        var prop = type.GetProperty(name, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
        if (prop != null && prop.PropertyType == typeof(T))
            return (T)prop.GetValue(activeVisualizer);

        return defaultVal;
    }

    private void BuildStyles(float scale)
    {
        if (Mathf.Approximately(_builtAtScale, scale)) return;
        _builtAtScale = scale;

        int baseFont = Mathf.Max(8, Mathf.RoundToInt(12f * scale));
        int pad = Mathf.RoundToInt(10f * scale);

        var bg = new Texture2D(1, 1);
        bg.SetPixel(0, 0, new Color(0f, 0f, 0f, 0.72f));
        bg.Apply();

        _box = new GUIStyle(GUI.skin.box);
        _box.normal.background = bg;
        _box.padding = new RectOffset(pad, pad, pad, pad);

        _header = new GUIStyle(GUI.skin.label)
        {
            fontSize = baseFont,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleLeft
        };
        _header.normal.textColor = new Color(0.55f, 0.8f, 1f);

        _label = new GUIStyle(GUI.skin.label) { fontSize = baseFont };
        _label.normal.textColor = new Color(0.75f, 0.75f, 0.78f);

        _value = new GUIStyle(GUI.skin.label)
        {
            fontSize = baseFont,
            alignment = TextAnchor.MiddleRight
        };
        _value.normal.textColor = Color.white;

        _key = new GUIStyle(GUI.skin.label) { fontSize = baseFont };
        _key.normal.textColor = new Color(1f, 0.85f, 0.5f);

        _on = new GUIStyle(_value);
        _on.normal.textColor = new Color(0.35f, 0.9f, 0.4f);

        _off = new GUIStyle(_value);
        _off.normal.textColor = new Color(0.6f, 0.6f, 0.62f);
    }

    private void OnGUI()
    {
        if (!visible || activeVisualizer == null) return;

        float scale = (Screen.height / (float)Mathf.Max(1, referenceHeight)) * uiScale;
        BuildStyles(scale);

        string verName = activeVisualizer.GetType().Name;
        float t1 = GetValue<float>("theta1Deg");
        float p1 = GetValue<float>("phi1Deg");
        float b1 = GetValue<float>("beta1Deg");
        float L1 = GetValue<float>("L1");

        float t2 = GetValue<float>("theta2Deg");
        float p2 = GetValue<float>("phi2Deg");
        float b2 = GetValue<float>("beta2Deg");
        float L2 = GetValue<float>("L2");

        float insertion = GetValue<float>("currentInsertionCm");
        int activeSeg = GetValue<int>("activeSegment", 1);
        
        Vector3 tipModel = GetValue<Vector3>("TipPositionMatlab");
        Vector3 tipWorld = GetValue<Vector3>("TipPosition");

        float rowH = _label.CalcHeight(new GUIContent("Xg"), 10000f) + _label.margin.vertical;
        int gap   = Mathf.RoundToInt(6f * scale);
        int pad2  = _box.padding.vertical;

        int rows = 1 + 5       
                 + 1 + 3       
                 + 1;          
        int gaps = 2;

        if (sim != null)
        {
            rows += 1 + 2;     
            if (_obstacle != null) rows += 2; 
            gaps += 1;
        }
        if (showControls)
        {
            rows += 1 + ControlRows.GetLength(0);
            if (escapeQuits) rows += 1;
            gaps += 1;
        }

        int h = Mathf.CeilToInt(rows * rowH) + gaps * gap + pad2;
        int w = Mathf.RoundToInt(panelWidth * scale);
        h = Mathf.Min(h, Screen.height - margin * 2);

        GUILayout.BeginArea(CornerRect(w, h), _box);

        GUILayout.Label("INPUTS  (joint configuration)", _header);
        Row("Active Driver", verName);
        Row("Segment 1  \u03B8 / \u03C6 / \u03B2", $"{t1,6:F1}  {p1,6:F1}  {b1,6:F1}");
        Row("Segment 2  \u03B8 / \u03C6 / \u03B2", $"{t2,6:F1}  {p2,6:F1}  {b2,6:F1}");
        Row("Lengths L1 / L2", $"{L1:F1} / {L2:F1} cm");
        Row("Elevation Depth", $"{insertion:F1} cm"); 

        GUILayout.Space(gap);
        GUILayout.Label("OUTPUT  (end effector)", _header);
        Row("Model (cm)", $"{tipModel.x:F2}, {tipModel.y:F2}, {tipModel.z:F2}");
        Row("Unity (m)",  $"{tipWorld.x:F3}, {tipWorld.y:F3}, {tipWorld.z:F3}");
        Row("Active segment", activeSeg.ToString());

        if (sim != null)
        {
            GUILayout.Space(gap);
            GUILayout.Label("SIMULATION", _header);
            StateRow("Target sim", sim.simulationActive);
            Row("Targets reached", sim.targetsReached.ToString());

            if (_obstacle != null)
            {
                StateRow("Obstacle", _obstacle.obstacleActive);
                Row("Contact events", _obstacle.contactEvents.ToString());
            }
        }

        if (showControls)
        {
            GUILayout.Space(gap);
            GUILayout.Label("CONTROLS", _header);
            for (int i = 0; i < ControlRows.GetLength(0); i++)
                KeyRow(ControlRows[i, 0], ControlRows[i, 1]);
            if (escapeQuits) KeyRow("Esc", "quit");
        }

        GUILayout.EndArea();
    }

    private void Row(string label, string value)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(label, _label);
        GUILayout.FlexibleSpace();
        GUILayout.Label(value, _value);
        GUILayout.EndHorizontal();
    }

    private void StateRow(string label, bool state)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(label, _label);
        GUILayout.FlexibleSpace();
        GUILayout.Label(state ? "ON" : "OFF", state ? _on : _off);
        GUILayout.EndHorizontal();
    }

    private void KeyRow(string key, string description)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(key, _key, GUILayout.Width(Mathf.RoundToInt(110f * _builtAtScale)));
        GUILayout.Label(description, _label);
        GUILayout.EndHorizontal();
    }

    private Rect CornerRect(int w, int h)
    {
        float sw = Screen.width;
        float sh = Screen.height;

        switch (corner)
        {
            case Corner.TopLeft:     return new Rect(margin, margin, w, h);
            case Corner.BottomLeft:  return new Rect(margin, sh - h - margin, w, h);
            case Corner.BottomRight: return new Rect(sw - w - margin, sh - h - margin, w, h);
            default:                 return new Rect(sw - w - margin, margin, w, h);
        }
    }

    private static bool TogglePressed()
    {
#if ENABLE_INPUT_SYSTEM
        return Keyboard.current != null && Keyboard.current.f1Key.wasPressedThisFrame;
#else
        return Input.GetKeyDown(KeyCode.F1);
#endif
    }

    private static bool QuitPressed()
    {
#if ENABLE_INPUT_SYSTEM
        return Keyboard.current != null && Keyboard.current.escapeKey.wasPressedThisFrame;
#else
        return Input.GetKeyDown(KeyCode.Escape);
#endif
    }
}