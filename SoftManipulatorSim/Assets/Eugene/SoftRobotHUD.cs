using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Telemetry + controls panel for the soft manipulator.
///
/// Shows joint inputs (theta / phi / beta per segment), the end-effector output
/// position, live simulation counters, and a legend of every key binding.
///
/// Uses OnGUI deliberately: it renders identically in a standalone build with
/// no UI assets, no Canvas and no scene setup. The Unity Inspector does NOT
/// exist in a build, so this is what replaces it.
///
/// SETUP
///   Add to the SoftRobot GameObject and press Play. It finds the visualiser
///   and the simulation component automatically.
///
/// Counters come from SoftRobotTargetSim (targetsReached) and, when the
/// obstacle course subclass is present, SoftRobotObstacleCourse (contactEvents).
/// Rows for components that are absent are simply not drawn.
/// </summary>
public class SoftRobotHUD : MonoBehaviour
{
    [Header("Source")]
    [Tooltip("Leave empty to find the SoftRobotVisualizer automatically.")]
    public SoftRobotVisualizer robot;

    [Tooltip("Leave empty to find the target simulation automatically. " +
             "SoftRobotObstacleCourse is a subclass, so assigning either works.")]
    public SoftRobotTargetSim sim;

    [Header("Size")]
    [Tooltip("Overall size multiplier. Bump this if the panel is hard to read.")]
    [Range(0.5f, 4f)] public float uiScale = 1.5f;

    [Tooltip("The panel is scaled so it looks the same at any resolution, using " +
             "this as the baseline screen height. 1080 suits most displays.")]
    public int referenceHeight = 1080;

    [Header("Panel")]
    public bool visible = true;
    [Tooltip("Show the key-binding legend. Turn off for a compact readout.")]
    public bool showControls = true;
    public Corner corner = Corner.TopRight;
    public int panelWidth = 310;
    public int margin = 10;

    [Header("Behaviour")]
    [Tooltip("Allow Esc to quit in a standalone build. Ignored in the editor.")]
    public bool escapeQuits = true;

    public enum Corner { TopLeft, TopRight, BottomLeft, BottomRight }

    // Key legend, kept as data so panel height can be computed before layout.
    private static readonly string[,] ControlRows =
    {
        { "1 / 2",       "select segment" },
        { "W / S",       "bend  \u03B8" },
        { "A / D",       "swing \u03C6" },
        { "Q / E",       "twist \u03B2" },
        { "R",           "reset pose" },
        { "T",           "target sim on/off" },
        { "O",           "obstacle on/off" },
        { "right-drag",  "orbit camera" },
        { "middle-drag", "pan camera" },
        { "scroll",      "zoom" },
        { "F",           "re-centre camera" },
        { "F1",          "hide this panel" },
    };

    private SoftRobotObstacleCourse _obstacle;   // null when only the base sim is present
    private GUIStyle _box, _header, _label, _value, _key, _on, _off;
    private float _builtAtScale = -1f;

    private void Start()
    {
        if (robot == null) robot = FindAnyObjectByType<SoftRobotVisualizer>();
        if (sim == null)   sim   = FindAnyObjectByType<SoftRobotTargetSim>();
        _obstacle = sim as SoftRobotObstacleCourse;

        if (robot == null)
            Debug.LogWarning("[SoftRobotHUD] No SoftRobotVisualizer found; panel will be empty.", this);
    }

    private void Update()
    {
        if (TogglePressed()) visible = !visible;
        if (escapeQuits && QuitPressed() && !Application.isEditor) Application.Quit();
    }

    private void BuildStyles(float scale)
    {
        // Rebuild only when the scale changes. Fonts rasterise at their point
        // size, so scaling fontSize keeps glyphs crisp -- scaling GUI.matrix
        // would stretch an already-rendered bitmap and blur it.
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
        _key.normal.textColor = new Color(1f, 0.85f, 0.5f);      // warm = "press this"

        _on = new GUIStyle(_value);
        _on.normal.textColor = new Color(0.35f, 0.9f, 0.4f);     // green = running

        _off = new GUIStyle(_value);
        _off.normal.textColor = new Color(0.6f, 0.6f, 0.62f);    // grey = idle
    }

    private void OnGUI()
    {
        if (!visible || robot == null) return;

        float scale = (Screen.height / (float)Mathf.Max(1, referenceHeight)) * uiScale;
        BuildStyles(scale);

        // The visualiser already publishes both, so no need to recompute here --
        // and reading them guarantees the HUD agrees with what is on screen.
        Vector3 tipModel = robot.TipPositionMatlab;   // cm, model frame
        Vector3 tipWorld = robot.TipPosition;         // m, Unity world

        // ---- size the panel to its contents -----------------------------------
        // Measure a real line rather than assuming a pixel height: GUILayout adds
        // the style's own margin around each row, and guessing it under-sizes the
        // box, which silently clips the bottom of the panel.
        float rowH = _label.CalcHeight(new GUIContent("Xg"), 10000f)
                   + _label.margin.vertical;
        int gap   = Mathf.RoundToInt(6f * scale);
        int pad2  = _box.padding.vertical;

        int rows = 1 + 3       // INPUTS header + 3 rows
                 + 1 + 2       // OUTPUT header + model + unity (xyz on one line each)
                 + 1;          // active segment
        int gaps = 2;

        if (sim != null)
        {
            rows += 1 + 2;                       // SIMULATION header + target rows
            if (_obstacle != null) rows += 2;    // obstacle rows
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

        // Never let the panel run off the bottom of the screen.
        h = Mathf.Min(h, Screen.height - margin * 2);

        GUILayout.BeginArea(CornerRect(w, h), _box);

        GUILayout.Label("INPUTS  (joint configuration)", _header);
        Row("Segment 1  \u03B8 / \u03C6 / \u03B2",
            $"{robot.theta1Deg,6:F1}  {robot.phi1Deg,6:F1}  {robot.beta1Deg,6:F1}");
        Row("Segment 2  \u03B8 / \u03C6 / \u03B2",
            $"{robot.theta2Deg,6:F1}  {robot.phi2Deg,6:F1}  {robot.beta2Deg,6:F1}");
        Row("Lengths L1 / L2", $"{robot.L1:F1} / {robot.L2:F1} cm");

        GUILayout.Space(gap);
        GUILayout.Label("OUTPUT  (end effector)", _header);
        Row("Model (cm)", $"{tipModel.x:F2}, {tipModel.y:F2}, {tipModel.z:F2}");
        Row("Unity (m)",  $"{tipWorld.x:F3}, {tipWorld.y:F3}, {tipWorld.z:F3}");
        Row("Active segment", robot.activeSegment.ToString());

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

    /// <summary>ON in green, OFF in grey -- readable at a glance from across a room.</summary>
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
        GUILayout.Label(key, _key, GUILayout.Width(Mathf.RoundToInt(92f * _builtAtScale)));
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

    // --------------------------------------------------------- input backends
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
