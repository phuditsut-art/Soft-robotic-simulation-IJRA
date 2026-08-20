using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Runtime camera for inspecting the soft manipulator, with two modes that
/// together approximate Unity's own Scene-view navigation.
///
/// The complication this solves: the robot already claims W/A/S/D, Q/E, 1/2, R
/// and T. A Scene-view-style WASD flythrough would fight it. So the camera has
/// an explicit mode toggle, and while flying it suspends the robot's keyboard
/// input (via SoftRobotVisualizer.enableKeyboardControl) and restores it on the
/// way out. Only one thing consumes the keyboard at a time.
///
/// SETUP
///   1. Select Main Camera, Add Component -> OrbitCamera.
///   2. Drag the SoftRobot GameObject into the Target field.
///   3. Press Play.
///
/// TAB toggles the mode.
///
/// ORBIT MODE (default — robot keys active)
///   right-drag / Alt+left-drag   orbit around the target
///   middle-drag                  pan the pivot
///   scroll                       zoom
///   F                            re-centre and reset zoom
///
/// FLY MODE (robot keys suspended)
///   right-drag                   look around
///   W / S                        forward / back
///   A / D                        left / right
///   Q / E                        down / up
///   hold Shift                   move faster
///   scroll                       adjust movement speed
///   F                            snap back to the target
/// </summary>
[RequireComponent(typeof(Camera))]
public class OrbitCamera : MonoBehaviour
{
    public enum CameraMode { Orbit, Fly }

    [Header("Target")]
    [Tooltip("What to orbit around. Leave empty to auto-find the SoftRobot.")]
    public Transform target;

    [Tooltip("Offset from the target's origin, in metres. The robot's base sits " +
             "at its origin and it extends upward, so lifting the pivot keeps the " +
             "whole arm in frame.")]
    public Vector3 pivotOffset = new Vector3(0f, 0.15f, 0f);

    [Header("Mode")]
    public CameraMode mode = CameraMode.Orbit;
    [Tooltip("Show a small on-screen reminder of the current mode and its keys.")]
    public bool showModeHint = true;

    [Header("Orbit")]
    public float yaw = 35f;
    public float pitch = 20f;
    public float orbitSensitivity = 0.25f;      // degrees per pixel
    [Range(-89f, 0f)] public float minPitch = -80f;
    [Range(0f, 89f)]  public float maxPitch = 85f;

    [Header("Zoom / distance")]
    public float distance = 1.2f;
    public float minDistance = 0.15f;
    public float maxDistance = 6f;
    public float zoomSensitivity = 0.15f;

    [Header("Pan")]
    public float panSensitivity = 0.0015f;

    [Header("Fly")]
    [Tooltip("Metres per second. The robot is only ~0.6 m tall, so this is small " +
             "by game standards on purpose.")]
    public float flySpeed = 0.6f;
    public float flyBoostMultiplier = 3f;
    public float lookSensitivity = 0.15f;
    public float minFlySpeed = 0.05f;
    public float maxFlySpeed = 5f;

    [Header("Smoothing")]
    [Tooltip("0 = instant. Applies to orbit mode only; flying is always direct.")]
    [Range(0f, 0.4f)] public float smoothTime = 0.06f;

    private Vector3 _panOffset;
    private Vector3 _currentPivot;
    private Vector3 _pivotVelocity;
    private float _currentDistance;
    private float _distanceVelocity;

    private SoftRobotVisualizer _robot;
    private bool _robotKeysWereEnabled = true;

    private void Start()
    {
        if (target == null)
        {
            // FindAnyObjectByType rather than FindFirstObjectByType: the latter is
            // deprecated in newer Unity 6 builds because it depends on instance-ID
            // ordering. Any instance is fine -- there is only one robot.
            var vis = FindAnyObjectByType<SoftRobotVisualizer>();
            if (vis != null) target = vis.transform;
            else Debug.LogWarning(
                "[OrbitCamera] No Target assigned and no SoftRobotVisualizer found. " +
                "Orbiting the world origin. Drag SoftRobot into the Target field.", this);
        }

        _robot = (target != null) ? target.GetComponent<SoftRobotVisualizer>() : null;
        if (_robot == null) _robot = FindAnyObjectByType<SoftRobotVisualizer>();
        if (_robot != null) _robotKeysWereEnabled = _robot.enableKeyboardControl;

        _currentPivot = DesiredPivot();
        _currentDistance = distance;
        SyncFromTransformIfNeeded();
        ApplyOrbitTransform();
    }

    private void OnDisable()
    {
        // Never leave the robot's controls switched off because we happened to
        // stop play while in fly mode.
        RestoreRobotKeys();
    }

    private void LateUpdate()
    {
        // LateUpdate so the camera settles after the robot has rebuilt this frame.
        if (TogglePressed())
            SetMode(mode == CameraMode.Orbit ? CameraMode.Fly : CameraMode.Orbit);

        if (mode == CameraMode.Orbit) UpdateOrbit();
        else                          UpdateFly();
    }

    // ------------------------------------------------------------------ modes
    private void SetMode(CameraMode newMode)
    {
        if (newMode == mode) return;

        if (newMode == CameraMode.Fly)
        {
            // Remember the robot's setting, then suspend it so W/A/S/D fly the
            // camera instead of bending the arm.
            if (_robot != null)
            {
                _robotKeysWereEnabled = _robot.enableKeyboardControl;
                _robot.enableKeyboardControl = false;
            }
        }
        else
        {
            RestoreRobotKeys();
            // Re-derive orbit parameters from wherever flying left us, so the
            // switch back doesn't teleport the view.
            SyncFromTransform();
        }
        mode = newMode;
    }

    private void RestoreRobotKeys()
    {
        if (_robot != null) _robot.enableKeyboardControl = _robotKeysWereEnabled;
    }

    private void SyncFromTransformIfNeeded()
    {
        if (mode == CameraMode.Fly) SyncFromTransform();
    }

    /// <summary>Recompute yaw/pitch/distance from the camera's current pose.</summary>
    private void SyncFromTransform()
    {
        Vector3 pivot = DesiredPivot();
        Vector3 offset = transform.position - pivot;
        float d = offset.magnitude;
        if (d > 1e-4f)
        {
            distance = Mathf.Clamp(d, minDistance, maxDistance);
            Vector3 dir = -offset.normalized;
            pitch = Mathf.Clamp(-Mathf.Asin(dir.y) * Mathf.Rad2Deg, minPitch, maxPitch);
            yaw = Mathf.Atan2(dir.x, dir.z) * Mathf.Rad2Deg;
        }
        _currentDistance = distance;
        _currentPivot = pivot;
    }

    // ------------------------------------------------------------------ orbit
    private void UpdateOrbit()
    {
        Vector2 md = MouseDelta();

        if (RightHeld() || (AltHeld() && LeftHeld()))
        {
            yaw   += md.x * orbitSensitivity;
            pitch -= md.y * orbitSensitivity;
            pitch  = Mathf.Clamp(pitch, minPitch, maxPitch);
        }

        if (MiddleHeld())
        {
            // Pan in the camera's own plane, scaled by distance so it feels the
            // same whether zoomed in or out.
            _panOffset += (-transform.right * md.x - transform.up * md.y)
                          * panSensitivity * _currentDistance;
        }

        float scroll = ScrollDelta();
        if (Mathf.Abs(scroll) > 0.0001f)
        {
            // Multiplicative, so each notch feels equal at any zoom level.
            distance = Mathf.Clamp(distance * Mathf.Exp(-scroll * zoomSensitivity),
                                   minDistance, maxDistance);
        }

        if (RefocusPressed())
        {
            _panOffset = Vector3.zero;
            distance = Mathf.Clamp(1.2f, minDistance, maxDistance);
        }

        Vector3 pivot = DesiredPivot();
        if (smoothTime > 0f)
        {
            _currentPivot = Vector3.SmoothDamp(_currentPivot, pivot, ref _pivotVelocity, smoothTime);
            _currentDistance = Mathf.SmoothDamp(_currentDistance, distance, ref _distanceVelocity, smoothTime);
        }
        else
        {
            _currentPivot = pivot;
            _currentDistance = distance;
        }
        ApplyOrbitTransform();
    }

    private void ApplyOrbitTransform()
    {
        Quaternion rot = Quaternion.Euler(pitch, yaw, 0f);
        transform.rotation = rot;
        transform.position = _currentPivot - rot * Vector3.forward * _currentDistance;
    }

    // -------------------------------------------------------------------- fly
    private void UpdateFly()
    {
        Vector2 md = MouseDelta();

        if (RightHeld())
        {
            yaw   += md.x * lookSensitivity;
            pitch -= md.y * lookSensitivity;
            pitch  = Mathf.Clamp(pitch, -89f, 89f);
            transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
        }

        // Scroll adjusts speed rather than zooming — matches Scene-view flythrough.
        float scroll = ScrollDelta();
        if (Mathf.Abs(scroll) > 0.0001f)
            flySpeed = Mathf.Clamp(flySpeed * Mathf.Exp(scroll * 0.15f), minFlySpeed, maxFlySpeed);

        Vector3 move = Vector3.zero;
        if (Held(Key_W)) move += transform.forward;
        if (Held(Key_S)) move -= transform.forward;
        if (Held(Key_D)) move += transform.right;
        if (Held(Key_A)) move -= transform.right;
        if (Held(Key_E)) move += Vector3.up;
        if (Held(Key_Q)) move -= Vector3.up;

        if (move.sqrMagnitude > 1e-6f)
        {
            float speed = flySpeed * (ShiftHeld() ? flyBoostMultiplier : 1f);
            transform.position += move.normalized * speed * Time.deltaTime;
        }

        if (RefocusPressed())
        {
            SyncFromTransform();
            SetMode(CameraMode.Orbit);
        }
    }

    private Vector3 DesiredPivot()
    {
        Vector3 basePoint = (target != null) ? target.position : Vector3.zero;
        return basePoint + pivotOffset + _panOffset;
    }

    // ------------------------------------------------------------------- hint
    private void OnGUI()
    {
        if (!showModeHint) return;
        var style = new GUIStyle(GUI.skin.label) { fontSize = 12 };
        style.normal.textColor = Color.white;
        string text = (mode == CameraMode.Orbit)
            ? "Camera: ORBIT  |  Tab = fly  |  right-drag orbit, middle-drag pan, scroll zoom, F recentre  |  robot keys ACTIVE"
            : "Camera: FLY  |  Tab = orbit  |  right-drag look, WASD move, Q/E down/up, Shift boost  |  robot keys SUSPENDED";
        GUI.Label(new Rect(10, 10, 1100, 20), text, style);
    }

    // --------------------------------------------------------- input backends
    private const int Key_W = 0, Key_A = 1, Key_S = 2, Key_D = 3, Key_Q = 4, Key_E = 5;

    private static bool Held(int k)
    {
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current;
        if (kb == null) return false;
        switch (k)
        {
            case Key_W: return kb.wKey.isPressed;
            case Key_A: return kb.aKey.isPressed;
            case Key_S: return kb.sKey.isPressed;
            case Key_D: return kb.dKey.isPressed;
            case Key_Q: return kb.qKey.isPressed;
            case Key_E: return kb.eKey.isPressed;
        }
        return false;
#else
        switch (k)
        {
            case Key_W: return Input.GetKey(KeyCode.W);
            case Key_A: return Input.GetKey(KeyCode.A);
            case Key_S: return Input.GetKey(KeyCode.S);
            case Key_D: return Input.GetKey(KeyCode.D);
            case Key_Q: return Input.GetKey(KeyCode.Q);
            case Key_E: return Input.GetKey(KeyCode.E);
        }
        return false;
#endif
    }

    private static Vector2 MouseDelta()
    {
#if ENABLE_INPUT_SYSTEM
        return Mouse.current != null ? Mouse.current.delta.ReadValue() : Vector2.zero;
#else
        return new Vector2(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y")) * 10f;
#endif
    }

    private static bool LeftHeld()
    {
#if ENABLE_INPUT_SYSTEM
        return Mouse.current != null && Mouse.current.leftButton.isPressed;
#else
        return Input.GetMouseButton(0);
#endif
    }

    private static bool RightHeld()
    {
#if ENABLE_INPUT_SYSTEM
        return Mouse.current != null && Mouse.current.rightButton.isPressed;
#else
        return Input.GetMouseButton(1);
#endif
    }

    private static bool MiddleHeld()
    {
#if ENABLE_INPUT_SYSTEM
        return Mouse.current != null && Mouse.current.middleButton.isPressed;
#else
        return Input.GetMouseButton(2);
#endif
    }

    private static bool AltHeld()
    {
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current;
        return kb != null && (kb.leftAltKey.isPressed || kb.rightAltKey.isPressed);
#else
        return Input.GetKey(KeyCode.LeftAlt) || Input.GetKey(KeyCode.RightAlt);
#endif
    }

    private static bool ShiftHeld()
    {
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current;
        return kb != null && (kb.leftShiftKey.isPressed || kb.rightShiftKey.isPressed);
#else
        return Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
#endif
    }

    private static float ScrollDelta()
    {
#if ENABLE_INPUT_SYSTEM
        // The new backend reports ~120 units per notch on Windows; normalise.
        return Mouse.current != null ? Mouse.current.scroll.ReadValue().y / 120f : 0f;
#else
        return Input.mouseScrollDelta.y;
#endif
    }

    private static bool TogglePressed()
    {
#if ENABLE_INPUT_SYSTEM
        return Keyboard.current != null && Keyboard.current.tabKey.wasPressedThisFrame;
#else
        return Input.GetKeyDown(KeyCode.Tab);
#endif
    }

    private static bool RefocusPressed()
    {
#if ENABLE_INPUT_SYSTEM
        return Keyboard.current != null && Keyboard.current.fKey.wasPressedThisFrame;
#else
        return Input.GetKeyDown(KeyCode.F);
#endif
    }
}
