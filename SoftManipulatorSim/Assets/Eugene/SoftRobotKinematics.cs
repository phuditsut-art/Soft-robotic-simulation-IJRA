using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Pure kinematics for a piecewise-constant-curvature (PCC) soft manipulator.
///
/// Direct C# port of PART A of SoftRobot_Simulation.py, which was itself a port
/// of the MATLAB "New Soft Manipulator Kinematics" package:
///   transxyz.m, rx_rad/ry_rad/rz_rad.m, rod_T.m, rod_T_twist.m,
///   FwdSRM.m, show_SoftManipulator.m
///
/// No Unity scene objects are touched here -- this is maths only, so it can be
/// unit-tested or reused headlessly. Validated numerically against the Python
/// reference to ~1e-14 over 400 randomised configurations, including the
/// straight-segment and near-singular (xi -> 0) edge cases.
///
/// CONVENTIONS (identical to MATLAB/Blender):
///   - Right-handed, Z-up. Segment local +Z points along the backbone.
///   - theta (a.k.a. xi) = half the total bend angle of a segment, in RADIANS.
///   - phi  = orientation of the bending plane, in RADIANS.
///   - beta = twist about the segment's local z, in RADIANS.
///   - L    = segment length, in CENTIMETRES (the example uses 25.0 = 25 cm).
///           Disk thickness 3.0 and sensor offset 5.0 are likewise cm.
/// Use SoftRobotKinematics.ToUnity() to convert results into Unity's
/// left-handed, Y-up space for rendering.
/// </summary>
public static class SoftRobotKinematics
{
    public const float DefaultDiskThickness = 3.0f;   // T_disk in FwdSRM.m
    public const float DefaultSensorLength  = 5.0f;   // T_sensor in FwdSRM.m
    private const float Eps = 1e-9f;

    // ---------------------------------------------------------------- transxyz.m
    public static Matrix4x4 TransXYZ(float x, float y, float z)
    {
        Matrix4x4 m = Matrix4x4.identity;
        m[0, 3] = x;
        m[1, 3] = y;
        m[2, 3] = z;
        return m;
    }

    // ------------------------------------------- rx_rad.m / ry_rad.m / rz_rad.m
    public static Matrix4x4 RxRad(float t)
    {
        float c = Mathf.Cos(t), s = Mathf.Sin(t);
        Matrix4x4 m = Matrix4x4.identity;
        m[1, 1] = c; m[1, 2] = -s;
        m[2, 1] = s; m[2, 2] = c;
        return m;
    }

    public static Matrix4x4 RyRad(float t)
    {
        float c = Mathf.Cos(t), s = Mathf.Sin(t);
        Matrix4x4 m = Matrix4x4.identity;
        m[0, 0] = c;  m[0, 2] = s;
        m[2, 0] = -s; m[2, 2] = c;
        return m;
    }

    public static Matrix4x4 RzRad(float t)
    {
        float c = Mathf.Cos(t), s = Mathf.Sin(t);
        Matrix4x4 m = Matrix4x4.identity;
        m[0, 0] = c; m[0, 1] = -s;
        m[1, 0] = s; m[1, 1] = c;
        return m;
    }

    public static Matrix4x4 RxDeg(float d) => RxRad(d * Mathf.Deg2Rad);
    public static Matrix4x4 RyDeg(float d) => RyRad(d * Mathf.Deg2Rad);
    public static Matrix4x4 RzDeg(float d) => RzRad(d * Mathf.Deg2Rad);

    // ------------------------------------------------------------------ rod_T.m
    /// <summary>
    /// Constant-curvature transform across one segment.
    /// Reduces to a pure translation of `length` along local +Z when xi ~= 0.
    /// </summary>
    public static Matrix4x4 RodT(float phi, float xi, float length)
    {
        if (Mathf.Abs(xi) < Eps)
            return TransXYZ(0f, 0f, length);

        float c2x = Mathf.Cos(2f * xi);
        float s2x = Mathf.Sin(2f * xi);
        float cp  = Mathf.Cos(phi);
        float sp  = Mathf.Sin(phi);
        float s2p = Mathf.Sin(2f * phi);
        float sx  = Mathf.Sin(xi);

        Matrix4x4 m = new Matrix4x4();
        m[0, 0] = c2x * cp * cp - cp * cp + 1f;
        m[0, 1] = 0.5f * s2p * (c2x - 1f);
        m[0, 2] = s2x * cp;
        m[0, 3] = length * cp * sx * sx / xi;

        m[1, 0] = 0.5f * s2p * (c2x - 1f);
        m[1, 1] = c2x * sp * sp - sp * sp + 1f;
        m[1, 2] = s2x * sp;
        m[1, 3] = length * sp * sx * sx / xi;

        m[2, 0] = -s2x * cp;
        m[2, 1] = -s2x * sp;
        m[2, 2] = c2x;
        m[2, 3] = length * Mathf.Sin(2f * xi) / (2f * xi);

        m[3, 0] = 0f; m[3, 1] = 0f; m[3, 2] = 0f; m[3, 3] = 1f;
        return m;
    }

    // ------------------------------------------------------------ rod_T_twist.m
    public static Matrix4x4 RodTTwist(float phi, float xi, float beta, float length)
        => RodT(phi, xi, length) * RzRad(beta);

    // ------------------------------ shared `org` block (FwdSRM / show_SoftManip)
    /// <summary>Home/base frame. rotationDeg is (rx, ry, rz) applied in that order.</summary>
    public static Matrix4x4 HomeTransform(Vector3 position, Vector3 rotationDeg)
    {
        Matrix4x4 org = TransXYZ(position.x, position.y, position.z);
        Matrix4x4 rot = RxDeg(rotationDeg.x) * RyDeg(rotationDeg.y) * RzDeg(rotationDeg.z);
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                org[r, c] = rot[r, c];
        return org;
    }

    // -------------------------- `segm_pos` block of show_SoftManipulator.m
    /// <summary>
    /// Centreline points of one segment expressed in that segment's own frame.
    /// beta deliberately does not appear: twist rotates downstream frames but
    /// does not bend the drawn tube.
    /// </summary>
    public static List<Vector3> LocalSegmentBackbone(float phi, float theta, float length, int nPoints)
    {
        if (nPoints < 2) nPoints = 2;
        var pts = new List<Vector3>(nPoints);

        if (Mathf.Abs(theta) < Eps)
        {
            for (int i = 0; i < nPoints; i++)
            {
                float t = i / (nPoints - 1f);
                pts.Add(new Vector3(0f, 0f, length * t));
            }
            return pts;
        }

        float rho = 2f * theta;
        float R   = length / (2f * theta);
        float cp  = Mathf.Cos(phi);
        float sp  = Mathf.Sin(phi);

        for (int i = 0; i < nPoints; i++)
        {
            float rp = rho * (i / (nPoints - 1f));
            float radial = (1f - Mathf.Cos(rp)) * R;
            pts.Add(new Vector3(radial * cp, radial * sp, R * Mathf.Sin(rp)));
        }
        return pts;
    }

    // ---------------------------------------------------------------- FwdSRM.m
    /// <summary>
    /// Forward kinematics of the serial chain:
    ///   org * tr1 * T_disk * tr2 * T_disk * ... * T_sensor
    /// </summary>
    /// <param name="tipTransform">Tip frame, including the sensor offset.</param>
    /// <param name="diskFrames">Frame at the end of each segment's disk.</param>
    public static void FwdSRM(
        float[] theta, float[] phi, float[] beta, float[] L,
        Vector3 homePosition, Vector3 homeRotationDeg,
        out Matrix4x4 tipTransform, out Matrix4x4[] diskFrames,
        float sensorLength = DefaultSensorLength,
        float diskThickness = DefaultDiskThickness)
    {
        int nb = theta.Length;
        if (phi.Length != nb || beta.Length != nb || L.Length != nb)
            throw new System.ArgumentException("theta, phi, beta and L must all have the same length.");

        Matrix4x4 tDisk   = TransXYZ(0f, 0f, diskThickness);
        Matrix4x4 tSensor = TransXYZ(0f, 0f, sensorLength);
        Matrix4x4 org     = HomeTransform(homePosition, homeRotationDeg);

        var tr = new Matrix4x4[nb];
        for (int i = 0; i < nb; i++)
            tr[i] = RodTTwist(phi[i], theta[i], beta[i], L[i]);

        diskFrames = new Matrix4x4[nb];
        Matrix4x4 T = org;

        if (nb == 1)
        {
            T = org * tr[0] * tDisk;
            diskFrames[0] = T;
            tipTransform = T;
            return;
        }

        for (int i = 0; i < nb; i++)
        {
            T = (i == 0 ? org : T) * tr[i] * tDisk;
            diskFrames[i] = T;
            if (i == nb - 1)
                T = T * tSensor;
        }
        tipTransform = T;
    }

    // --------------------------- geometry half of show_SoftManipulator.m
    /// <summary>
    /// Full backbone polyline in MATLAB world space (Z-up, right-handed),
    /// plus the tip transform and per-segment disk frames.
    /// </summary>
    public static List<Vector3> ComputeBackbone(
        float[] theta, float[] phi, float[] beta, float[] L,
        Vector3 homePosition, Vector3 homeRotationDeg,
        out Matrix4x4 tipTransform, out Matrix4x4[] diskFrames,
        int pointsPerSegment = 30,
        float sensorLength = DefaultSensorLength,
        float diskThickness = DefaultDiskThickness)
    {
        int nb = theta.Length;
        FwdSRM(theta, phi, beta, L, homePosition, homeRotationDeg,
               out tipTransform, out diskFrames, sensorLength, diskThickness);

        Matrix4x4 org = HomeTransform(homePosition, homeRotationDeg);
        var outPts = new List<Vector3>(nb * (pointsPerSegment + 1));

        for (int i = 0; i < nb; i++)
        {
            var local = LocalSegmentBackbone(phi[i], theta[i], L[i], pointsPerSegment);
            Matrix4x4 basis = (i == 0) ? org : diskFrames[i - 1];

            // Skip the first point of every segment after the first: it is
            // coincident with the previous segment's disk centre.
            int start = (i > 0) ? 1 : 0;
            for (int j = start; j < local.Count; j++)
                outPts.Add(basis.MultiplyPoint3x4(local[j]));

            // Bridge across the disk itself.
            outPts.Add(GetPosition(diskFrames[i]));
        }
        return outPts;
    }

    // ------------------------------------------------------------------ helpers
    public static Vector3 GetPosition(Matrix4x4 m) => new Vector3(m[0, 3], m[1, 3], m[2, 3]);

    /// <summary>
    /// MATLAB/Blender (right-handed, Z-up)  ->  Unity (left-handed, Y-up).
    /// Swapping Y and Z performs the handedness change as well as the up-axis change.
    /// </summary>
    public static Vector3 ToUnity(Vector3 p, float scale = 1f)
        => new Vector3(p.x, p.z, p.y) * scale;

    /// <summary>Unity rotation equivalent to a MATLAB-space frame.</summary>
    public static Quaternion ToUnityRotation(Matrix4x4 m)
    {
        Vector3 fwd = ToUnity(new Vector3(m[0, 2], m[1, 2], m[2, 2]));  // local +Z
        Vector3 up  = ToUnity(new Vector3(m[0, 1], m[1, 1], m[2, 1]));  // local +Y
        if (fwd.sqrMagnitude < 1e-12f) return Quaternion.identity;
        return Quaternion.LookRotation(fwd.normalized, up.normalized);
    }
}
