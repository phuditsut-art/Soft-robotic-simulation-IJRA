using UnityEngine;
using System.IO.Ports;

public class TipHapticFeedback : MonoBehaviour
{
    [Header("Serial Connection to Seeed Studio XIAO")]
    [Tooltip("The COM port number connected to your XIAO vibrator controller (e.g., 8 for COM8).")]
    public int portNumber = 8; 
    public int baudRate = 9600;

    private SerialPort xiaoStream;
    private string lastSentCommand = "S";
    private string xiaoPortName;

    void Start()
    {
        // Automatically format the integer into a Windows COM port string
        // Note: If using Mac/Linux, change this to something like "/dev/ttyUSB" + portNumber
        xiaoPortName = "COM" + portNumber; 

        // Establish connection to the Seeed Studio board
        xiaoStream = new SerialPort(xiaoPortName, baudRate);
        try
        {
            xiaoStream.Open();
            Debug.Log($"Haptic System: Successfully connected to XIAO on {xiaoPortName}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Could not open XIAO haptic port {xiaoPortName}: {e.Message}");
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        // STAGE 1: Check if Unity's physics engine even recognizes the physical overlap
        Debug.Log($"[STAGE 1: PHYSICS] Physical overlap detected with: {other.gameObject.name}");

        // STAGE 2: Hierarchy & Component Safety Validation
        if (other.isTrigger)
        {
            Debug.LogWarning($"[STAGE 2: FAILED] Ignored because {other.gameObject.name} is marked as 'Is Trigger'.");
            return;
        }

        if (other.transform.IsChildOf(transform.root))
        {
            Debug.LogWarning($"[STAGE 2: FAILED] Ignored because {other.gameObject.name} is structurally a part of this robot arm architecture.");
            return;
        }

        Debug.Log($"[STAGE 3: PASSED] Safety filters cleared. Calculating spatial intersection...");

        Vector3 contactPoint;

        // Extract contact vector points based on surface type
        if (other is BoxCollider || other is SphereCollider || other is CapsuleCollider || (other is MeshCollider meshCol && meshCol.convex))
        {
            contactPoint = other.ClosestPoint(transform.position);
        }
        else
        {
            // Bounding box override fallback for complex environment geometries
            contactPoint = other.bounds.ClosestPoint(transform.position);
        }
        
        // Transform absolute global coordinates into local tip space matrices
        Vector3 localContact = transform.InverseTransformPoint(contactPoint);
        
        // STAGE 4: Mathematical Vector Readout
        Debug.Log($"[STAGE 4: MATH] Local Contact Points -> X: {localContact.x:F2}, Y: {localContact.y:F2}, Z: {localContact.z:F2}");

        string directionSignal = "S";

        // Calculate absolute values to locate the dominant vector direction
        float absX = Mathf.Abs(localContact.x);
        float absY = Mathf.Abs(localContact.y);
        float absZ = Mathf.Abs(localContact.z);

        // Map dominant spatial interaction to directions
        if (absX > absY && absX > absZ)
        {
            directionSignal = (localContact.x > 0) ? "R" : "L"; // X-Axis tracking
        }
        else if (absY > absX && absY > absZ)
        {
            directionSignal = (localContact.y > 0) ? "U" : "D"; // Y-Axis tracking
        }
        else
        {
            directionSignal = (localContact.z > 0) ? "U" : "D"; // Z-Axis tracking fallback
        }

        // STAGE 5: Decision Matrix Completed
        Debug.Log($"[STAGE 5: COMPLETE] Target direction calculated as: {directionSignal}. Passing to serial stream...");
        SendHapticToMotors(directionSignal);
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.isTrigger || other.transform.IsChildOf(transform.root)) return;
        
        Debug.Log($"[HITBOX EXIT] Disconnected from: {other.gameObject.name}. Halting haptics.");
        SendHapticToMotors("S"); 
    }

    private void SendHapticToMotors(string command)
    {
        // De-duplicate commands to prevent transmission line buffer flooding
        if (command != lastSentCommand)
        {
            if (xiaoStream != null && xiaoStream.IsOpen)
            {
                xiaoStream.WriteLine(command); 
                lastSentCommand = command;
                Debug.Log($"[SERIAL TRANSMITTED] Successfully pushed string to hardware -> {command}");
            }
            else
            {
                Debug.LogError($"[SERIAL ERROR] Cannot send command '{command}'. COM Port connection status: Closed.");
            }
        }
    }

    void OnApplicationQuit()
    {
        if (xiaoStream != null && xiaoStream.IsOpen)
        {
            xiaoStream.WriteLine("S"); // Standard stop bit safety sequence
            xiaoStream.Close();
            Debug.Log("Haptic serial port pipeline cleanly flushed and closed.");
        }
    }
}