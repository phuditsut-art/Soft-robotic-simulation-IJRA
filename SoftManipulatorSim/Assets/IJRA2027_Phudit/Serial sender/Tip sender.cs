using UnityEngine;
using System.IO.Ports;

public class TipPositionSender : MonoBehaviour
{
    [Header("Serial Port Settings")]
    public string portName = "COM3"; // Change this to your outgoing virtual COM port
    public int baudRate = 9600;
    
    private SerialPort serialPort;

    void Start()
    {
        // Initialize the serial port
        serialPort = new SerialPort(portName, baudRate);
        
        try
        {
            serialPort.Open();
            Debug.Log($"Successfully opened {portName}");
            
            // Call the SendPosition method immediately, then every 0.2 seconds
            InvokeRepeating(nameof(SendPosition), 0f, 0.2f);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to open serial port {portName}: {e.Message}");
        }
    }

    void SendPosition()
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            // transform.position returns the position relative to the world origin (0,0,0)
            Vector3 pos = transform.position;
            
            // Format the data as a comma-separated string: X,Y,Z
            string dataString = $"{pos.x},{pos.y},{pos.z}";
            
            // Send over serial
            serialPort.WriteLine(dataString);
        }
    }

    // Ensure the port is closed when the game stops to prevent locking the COM port
    void OnDestroy()
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
        }
    }
}