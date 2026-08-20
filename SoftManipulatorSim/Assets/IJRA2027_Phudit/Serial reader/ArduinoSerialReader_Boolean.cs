using UnityEngine;
using System.IO.Ports;

public class ArduinoSerialReader : MonoBehaviour
{
    [Header("Serial Port Settings")]
    [Tooltip("Change this to match your Arduino Mega's COM port (e.g., COM3, COM4, etc.)")]
    public string portName = "COM3";
    public int baudRate = 9600;

    [Header("Shared Load Cell States (Read-Only)")]
    public bool loadCellRightPressed = false;
    public bool loadCellLeftPressed = false;
    public bool loadCellUpPressed = false;
    public bool loadCellDownPressed = false;

    private SerialPort stream;

    void Start()
    {
        stream = new SerialPort(portName, baudRate);
        
        try
        {
            stream.Open();
            stream.ReadTimeout = 50; 
            Debug.Log($"Serial Port {portName} opened successfully.");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Could not open serial port {portName}: {e.Message}");
        }
    }

    void Update()
    {
        if (stream != null && stream.IsOpen)
        {
            try
            {
                string incomingString = stream.ReadLine(); 
                string[] data = incomingString.Split(',');

                if (data.Length == 4)
                {
                    // Save the booleans directly to THIS script instance
                    loadCellRightPressed = (data[0] == "1");
                    loadCellLeftPressed  = (data[1] == "1");
                    loadCellUpPressed    = (data[2] == "1");
                    loadCellDownPressed  = (data[3] == "1");
                }
            }
            catch (System.TimeoutException)
            {
                // Normal timeout handling to prevent Unity main thread lockups
            }
        }
    }

    void OnApplicationQuit()
    {
        if (stream != null && stream.IsOpen)
        {
            stream.Close();
            Debug.Log("Serial Port safely closed.");
        }
    }
}