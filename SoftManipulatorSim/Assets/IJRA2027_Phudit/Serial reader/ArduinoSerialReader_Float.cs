using UnityEngine;
using System.IO.Ports;
using System.Globalization;

public class ArduinoSerialReader_Float : MonoBehaviour
{
    [Header("Serial Port Settings")]
    [Tooltip("Change this to match your Arduino Mega's COM port (e.g., COM3, COM4, etc.)")]
    public string portName = "COM3";
    public int baudRate = 19200; 

    [Header("Shared Load Cell States (Read-Only)")]
    public float loadCellRightValue = 0f;
    public float loadCellLeftValue = 0f;
    public float loadCellUpValue = 0f;
    public float loadCellDownValue = 0f;

    private SerialPort stream;

    void Start()
    {
        stream = new SerialPort(portName, baudRate);
        
        try
        {
            stream.Open();
            stream.ReadTimeout = 15; // Lowered timeout to prevent Unity from freezing
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
                // ONLY try to read if there is actually data waiting
                if (stream.BytesToRead > 0)
                {
                    string incomingString = "";
                    
                    // Flush the buffer: read ALL lines waiting in the queue, 
                    // overwriting incomingString so we only keep the absolute newest one.
                    while (stream.BytesToRead > 0)
                    {
                        incomingString = stream.ReadLine();
                    }

                    // Process the freshest data
                    if (!string.IsNullOrEmpty(incomingString))
                    {
                        incomingString = incomingString.Trim(); // Clean off hidden \r\n characters
                        string[] data = incomingString.Split(',');

                        if (data.Length == 4)
                        {
                            // Safely parse the CSV strings into float variables
                            float.TryParse(data[0], NumberStyles.Float, CultureInfo.InvariantCulture, out loadCellRightValue);
                            float.TryParse(data[1], NumberStyles.Float, CultureInfo.InvariantCulture, out loadCellLeftValue);
                            float.TryParse(data[2], NumberStyles.Float, CultureInfo.InvariantCulture, out loadCellUpValue);
                            float.TryParse(data[3], NumberStyles.Float, CultureInfo.InvariantCulture, out loadCellDownValue);
                        }
                    }
                }
            }
            catch (System.TimeoutException)
            {
                // Normal timeout handling to prevent Unity main thread lockups
            }
            catch (System.Exception e)
            {
                Debug.LogWarning("Serial Reader Error: " + e.Message);
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