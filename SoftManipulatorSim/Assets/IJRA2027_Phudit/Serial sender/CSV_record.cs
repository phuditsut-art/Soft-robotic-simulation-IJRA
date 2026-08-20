using UnityEngine;
using System.IO;

public class DirectCSVLogger : MonoBehaviour
{
    [Header("Target Setup")]
    [Tooltip("Drag the SR_Tip object here from the Hierarchy")]
    public Transform targetToTrack; 

    [Header("File Settings")]
    public string fileName = "TipPositionLog.csv";
    private string filePath;
    
    // Using a StreamWriter is faster and safer for continuous logging
    private StreamWriter writer;

    void Start()
    {
        // Safety check to ensure you assigned the tip
        if (targetToTrack == null)
        {
            Debug.LogError("No target assigned! Please drag SR_Tip into the Target To Track slot in the Inspector.");
            return; 
        }

        // This creates the file directly in your Unity project's "Assets" folder
        filePath = Path.Combine(Application.dataPath, fileName);
        
        // IMPORTANT: The 'false' parameter tells Unity to OVERWRITE the file.
        // This clears out all data from the previous run and starts fresh.
        writer = new StreamWriter(filePath, false); 
        
        // Write the header row at the very top of the fresh file
        writer.WriteLine("Timestamp,X,Y,Z");
        
        // Start recording every 0.2 seconds
        InvokeRepeating(nameof(LogPosition), 0f, 0.002f);
        
        Debug.Log("Created fresh CSV and logging tip data to: " + filePath);
    }

    void LogPosition()
    {
        if (targetToTrack != null && writer != null)
        {
            // .position always returns the absolute world space coordinates 
            // relative to the Unity space origin (0,0,0)
            Vector3 pos = targetToTrack.position;
            
            // Get the current time
            string timestamp = System.DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff");
            
            // Format the line and write it directly to the open stream
            writer.WriteLine($"{timestamp},{pos.x},{pos.y},{pos.z}");
        }
    }

    // These methods ensure the file safely saves and closes when you stop playing
    void OnApplicationQuit()
    {
        CloseFile();
    }

    void OnDestroy()
    {
        CloseFile();
    }

    void CloseFile()
    {
        if (writer != null)
        {
            writer.Close();
            writer = null;
        }
    }
}