using System.Diagnostics;
using UnityEngine;

public class SSHRunner : MonoBehaviour
{
    // Path to Plink executable (set in the Inspector)
    [SerializeField]
    private string plinkPath = @"C:\Tools\plink.exe";

    // SSH credentials and IP (set in the Inspector)
    [SerializeField]
    private string username = "nakama";
    [SerializeField]
    private string password = "eve";
    [SerializeField]
    private string host = "10.4.0.11";

    // Command to run on the remote PC
    private string command = "cd VR_Interface/simple_ws && docker compose up -d";

    void Start()
    {
        RunSSHCommand();
    }

    void RunSSHCommand()
    {
        // Create the Plink command string
        string plinkCommand = $"-ssh {username}@{host} -pw {password} {command}";

        // Set up the process
        ProcessStartInfo processInfo = new ProcessStartInfo
        {
            FileName = plinkPath,
            Arguments = plinkCommand,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            CreateNoWindow = true
        };

        // Start the process
        using (Process process = Process.Start(processInfo))
        {
            // Read the output (optional)
            string output = process.StandardOutput.ReadToEnd();
            string error = process.StandardError.ReadToEnd();

            process.WaitForExit();

            // Log the output
            Debug.Log("Output: " + output);
            if (!string.IsNullOrEmpty(error))
            {
                Debug.LogError("Error: " + error);
            }
        }
    }
}
