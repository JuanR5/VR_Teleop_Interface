using UnityEngine;
using Renci.SshNet;
using System.Threading;
using System;

public class SSHRunner : MonoBehaviour
{
    // SSH credentials and IP (set in the Inspector)
    [SerializeField]
    private string username = "nakama";
    [SerializeField]
    private string password = "password";
    [SerializeField]
    private string host = "10.4.0.11";

    // Command to run on the remote PC
    private string command = "cd VR_Interface/simple_ws && docker compose up -d";

    // SSH client and cancellation token
    private SshClient sshClient;
    private CancellationTokenSource cancellationTokenSource;

    void Start()
    {
        RunSSHCommand();
    }

    void RunSSHCommand()
    {
        UnityEngine.Debug.Log("SSH connection trying.");
        // Initialize the SSH client
        sshClient = new SshClient(host, username, password);

        try
        {
            // Connect to the remote server
            sshClient.Connect();
            UnityEngine.Debug.Log("SSH connection established.");

            // Create a cancellation token source
            cancellationTokenSource = new CancellationTokenSource();

            // Run the command asynchronously
            RunCommandAsync(command, cancellationTokenSource.Token);
        }
        catch (Exception e)
        {
            UnityEngine.Debug.LogError("SSH connection failed: " + e.Message);
        }
    }

    async void RunCommandAsync(string command, CancellationToken cancellationToken)
    {
        try
        {
            // Create and run the command
            var sshCommand = sshClient.CreateCommand(command);
            var asyncResult = sshCommand.BeginExecute();

            // Wait for the command to complete or cancellation
            while (!asyncResult.IsCompleted && !cancellationToken.IsCancellationRequested)
            {
                await System.Threading.Tasks.Task.Yield();
            }

            if (cancellationToken.IsCancellationRequested)
            {
                // Cancel the command
                sshCommand.CancelAsync();
                UnityEngine.Debug.Log("SSH command canceled.");
            }
            else
            {
                // Command completed
                string output = sshCommand.EndExecute(asyncResult);
                UnityEngine.Debug.Log("Command output: " + output);
            }

            // Keep the connection alive while the simulation is running
            while (!cancellationToken.IsCancellationRequested)
            {
                await System.Threading.Tasks.Task.Delay(1000, cancellationToken); // Wait for 1 second
            }
        }
        catch (Exception e)
        {
            UnityEngine.Debug.LogError("Command execution failed: " + e.Message);
        }
        finally
        {
            // Disconnect the SSH client
            if (sshClient != null && sshClient.IsConnected)
            {
                sshClient.Disconnect();
                UnityEngine.Debug.Log("SSH connection closed.");
            }
        }
    }

    void OnApplicationQuit()
    {
        CancelSSHCommand();
    }

    void OnDestroy()
    {
        CancelSSHCommand();
    }

    void CancelSSHCommand()
    {
        // Cancel the command if it's running
        if (cancellationTokenSource != null && !cancellationTokenSource.IsCancellationRequested)
        {
            cancellationTokenSource.Cancel();
        }

        // Dispose the SSH client
        if (sshClient != null && sshClient.IsConnected)
        {
            sshClient.Disconnect();
            sshClient.Dispose();
        }
    }
}