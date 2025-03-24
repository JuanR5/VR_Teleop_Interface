using UnityEngine;
using Renci.SshNet;
using System.Threading;
using System;

public class SSHRunnerFranka : MonoBehaviour
{
    // SSH credentials and IP (set in the Inspector)
    [SerializeField]
    private string username = "nakama";
    [SerializeField]
    private string password = "password";
    [SerializeField]
    private string host = "10.4.0.7";

    // Command to run on the remote PC
    private string command = "cd JPRA/franka_teleop && docker compose up";

    // SSH client and cancellation token
    private SshClient sshClient;
    private CancellationTokenSource cancellationTokenSource;

    void Start()
    {
        RunSSHCommand();
    }

    void RunSSHCommand()
    {
        Debug.Log("Attempting SSH connection...");
        // Initialize the SSH client with the specified host, username, and password
        sshClient = new SshClient(host, username, password);

        try
        {
            // Connect to the remote server
            sshClient.Connect();
            Debug.Log("SSH connection established.");

            // Create a cancellation token source
            cancellationTokenSource = new CancellationTokenSource();

            // Run the command asynchronously
            RunCommandAsync(command, cancellationTokenSource.Token);
        }
        catch (Exception e)
        {
            Debug.LogError("SSH connection failed: " + e.Message);
        }
    }

    async void RunCommandAsync(string command, CancellationToken cancellationToken)
    {
        try
        {
            // Create and execute the SSH command
            var sshCommand = sshClient.CreateCommand(command);
            var asyncResult = sshCommand.BeginExecute();

            // Wait until the command completes or cancellation is requested
            while (!asyncResult.IsCompleted && !cancellationToken.IsCancellationRequested)
            {
                await System.Threading.Tasks.Task.Yield();
            }

            if (cancellationToken.IsCancellationRequested)
            {
                // If cancelled, cancel the SSH command execution
                sshCommand.CancelAsync();
                Debug.Log("SSH command canceled.");
            }
            else
            {
                // Retrieve and log the output from the command
                string output = sshCommand.EndExecute(asyncResult);
                Debug.Log("Command output: " + output);
            }

            // Optionally keep the connection alive while the simulation is running
            while (!cancellationToken.IsCancellationRequested)
            {
                await System.Threading.Tasks.Task.Delay(1000, cancellationToken); // Wait for 1 second
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Command execution failed: " + e.Message);
        }
        finally
        {
            // Disconnect the SSH client if still connected
            if (sshClient != null && sshClient.IsConnected)
            {
                sshClient.Disconnect();
                Debug.Log("SSH connection closed.");
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
        // Cancel the running command if necessary
        if (cancellationTokenSource != null && !cancellationTokenSource.IsCancellationRequested)
        {
            cancellationTokenSource.Cancel();
        }

        // Disconnect and dispose the SSH client if it's still connected
        if (sshClient != null && sshClient.IsConnected)
        {
            sshClient.Disconnect();
            sshClient.Dispose();
        }
    }
}
