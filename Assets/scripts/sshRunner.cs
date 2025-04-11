using UnityEngine;
using Renci.SshNet;
using System.Threading;
using System;

/// <summary>
/// Handles establishing an SSH connection to a remote machine and executing a Docker Compose command asynchronously.
/// </summary>
public class SSHRunner : MonoBehaviour
{
    /// <summary>
    /// SSH username to connect with (set via Inspector).
    /// </summary>
    [SerializeField] private string username = "nakama";

    /// <summary>
    /// SSH password to authenticate (set via Inspector).
    /// </summary>
    [SerializeField] private string password = "password";

    /// <summary>
    /// Host IP of the remote machine (set via Inspector).
    /// </summary>
    [SerializeField] private string host = "10.4.0.11";

    /// <summary>
    /// The command to be executed remotely via SSH.
    /// </summary>
    private string command = "cd VR_Interface/simple_ws && docker compose up -d";

    private SshClient sshClient;
    private CancellationTokenSource cancellationTokenSource;

    /// <summary>
    /// Unity Start() method. Triggers the SSH connection and command execution.
    /// </summary>
    void Start()
    {
        RunSSHCommand();
    }

    /// <summary>
    /// Initiates the SSH connection and starts the asynchronous command execution.
    /// </summary>
    void RunSSHCommand()
    {
        Debug.Log("SSH connection trying.");
        sshClient = new SshClient(host, username, password);

        try
        {
            sshClient.Connect();
            Debug.Log("SSH connection established.");

            cancellationTokenSource = new CancellationTokenSource();
            RunCommandAsync(command, cancellationTokenSource.Token);
        }
        catch (Exception e)
        {
            Debug.LogError("SSH connection failed: " + e.Message);
        }
    }

    /// <summary>
    /// Asynchronously executes the SSH command and monitors cancellation.
    /// </summary>
    /// <param name="command">The command to execute on the remote machine.</param>
    /// <param name="cancellationToken">Cancellation token to stop execution if needed.</param>
    async void RunCommandAsync(string command, CancellationToken cancellationToken)
    {
        try
        {
            var sshCommand = sshClient.CreateCommand(command);
            var asyncResult = sshCommand.BeginExecute();

            while (!asyncResult.IsCompleted && !cancellationToken.IsCancellationRequested)
            {
                await System.Threading.Tasks.Task.Yield();
            }

            if (cancellationToken.IsCancellationRequested)
            {
                sshCommand.CancelAsync();
                Debug.Log("SSH command canceled.");
            }
            else
            {
                string output = sshCommand.EndExecute(asyncResult);
                Debug.Log("Command output: " + output);
            }

            // Keep alive while running
            while (!cancellationToken.IsCancellationRequested)
            {
                await System.Threading.Tasks.Task.Delay(1000, cancellationToken);
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Command execution failed: " + e.Message);
        }
        finally
        {
            if (sshClient != null && sshClient.IsConnected)
            {
                sshClient.Disconnect();
                Debug.Log("SSH connection closed.");
            }
        }
    }

    /// <summary>
    /// Ensures SSH cleanup on application quit.
    /// </summary>
    void OnApplicationQuit()
    {
        CancelSSHCommand();
    }

    /// <summary>
    /// Ensures SSH cleanup on component destroy.
    /// </summary>
    void OnDestroy()
    {
        CancelSSHCommand();
    }

    /// <summary>
    /// Cancels the running SSH command and safely disposes the client.
    /// </summary>
    void CancelSSHCommand()
    {
        if (cancellationTokenSource != null && !cancellationTokenSource.IsCancellationRequested)
        {
            cancellationTokenSource.Cancel();
        }

        if (sshClient != null && sshClient.IsConnected)
        {
            sshClient.Disconnect();
            sshClient.Dispose();
        }
    }
}
