using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace OptiSort.Classes
{
    internal class PythonProcessRunner
    {
        private Process _process;
        private bool _isRunning;

        public event Action<string> OnOutputReceived;
        public event Action<string> OnErrorReceived;
        public event Action<bool> OnExecutionTerminated;

        public int RunPythonScript(string scriptPath)
        {
            if (_isRunning)
            {
                OnErrorReceived?.Invoke("Process already running.");
                return 0;
            }

            string pythonExe = LocatePythonInterpreter();
            if (string.IsNullOrEmpty(pythonExe))
            {
                OnErrorReceived?.Invoke("Python interpreter not found.");
                return 0;
            }

            if (!File.Exists(scriptPath))
            {
                OnErrorReceived?.Invoke("Python script not found.");
                return 0;
            }

            var processStartInfo = new ProcessStartInfo
            {
                FileName = pythonExe,
                Arguments = $"\"{scriptPath}\"",  // Add explicit quotes around the script path (to solve issues caused by whitespaces in the file path)
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true,
                WorkingDirectory = Path.GetDirectoryName(scriptPath)
            };

            _process = new Process { StartInfo = processStartInfo };

            _process.OutputDataReceived += (sender, e) =>
            {
                if (!string.IsNullOrEmpty(e.Data))
                    OnOutputReceived?.Invoke(e.Data);
            };

            StringBuilder errorBuffer = new StringBuilder();
            _process.ErrorDataReceived += (sender, e) =>
            {
                if (!string.IsNullOrEmpty(e.Data))
                {
                    errorBuffer.AppendLine(e.Data);
                }
            };

            _process.EnableRaisingEvents = true;
            _process.Exited += (sender, e) =>
            {
                _isRunning = false;
                if (errorBuffer.Length > 0)
                {
                    string fullError = errorBuffer.ToString();
                    Console.WriteLine($"Python Error:\n{fullError}"); // Debugging
                    OnErrorReceived?.Invoke(fullError);
                }
                OnExecutionTerminated?.Invoke(true);
            };

            _isRunning = true;
            _process.Start();
            _process.BeginOutputReadLine();
            _process.BeginErrorReadLine();

            return _process.Id;
        }

        public void Stop()
        {
            if (_isRunning && _process != null && !_process.HasExited)
            {
                _process.Kill();
                _isRunning = false;
                OnExecutionTerminated?.Invoke(true);
            }
        }

        private string LocatePythonInterpreter()
        {
            // Attempt to find Python in the PATH
            var processStartInfo = new ProcessStartInfo
            {
                FileName = "cmd.exe",
                Arguments = "/c python --version",
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false,
                CreateNoWindow = true
            };

            try
            {
                using (var process = Process.Start(processStartInfo))
                {
                    process.WaitForExit();
                    string output = process.StandardOutput.ReadToEnd();
                    if (!string.IsNullOrWhiteSpace(output) && output.Contains("Python"))
                    {
                        return "python"; // Python was found in the PATH
                    }
                }
            }
            catch
            {
                // Ignore errors and fall back to alternative methods
            }

            OnErrorReceived?.Invoke("Python interpreted not found; Double check installation and PATH configuration");
            return null;
        }
    }
}
