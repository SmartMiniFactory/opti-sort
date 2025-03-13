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

        public void RunPythonScript(string scriptPath)
        {
            if (_isRunning)
            {
                OnErrorReceived?.Invoke("Process already running.");
                return;
            }

            string pythonExe = LocatePythonInterpreter();
            if (string.IsNullOrEmpty(pythonExe))
            {
                OnErrorReceived?.Invoke("Python interpreter not found.");
                return;
            }

            if (!File.Exists(scriptPath))
            {
                OnErrorReceived?.Invoke("Python script not found.");
                return;
            }

            var processStartInfo = new ProcessStartInfo
            {
                FileName = pythonExe,
                Arguments = scriptPath,
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true
            };

            _process = new Process { StartInfo = processStartInfo };

            _process.OutputDataReceived += (sender, e) =>
            {
                if (!string.IsNullOrEmpty(e.Data))
                    OnOutputReceived?.Invoke(e.Data);
            };

            _process.ErrorDataReceived += (sender, e) =>
            {
                if (!string.IsNullOrEmpty(e.Data))
                    OnErrorReceived?.Invoke(e.Data);
            };

            _process.EnableRaisingEvents = true;
            _process.Exited += (sender, e) =>
            {
                _isRunning = false;
                OnExecutionTerminated?.Invoke(true);
            };

            _isRunning = true;
            _process.Start();
            _process.BeginOutputReadLine();
            _process.BeginErrorReadLine();
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
