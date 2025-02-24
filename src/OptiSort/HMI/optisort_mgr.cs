using Ace.Core.Util;
using Ace.UIBuilder.Client.Controls.Tools.WindowsForms;
using FlexibowlLibrary;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace OptiSort
{

    internal class optisort_mgr : INotifyPropertyChanged
    {
        public MQTT MqttClient { get; set; }
        public Cobra600 Cobra600 { get; set; }
        public Flexibowl Flexibowl { get; set; }


        // status
        public event PropertyChangedEventHandler PropertyChanged; // declare propertyChanged event (required by the associated interface)
        private bool _statusScara = false;
        private bool _statusScaraEmulation = true;
        private bool _statusFelxibowl = false;
        private bool _statusMqttClient = false;
        private string _streamingTopic = null;
        private bool _requestScreenshots = false;


        // custom defined properties to trigger events on status changed
        public bool StatusScara
        {
            get { return _statusScara; }
            set
            {
                if (_statusScara != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusScara = value;
                    OnPropertyChanged(nameof(StatusScara));
                }
            }
        }
        public bool StatusFlexibowl
        {
            get { return _statusFelxibowl; }
            set
            {
                if (_statusFelxibowl != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusFelxibowl = value;
                    OnPropertyChanged(nameof(StatusFlexibowl));
                }
            }
        }
        public bool StatusMqttClient
        {
            get { return _statusMqttClient; }
            set
            {
                if (_statusMqttClient != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusMqttClient = value;
                    OnPropertyChanged(nameof(StatusMqttClient));
                }
            }
        }
        public bool StatusScaraEmulation
        {
            get { return _statusScaraEmulation; }
            set
            {
                if (_statusScaraEmulation != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusScaraEmulation = value;
                    OnPropertyChanged(nameof(StatusScaraEmulation));
                }
            }
        }
        public string StreamingTopic
        {
            get { return _streamingTopic; }
            set
            {
                if (_streamingTopic != value) // setting value different from actual value -> store new value and trigger event
                {
                    _streamingTopic = value;
                    OnPropertyChanged(nameof(StreamingTopic));
                }
            }
        }
        public bool RequestScreenshots
        {
            get { return _requestScreenshots; }
            set
            {
                if (_requestScreenshots != value) // setting value different from actual value -> store new value and trigger event
                {
                    _requestScreenshots = value;
                    OnPropertyChanged(nameof(RequestScreenshots));
                }
            }
        }
        protected void OnPropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }


        // events
        public event Action<Control> NewUserControlRequested; // Event to notify subscribers
        public event Action TempFileDeleted;
        public event Action TempFolderWatcherResumed;
        public event Action<string> BitmapQueued;


        // MQTT
        public ConcurrentQueue<(Bitmap Frame, DateTime messageTimestamp, DateTime receptionTimestamp)> _idsQueue = new ConcurrentQueue<(Bitmap Frame, DateTime messageTimestamp, DateTime receptionTimestamp)>();
        public ConcurrentQueue<(Bitmap Frame, DateTime messageTimestamp, DateTime receptionTimestamp)> _baslerQueue = new ConcurrentQueue<(Bitmap Frame, DateTime messageTimestamp, DateTime receptionTimestamp)>();
        public ConcurrentQueue<(Bitmap Frame, DateTime messageTimestamp, DateTime receptionTimestamp)> _luxonisQueue = new ConcurrentQueue<(Bitmap Frame, DateTime messageTimestamp, DateTime receptionTimestamp)>();
        private DateTime lastNonStreamingUpdate = DateTime.MinValue;

        // file management
        public string TempFolder { get; private set; } = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\Temp"));
        public string ConfigFolder { get; private set; } = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\Config"));

        public FileSystemWatcher TempFolderWatcher;
        private bool _isFileWatcherPaused = false;


        // Logging
        public event Action<LogEntry> LogEvent; // Event to notify subscribers
        public class LogEntry
        {
            public string Message { get; set; }
            public bool IsError { get; set; }
            public bool IsSuccess { get; set; }
            public override string ToString() => Message; // Fallback for ListBox's default behavior
        }


        public optisort_mgr()
        {
            // Instance MQTT class
            string mqttBroker = Properties.Settings.Default.mqtt_broker;
            string mqttPort = Properties.Settings.Default.mqtt_port;
            MqttClient = new MQTT(mqttBroker, mqttPort);

            // Attach mqtt messages to handler
            MqttClient.MessageReceived += OnMessageReceived;

            // attach property changes to various useful methods
            PropertyChanged += OnPropertyUpdate;

            // Initialize the remoting subsystem
            RemotingUtil.InitializeRemotingSubsystem(true, 0);

            // Instance cobra class
            string remotingPort = Properties.Settings.Default.scara_port;
            Cobra600 = new Cobra600("ace", remotingPort);

            // Instance flexibowl class
            string flexibowlIP = Properties.Settings.Default.flexibowl_IP;
            Flexibowl = new Flexibowl(flexibowlIP);
        }


        private void OnPropertyUpdate(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(StreamingTopic))
                ResetQueues();
        }

        public void RequestNewUcLoading(Control control)
        {
            NewUserControlRequested?.Invoke(control);
        }

        // -----------------------------------------------------------------------------------
        // ---------------------------------- SUBSYSTEMS -------------------------------------
        // -----------------------------------------------------------------------------------

        #region Subsystems

        // TODO: is possible to use async?

        public bool ConnectScara()
        {
            // check if ACE is running
            Process[] ProcessList = Process.GetProcessesByName("Ace");
            if (ProcessList.Length != 1)
            {
                MessageBox.Show("ACE is not running: please open the robot server");
                return false;
            }
            Log("Trying to connect to robot", false, false);


            string controllerName = Properties.Settings.Default.scara_controllerName;
            string robotName = Properties.Settings.Default.scara_robotName;
            string endEffectorName = Properties.Settings.Default.scara_endEffectorName;


            if (!StatusScaraEmulation)
                Cobra600.RobotIP = Properties.Settings.Default.scara_controllerIP;

            var ex = Cobra600.Connect(StatusScaraEmulation, controllerName, robotName, endEffectorName);
            if (ex == null)
            {
                StatusScara = true;
                Log("Robot successfully connected", false, true);
                MessageBox.Show("Scara connected, enabling power. Please press the physical button on the front panel!");
                return true;
            }
            else
            {
                MessageBox.Show($"{ex}");
                Log("Failed to connect to robot", true, false);
                return false;
            }
        }

        public bool DisconnectScara()
        {
            Log("Trying to disconnect from robot", false, false);

            if (Cobra600.Disconnect())
            {
                StatusScara = false;
                Log("Robot succesfully disconnected", false, true);
                return true;
            }
            else
            {
                Log("Failed to disconnect from Cobra", true, false);
                return false;
            }
        }

        public void ToggleScaraEmulationMode()
        {
            if (StatusScaraEmulation)
            {
                StatusScaraEmulation = false;
                Log("Scara emulation mode disabled", false, false);
            }
            else
            {
                StatusScaraEmulation = true;
                Log("Scara emulation mode enabled", false, false);
            }
        }

        public bool ConnectFlexibowl()
        {
            Log("Trying to connect to flexibowl", false, false);

            Flexibowl.Connect();
            try
            {
                Flexibowl.Set.Servo(true);
                // TODO: should check for a connection by asking for a response
                StatusFlexibowl = true;
                Log("Flexibowl connected and servo ON", false, true);
                return true;
            }
            catch (Exception ex)
            {
                Log($"Failed to connect Flexibowl UDP client", true, false);
                MessageBox.Show($"{ex}");
                return false;
            }
        }

        public bool DisconnectFlexibowl()
        {
            Log("Trying to disconnect to flexibowl", false, false);

            if (Flexibowl.Disconnect())
            {
                // TODO: should check for a connection by asking for a response
                StatusFlexibowl = false;
                Log("Flexibowl UDP client disconnected", false, true);
                return true;
            }
            else
            {
                Log("Failed to disconnect from Flexibowl UDP Client", true, false);
                return false;
            }
        }

        public async Task<bool> ConnectMQTTClient()
        {
            if (StatusMqttClient)
            {
                Log("MQTT client already connected, cannot connect another", true, false);
                return false;
            }

            Log("Trying to create new MQTT client", false, false);

            string mqttClientName = Properties.Settings.Default.mqtt_client;
            Task<bool> createClient = MqttClient.CreateClient(mqttClientName);

            if (await createClient)
            {
                StatusMqttClient = true;

                Log($"MQTT client '{mqttClientName}' created", false, true);

                List<string> topics = new List<string>
                {
                    // Retreive topics from configuration file
                    Properties.Settings.Default.mqtt_topic_scaraTarget,
                    Properties.Settings.Default.mqtt_topic_idsStream,
                    Properties.Settings.Default.mqtt_topic_luxonisStream,
                    Properties.Settings.Default.mqtt_topic_baslerStream
                };

                foreach (string topic in topics)
                {
                    SubscribeMqttTopic(topic);
                }

                return true;
            }
            else
            {
                Log($"Failed to create '{mqttClientName}' MQTT client", true, false);
                return false;
            }
        }

        public async Task<bool> DisconnectMqttClient()
        {
            string mqttClientName = Properties.Settings.Default.mqtt_client;
            Log($"Trying to destroy MQTT client {mqttClientName}", false, false);

            Task<bool> destroyClient = MqttClient.DestroyClient(mqttClientName);
            if (await destroyClient)
            {
                StatusMqttClient = false;
                Log($"MQTT client '{mqttClientName}' destroyed", false, true);
                return true;
            }
            else
            {
                Log($"Error destroying '{mqttClientName}'", true, false);
                return false;
            }
        }

        public async void SubscribeMqttTopic(string topic)
        {
            string mqttClientName = Properties.Settings.Default.mqtt_client;
            bool subscribed = await MqttClient.SubscribeClientToTopic(mqttClientName, topic);
            if (subscribed)
                Log($"{mqttClientName} subscribed to {topic}", false, true);
            else
                Log($"Unable subscribing {mqttClientName} to {topic}", true, false);
        }

        public async void UnsubscribeMqttTopic(string topic)
        {
            string mqttClientName = Properties.Settings.Default.mqtt_client;
            bool unsubscribed = await MqttClient.UnsubscribeClientFromTopic(mqttClientName, topic);
            if (unsubscribed)
                Log($"{mqttClientName} unsubscribed to {topic}", false, true);
            else
                Log($"Unable unsubscribing {mqttClientName} to {topic}", true, false);
        }

        public void UpdateStreamingTopic(string newTopic)
        {
            if (StatusMqttClient)
            {
                UnsubscribeMqttTopic(StreamingTopic);
                StreamingTopic = newTopic;
                SubscribeMqttTopic(StreamingTopic);
            }
        }


        #endregion

        // -----------------------------------------------------------------------------------
        // ---------------------------------- MQTT MANAGEMENT --------------------------------
        // -----------------------------------------------------------------------------------

        /// <summary>
        /// Each time an MQTT arrives, a dedicated task is created to convert the incoming JSON to a BITMAP and queue it
        /// </summary>
        /// <param name="topic"></param>
        /// <param name="message"></param>
        private void OnMessageReceived(string topic, JsonElement message)
        {
            // TODO: activate this only in case of streaming topics
            Task.Run(() => AddBitmapToQueue(topic, message)); // Start a dedicated task for the method
        }


        /// <summary>
        /// Converts the incoming JSON to a BITMAP of any mqtt topic
        /// If the triggering topic is the streaming topic, the queue can reach a maximum of 5 items then is cleared to avoid latency
        /// If the triggering topic is a NON-streaming one, the respective queue is updated only each 1s and the queue can contain at maximum 1 item (saving resources)
        /// Queuing non-streaming bitmaps is useful to render them in case of screenshots (e.g. camera lens calibration)
        /// </summary>
        /// <param name="topic"></param>
        /// <param name="message"></param>
        private void AddBitmapToQueue(string topic, JsonElement message)
        {

            // coverting json into bitmap
            string base64Image = message.GetProperty("image").GetString();
            Bitmap image = JsonToBitmap(base64Image); // Decode the base64 image

            string timestampString = message.GetProperty("timestamp").GetString();
            DateTime messageTimestamp = DateTime.Parse(timestampString, null, System.Globalization.DateTimeStyles.RoundtripKind); // Decode timestamp


            const int MaxStreamingQueueSize = 5;            // Max size for StreamingTopic queue
            const int NonStreamingUpdateInterval = 1000;    // 1 second for updates to non-streaming queues


            if (topic == StreamingTopic)
            {
                // Determine the queue dynamically and add bitmap to queue (including json timestamp, enqueuing timestamp)
                ConcurrentQueue<(Bitmap, DateTime, DateTime)> streamingQueue = GetQueueForTopic(topic);
                streamingQueue.Enqueue((image, messageTimestamp, DateTime.Now));

                // If the queue exceeds the maximum size, remove previous frames to reduce latency
                while (streamingQueue.Count > MaxStreamingQueueSize)
                {
                    streamingQueue.TryDequeue(out _);
                }
            }
            else // Handle non-streaming topics
            {
                if (((DateTime.Now - lastNonStreamingUpdate).TotalMilliseconds >= NonStreamingUpdateInterval) || RequestScreenshots == true) // if screenshot mode is disabled, update queue with reduced frequency (discard some mqtt messages to avoid overload)
                {
                    ConcurrentQueue<(Bitmap, DateTime, DateTime)> nonStreamingQueue = GetQueueForTopic(topic);
                    
                    if(nonStreamingQueue.Count > 0 || RequestScreenshots == true)
                        nonStreamingQueue.TryDequeue(out _);

                    nonStreamingQueue.Enqueue((image, messageTimestamp, DateTime.Now)); 
                    lastNonStreamingUpdate = DateTime.Now; // update last queuing time for non-streaming topics
                }
            }

            // Uncomment below to debug queues
            // Console.WriteLine("Ids queue: " + _idsQueue.Count + "; Basler queue: " + _baslerQueue.Count + "; Luxonis queue: " + _luxonisQueue.Count);
            BitmapQueued?.Invoke(topic); // notify subscribers about queuing a bitmap
        }


        private ConcurrentQueue<(Bitmap, DateTime, DateTime)> GetQueueForTopic(string topic)
        {
            if (topic == Properties.Settings.Default.mqtt_topic_idsStream)
                return _idsQueue;

            if (topic == Properties.Settings.Default.mqtt_topic_baslerStream)
                return _baslerQueue;

            if (topic == Properties.Settings.Default.mqtt_topic_luxonisStream)
                return _luxonisQueue;

            throw new ArgumentException("Unknown topic: " + topic);
        }

        private void ResetQueues()
        {
            Console.Write("Resetting queues");
            while (_idsQueue.TryDequeue(out _)) { } // Clear the queue (remove all elements)
            while (_baslerQueue.TryDequeue(out _)) { } // Clear the queue (remove all elements)
            while (_luxonisQueue.TryDequeue(out _)) { } // Clear the queue (remove all elements)
        }


        // -----------------------------------------------------------------------------------
        // ---------------------------------- FILE MANAGEMENT --------------------------------
        // -----------------------------------------------------------------------------------

        #region file management

        /// <summary>
        /// Creates an agent that checks on updates within the TEMP folder to update the flow panels respectively
        /// </summary>
        public void SetupTempFolderWatcher()
        {

            if (!Directory.Exists(TempFolder))
            {
                Directory.CreateDirectory(TempFolder);
            }

            TempFolderWatcher = new FileSystemWatcher
            {
                Path = TempFolder, // Directory to watch
                Filter = "*.bmp", // Monitor only BMP files
                NotifyFilter = NotifyFilters.FileName | NotifyFilters.LastWrite | NotifyFilters.CreationTime
            };

            // Subscribe to events
            //_fileWatcher.Created += OnFileChanged; // too aggressive (three images cannot be added simultaneously, not even by pasting)
            TempFolderWatcher.Deleted += OnFileDeleted;
            //_fileWatcher.Changed += OnFileChanged; // too aggressive (three images cannot changed simultaneously)
            //_fileWatcher.Renamed += OnFileRenamed; // too aggressive (three images cannot renamed simultaneously)

            TempFolderWatcher.EnableRaisingEvents = true;
        }

        public void PauseFileWatcher()
        {
            _isFileWatcherPaused = true;
        }

        public void ResumeFileWatcher()
        {
            _isFileWatcherPaused = false;
            TempFolderWatcherResumed?.Invoke();
        }

        public void OnFileDeleted(object sender, FileSystemEventArgs e)
        {
            if (_isFileWatcherPaused)
                return;

            TempFileDeleted?.Invoke();
            Debug.WriteLine($"File {e.ChangeType}: {e.FullPath}");
        }

        public void DisposeTempFolderWatcher()
        {
            TempFolderWatcher?.Dispose();
        }


        /// <summary>
        /// Converts the image MQTT image (base64) to a bitmap
        /// </summary>
        /// <param name="base64Image"></param>
        /// <returns></returns>
        private Bitmap JsonToBitmap(string base64Image)
        {
            // Trim off any metadata if present
            if (base64Image.Contains(","))
            {
                base64Image = base64Image.Substring(base64Image.IndexOf(",") + 1);
            }

            // Convert from Base64 to Bitmap
            byte[] imageBytes = Convert.FromBase64String(base64Image);
            using (var ms = new MemoryStream(imageBytes))
            {
                return new Bitmap(ms);
            }
        }


        /// <summary>
        /// Export screenshots as bmp files to the TEMP folder
        /// </summary>
        /// <param name="bitmap"></param>
        /// <param name="fileName"></param>
        /// <exception cref="ArgumentNullException"></exception>
        /// <exception cref="ArgumentException"></exception>
        public void SaveBitmapAsFile(Bitmap bitmap, string fileName)
        {
            if (bitmap == null)
            {
                throw new ArgumentNullException(nameof(bitmap), "Bitmap cannot be null.");
            }

            if (string.IsNullOrWhiteSpace(fileName))
            {
                throw new ArgumentException("File name cannot be null or empty.", nameof(fileName));
            }


            if (!Directory.Exists(TempFolder))
            {
                Directory.CreateDirectory(TempFolder);
            }

            // Combine the output folder with the file name
            string tempFilePath = Path.Combine(TempFolder, fileName + ".bmp");

            // Save the bitmap as a BMP file
            bitmap.Save(tempFilePath, ImageFormat.Bmp);

            Console.WriteLine($"Bitmap saved as BMP at: {tempFilePath}");
        }


        /// <summary>
        /// Delete specific image from the TEMP folder, shift other indexes to occupy that groupId
        /// </summary>
        /// <param name="name"></param>
        public void DeleteFile(string name)
        {
            try
            {
                string filePath = Path.Combine(TempFolder, name);
                if (File.Exists(filePath))
                    File.Delete(filePath);
                else
                    Console.WriteLine($"Delete failed because file is not found: {filePath}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error deleting file: {ex.Message}");
            }
        }


        /// <summary>
        /// Delete all the contents of the TEMP folder (calibration screenshots of the cameras)
        /// </summary>
        /// <exception cref="ArgumentException"></exception>
        public void ClearTempDirectory()
        {
            if (string.IsNullOrWhiteSpace(TempFolder))
            {
                throw new ArgumentException("Temporary directory path cannot be null or empty.", nameof(TempFolder));
            }

            try
            {
                // Ensure the temporary directory exists
                if (Directory.Exists(TempFolder))
                {
                    // Get all files in the temporary directory
                    var files = Directory.GetFiles(TempFolder);

                    // Delete each file
                    foreach (var file in files)
                    {
                        try
                        {
                            File.Delete(file);
                            Console.WriteLine($"Deleted: {file}");
                        }
                        catch (Exception ex)
                        {
                            // Log the error (or handle it accordingly)
                            Console.WriteLine($"Failed to delete file: {file}. Error: {ex.Message}");
                        }
                    }

                    // Optionally, delete empty subdirectories
                    var directories = Directory.GetDirectories(TempFolder);
                    foreach (var directory in directories)
                    {
                        try
                        {
                            Directory.Delete(directory, true); // Recursive delete
                            Console.WriteLine($"Deleted directory: {directory}");
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine($"Failed to delete directory: {directory}. Error: {ex.Message}");
                        }
                    }
                }
                else
                {
                    // If the directory doesn't exist, create it
                    Directory.CreateDirectory(TempFolder);
                    Console.WriteLine($"Temporary directory created: {TempFolder}");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Failed to clean temporary directory. Error: {ex.Message}",
                                "Error",
                                MessageBoxButtons.OK,
                                MessageBoxIcon.Error);
            }
        }

        #endregion

        // -----------------------------------------------------------------------------------
        // ------------------------------------- GENERAL -------------------------------------
        // -----------------------------------------------------------------------------------

        /// <summary>
        /// Can be called by any user control to log things inside the listbox in the frmain, which is invoked by this
        /// </summary>
        /// <param name="message"></param>
        /// <param name="isError"></param>
        /// <param name="isSuccess"></param>
        public void Log(string message, bool isError = false, bool isSuccess = false)
        {
            // Prepend timestamp to the message
            var logEntry = new LogEntry
            {
                Message = $"{DateTime.Now}: {message}",
                IsError = isError,
                IsSuccess = isSuccess
            };

            // Raise the log event
            LogEvent?.Invoke(logEntry);
        }

        /// <summary>
        /// Run a python script in background and returns its output as a string
        /// </summary>
        /// <param name="scriptPath"></param>
        public string RunPythonScript(string scriptPath)
        {
            string pythonExe = LocatePythonInterpreter();
            if (string.IsNullOrEmpty(pythonExe))
            {
                throw new Exception("Python interpreter not found. Ensure Python is installed and added to the system PATH.");
            }

            if (!File.Exists(scriptPath))
            {
                throw new Exception("Python script not found.");
            }

            var processStartInfo = new ProcessStartInfo
            {
                FileName = pythonExe,
                Arguments = scriptPath, // Path to the script
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true
            };

            var process = new Process { StartInfo = processStartInfo };

            // Use StringBuilder to collect output data
            var outputBuilder = new StringBuilder();
            var errorBuilder = new StringBuilder();

            process.OutputDataReceived += (sender, e) =>
            {
                if (!string.IsNullOrEmpty(e.Data))
                {
                    outputBuilder.AppendLine(e.Data);
                }
            };
            process.ErrorDataReceived += (sender, e) =>
            {
                if (!string.IsNullOrEmpty(e.Data))
                {
                    errorBuilder.AppendLine(e.Data);
                }
            };

            // Start the process
            process.Start();
            process.BeginOutputReadLine();
            process.BeginErrorReadLine();

            // Wait for the process to complete (or temporary imaged used will overlap)
            process.WaitForExit();

            // Handle errors if any
            if (errorBuilder.Length > 0)
            {
                Console.WriteLine("ERROR: " + errorBuilder.ToString());
                // Optionally, handle errors here (e.g., log them or throw an exception)
            }

            // Trigger the CalibrationResult method with the collected output
            if (outputBuilder.Length > 0)
            {
                return outputBuilder.ToString(); // call explicitly the calibration result method
            }

            return null;
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

            return null; // Python not found
        }

    }
}
