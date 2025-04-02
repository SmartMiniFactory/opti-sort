using Ace.Core.Util;
using Ace.UIBuilder.Client.Controls.Tools.WindowsForms;
using FlexibowlLibrary;
using OptiSort.Classes;
using OptiSort.systems;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Security;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace OptiSort
{

    internal class optisort_mgr : INotifyPropertyChanged
    {

        private frmMain _frmMain;

        // Services
        public MQTT MqttClient { get; set; }
        public Cobra600 Cobra600 { get; set; }
        public Flexibowl Flexibowl { get; set; }
        public CameraManager Cameramanager { get; set; }



        // Python runner
        private Dictionary<int, PythonProcessRunner> _runners;
        private Dictionary<int, string> _activeProcesses;


        // MQTT-specific
        public ConcurrentQueue<(Bitmap Frame, DateTime messageTimestamp, DateTime receptionTimestamp)> _idsQueue = new ConcurrentQueue<(Bitmap Frame, DateTime messageTimestamp, DateTime receptionTimestamp)>();
        public ConcurrentQueue<(Bitmap Frame, DateTime messageTimestamp, DateTime receptionTimestamp)> _baslerQueue = new ConcurrentQueue<(Bitmap Frame, DateTime messageTimestamp, DateTime receptionTimestamp)>();
        public ConcurrentQueue<(Bitmap Frame, DateTime messageTimestamp, DateTime receptionTimestamp)> _luxonisQueue = new ConcurrentQueue<(Bitmap Frame, DateTime messageTimestamp, DateTime receptionTimestamp)>();
        private DateTime lastNonStreamingUpdate = DateTime.MinValue;


        // Status
        public event PropertyChangedEventHandler PropertyChanged; // declare propertyChanged event (required by the associated interface)
        private readonly SynchronizationContext _syncContext = SynchronizationContext.Current; // used to update properties from other threads (e.g. CameraManager class)
        private bool _statusScara = false;
        private bool _statusScaraEmulation = true;
        private bool _statusFelxibowl = false;
        private bool _statusMqttClient = false;
        private bool _statusCameraManager = false;
        private bool _statusCameraTesting = false;
        private string _streamingTopic = null;
        private bool _requestScreenshots = false;

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
        public bool StatusCameraManager
        {
            get { return _statusCameraManager; }
            set
            {
                if (_statusCameraManager != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusCameraManager = value;
                    OnPropertyChanged(nameof(StatusCameraManager));
                }
            }
        }
        public bool StatusCameraTesting
        {
            get { return _statusCameraTesting; }
            set
            {
                if (_statusCameraTesting != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusCameraTesting = value;
                    OnPropertyChanged(nameof(StatusCameraTesting));
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
            // invoke correct thread to trigger PropertyChanged event (since properties may be changed from other threads)
            if (_syncContext != null && SynchronizationContext.Current != _syncContext)
            {
                _syncContext.Post(_ => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName)), null);
            }
            else
            {
                PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
            }
        }


        // Events
        public event Action<string, JsonElement, int> MqttMessageReceived; // expose mqtt messaging events for user controls' subscriptions
        public event Action<Control> NewUserControlRequested; // Event to notify subscribers
        public event Action TempFileDeleted;
        public event Action TempFolderWatcherResumed;
        public event Action<string> BitmapQueued;
        public event Action<int, string> OnOutputReceived; // python script generates an output 
        public event Action<int, string> OnErrorReceived; // python script generates an error
        public event Action<int, bool> OnExecutionTerminated; // python script ended


        // File management
        public FileSystemWatcher TempFolderWatcher;
        private bool _isFileWatcherPaused = false;

        public string TempFolder { get; private set; } = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\Temp"));
        public string ConfigFolder { get; private set; } = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\Config"));


        // Logging
        public event Action<LogEntry> LogEvent; // Event to notify subscribers
        public class LogEntry
        {
            public string Message { get; set; }
            public bool IsError { get; set; }
            public bool IsSuccess { get; set; }
            public override string ToString() => Message; // Fallback for ListBox's default behavior
        }

        public event Action<string, string, MessageBoxIcon> MessageBoxEvent; // Event to notify subscribers


        // -----------------------------------------------------------------------------------
        // ---------------------------------- CONSTRUCTOR -------------------------------------
        // -----------------------------------------------------------------------------------

        public optisort_mgr(frmMain frmMain)
        {
            _frmMain = frmMain;

            // Instance MQTT class
            MqttClient = new MQTT();

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

            // Instance class dedicated to managing cameras
            Cameramanager = new CameraManager(this, frmMain);

            // Instance class dedicated to running python files
            _activeProcesses = new Dictionary<int, string>();
            _runners = new Dictionary<int, PythonProcessRunner>();

        }



        private void OnPropertyUpdate(object sender, PropertyChangedEventArgs e)
        {
            // This method is triggered anytime one of the properties defined under "status" gets updated
            if (e.PropertyName == nameof(StreamingTopic))
                ResetQueues();

            // add stuff here...
        }

        /// <summary>
        /// User controls can request loading another user control by calling this method
        /// </summary>
        /// <param name="control"></param>
        public void RequestNewUcLoading(Control uc)
        {
            NewUserControlRequested?.Invoke(uc);
        }

        // -----------------------------------------------------------------------------------
        // ---------------------------------- SUBSYSTEMS -------------------------------------
        // -----------------------------------------------------------------------------------

        #region Subsystems

        // TODO: is possible to use async?

        /// <summary>
        /// Connects to the SCARA robot (ACE software should be running)
        /// </summary>
        /// <returns></returns>
        public bool ConnectScara()
        {
            // check if ACE is running
            Process[] ProcessList = Process.GetProcessesByName("Ace");
            if (ProcessList.Length != 1)
            {
                NonBlockingMessageBox("ACE is not running: please open the robot server", "Interlock!", MessageBoxIcon.Hand);
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
                NonBlockingMessageBox("Scara connected, enabling power. Please press the physical button on the front panel!", "Success!", MessageBoxIcon.Information);
                return true;
            }
            else
            {
                NonBlockingMessageBox($"{ex}", "Error!", MessageBoxIcon.Error);
                Log("Failed to connect to robot", true, false);
                return false;
            }
        }

        /// <summary>
        /// Disconnects the connected SCARA robot
        /// </summary>
        /// <returns></returns>
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

        /// <summary>
        /// Switch connection between a connection to a real robot or an emulated one
        /// </summary>
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

        /// <summary>
        /// Connect to the FLEXIBOWL robot
        /// </summary>
        /// <returns></returns>
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
                NonBlockingMessageBox($"{ex}", "Error!", MessageBoxIcon.Error);
                return false;
            }
        }

        /// <summary>
        /// Disconnect from a connected FLEXIBOWL robot
        /// </summary>
        /// <returns></returns>
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

        /// <summary>
        /// Create MQTT client and connect to a broker
        /// </summary>
        /// <returns></returns>
        public async Task<bool> ConnectMQTTClient(string client, string broker)
        {
            if (StatusMqttClient)
            {
                Log("MQTT client already connected, cannot connect another", true, false);
                return false;
            }

            Log("Trying to create new MQTT client", false, false);

            string mqttPort = Properties.Settings.Default.mqtt_port;
            Task<bool> createClient = MqttClient.CreateClient(client, broker, mqttPort);

            if (await createClient)
            {
                StatusMqttClient = true;

                Log($"MQTT client '{client}' created", false, true);

                List<string> topics = new List<string>
                {
                    // Retreive topics from configuration file
                    Properties.Settings.Default.mqtt_topic_scaraTarget,
                    Properties.Settings.Default.mqtt_topic_idsStream,
                    Properties.Settings.Default.mqtt_topic_luxonisStream,
                    Properties.Settings.Default.mqtt_topic_baslerStream
                };

                if (StatusCameraManager)
                    topics.Add("optisort/camera_manager/output");

                foreach (string topic in topics)
                {
                    SubscribeMqttTopic(client, topic);
                }

                return true;
            }
            else
            {
                Log($"Failed to create '{client}' MQTT client", true, false);
                return false;
            }
        }

        /// <summary>
        /// Disconnect MQTT client from a broker, then destroy client
        /// </summary>
        /// <returns></returns>
        public async Task<bool> DisconnectMqttClient(string client)
        {
            Log($"Trying to destroy MQTT client {client}", false, false);

            Task<bool> destroyClient = MqttClient.DestroyClient(client);
            if (await destroyClient)
            {
                StatusMqttClient = false;
                Log($"MQTT client '{client}' destroyed", false, true);
                return true;
            }
            else
            {
                Log($"Error destroying '{client}'", true, false);
                return false;
            }
        }

        /// <summary>
        /// Subscribe the connected MQTT client to a specific topic of the broker
        /// </summary>
        /// <param name="topic"></param>
        public async void SubscribeMqttTopic(string client, string topic)
        {
            bool subscribed = await MqttClient.SubscribeClientToTopic(client, topic);
            if (subscribed)
                Log($"{client} subscribed to {topic}", false, true);
            else
                Log($"Unable subscribing {client} to {topic}", true, false);
        }

        /// <summary>
        /// Unsubscribe MQTT client from topic
        /// </summary>
        /// <param name="topic"></param>
        public async void UnsubscribeMqttTopic(string client, string topic)
        {
            bool unsubscribed = await MqttClient.UnsubscribeClientFromTopic(client, topic);
            if (unsubscribed)
                Log($"{client} unsubscribed to {topic}", false, true);
            else
                Log($"Unable unsubscribing {client} to {topic}", true, false);
        }


        public void UpdateStreamingTopic(string newTopic)
        {
            if (StatusMqttClient)
            {
                StreamingTopic = newTopic;
            }
        }


        public async void PublishMqttMessage(string client, string topic, object message)
        {
            _ = await MqttClient.PublishMessage(client, topic, message);
        }


        public void ToggleCameraTestingMode()
        {
            if (StatusCameraTesting)
            {
                StatusCameraTesting = false;
                Log("Camera manager will now connect to optisort's cameras", false, false);
            }
            else
            {
                StatusCameraTesting = true;
                Log("Camera manager will now use host's webcam (if there is any)", false, false);
            }
        }

        public void ConnectCameras()
        {
            if (!StatusMqttClient)
            {
                NonBlockingMessageBox("Cannot connect to camera manager: MQTT client is not connected", "Interlock!", MessageBoxIcon.Hand);
                return;
            }

            Cameramanager.ConnectCameraManager();
            StatusCameraManager = true;
        }

        public void DisconnectCameras()
        {
            if (!StatusMqttClient)
            {
                NonBlockingMessageBox("Cannot connect to camera manager: MQTT client is not connected", "Interlock!", MessageBoxIcon.Hand);
                return;
            }

            Cameramanager.DisconnectCameraManager();
            StatusCameraManager = false;
        }


        #endregion

        // -----------------------------------------------------------------------------------
        // ---------------------------------- MQTT MANAGEMENT --------------------------------
        // -----------------------------------------------------------------------------------

        #region MQTT

        /// <summary>
        /// Triggered each time an MQTT arrives
        /// </summary>
        /// <param name="topic"></param>
        /// <param name="message"></param>
        private void OnMessageReceived(string topic, JsonElement message)
        {
            // launch bitmap conversion only if message comes from streaming topics
            if (Regex.IsMatch(topic, @".*/stream$"))
                Task.Run(() => AddBitmapToQueue(topic, message)); // Start a dedicated task for the method

            // if the message is from a script, it is probably awaited from an user control, thus message received event is exposed with script processID
            if (message.TryGetProperty("script", out JsonElement scriptElement))
            {
                if (scriptElement.TryGetProperty("PID", out JsonElement pidElement))
                {
                    int PID = int.Parse(pidElement.GetString());
                    Console.WriteLine($"Mqtt PID: {PID}");
                    foreach (var activeProcessID in _activeProcesses)
                    {
                        if (activeProcessID.Key == PID)
                        {
                            MqttMessageReceived?.Invoke(topic, message, PID); // expose mqtt message with process value (user forms need the process value to trigger event handlers)
                            break;
                        }
                    }
                }
            }

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

                    if (nonStreamingQueue.Count > 0 || RequestScreenshots == true)
                        nonStreamingQueue.TryDequeue(out _);

                    nonStreamingQueue.Enqueue((image, messageTimestamp, DateTime.Now));
                    lastNonStreamingUpdate = DateTime.Now; // update last queuing time for non-streaming topics
                }
            }

            // Uncomment below to debug queues
            // Console.WriteLine("Ids queue: " + _idsQueue.Count + "; Basler queue: " + _baslerQueue.Count + "; Luxonis queue: " + _luxonisQueue.Count);
            BitmapQueued?.Invoke(topic); // notify subscribers about queuing a bitmap
        }

        /// <summary>
        /// Returns streaming queue based on requested topic
        /// </summary>
        /// <param name="topic"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException"></exception>
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

        /// <summary>
        /// Resets contents of all the streaming queues
        /// </summary>
        private void ResetQueues()
        {
            Console.Write("Resetting queues");
            while (_idsQueue.TryDequeue(out _)) { } // Clear the queue (remove all elements)
            while (_baslerQueue.TryDequeue(out _)) { } // Clear the queue (remove all elements)
            while (_luxonisQueue.TryDequeue(out _)) { } // Clear the queue (remove all elements)
        }


        public async void ForecastDigitalTwinInfo()
        {
            string clientNameDT = "OptisortDT";
            string mqttPort = Properties.Settings.Default.mqtt_port;

            // TODO: change with regular connnectClient method
            Task<bool> createClient = MqttClient.CreateClient(clientNameDT, "10.12.238.20", mqttPort);

            if (await createClient)
            {
                Log("Digital Twin client created", false, true);
                SubscribeMqttTopic(clientNameDT, "DT_BROADCAST");

                Thread.Sleep(2000);

                double[] values = { 1.23, 4.56, 7.89, 0.12 };
                var message = new
                {
                    sender = "SCARA",
                    receiver = "IPHYSICS",
                    command = 10,
                    payload = new
                    {
                        q1 = values[0],
                        q2 = values[1],
                        q3 = values[2],
                        q4 = values[3]
                    }
                };
                string jsonMessage = JsonSerializer.Serialize(message, new JsonSerializerOptions { WriteIndented = true });

                PublishMqttMessage(clientNameDT, "DT_BROADCAST", jsonMessage);

                Thread.Sleep(2000);

                UnsubscribeMqttTopic(clientNameDT, "DT_BROADCAST");

                Thread.Sleep(2000);

                _ = DisconnectMqttClient(clientNameDT);

            }



        }

        #endregion

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

        /// <summary>
        /// Pauses checking contents of the target folder
        /// </summary>
        public void PauseFileWatcher()
        {
            _isFileWatcherPaused = true;
        }

        /// <summary>
        /// Resumes checking contents of the target folder
        /// </summary>
        public void ResumeFileWatcher()
        {
            _isFileWatcherPaused = false;
            TempFolderWatcherResumed?.Invoke();
        }

        /// <summary>
        /// Generates deletion event when a file gets deleted from target folder
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void OnFileDeleted(object sender, FileSystemEventArgs e)
        {
            if (_isFileWatcherPaused)
                return;

            TempFileDeleted?.Invoke();
            Debug.WriteLine($"File {e.ChangeType}: {e.FullPath}");
        }

        /// <summary>
        /// Elimante agent checking the target folder
        /// </summary>
        public void DisposeTempFolderWatcher()
        {
            TempFolderWatcher?.Dispose();
        }

        /// <summary>
        /// Converts a JSON-encoded image (base64) to a bitmap
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

            try
            {
                // Convert from Base64 to Bitmap
                byte[] imageBytes = Convert.FromBase64String(base64Image);
                using (var ms = new MemoryStream(imageBytes))
                {
                    return new Bitmap(ms);
                }
            }
            catch (FormatException ex)
            {
                // Handle invalid Base64 format
                throw new ArgumentException("The provided Base64 string is not valid.", ex);
            }
            catch (Exception ex)
            {
                // Handle other exceptions (e.g., memory issues, invalid image data)
                throw new InvalidOperationException("An error occurred while converting the Base64 string to a Bitmap.", ex);
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
            string tempFilePath = Path.Combine(TempFolder, fileName + ".jpg");

            // Save the bitmap as a BMP file
            using (Image img = new Bitmap(bitmap))
            {
                img.Save(tempFilePath, ImageFormat.Jpeg);
            }

            Console.WriteLine($"Bitmap saved as JPG at: {tempFilePath}");
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
        // ------------------------------------- LOG -------------------------------------
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

        public void NonBlockingMessageBox(string text, string title = "Message", MessageBoxIcon icon = MessageBoxIcon.None)
        {
            MessageBoxEvent?.Invoke(text, title, icon);
        }


        // -----------------------------------------------------------------------------------
        // ------------------------------------- PYTHON -------------------------------------
        // -----------------------------------------------------------------------------------

        #region python

        /// <summary>
        /// User controls can call this method to run specific python scripts, which will be managed by dedicated threads
        /// </summary>
        /// <param name="scriptPath"></param>
        /// <returns></returns>
        public int ExecuteScript(string scriptPath)
        {

            foreach (var process in _activeProcesses)
            {
                if (process.Value == scriptPath) // script already running
                    return 0;
            }

            var runner = new PythonProcessRunner();
            int pid = runner.RunPythonScript(scriptPath);

            // Subscribe to runner events and associate with process ID
            runner.OnOutputReceived += (output) => OnOutputReceived?.Invoke(pid, output);
            runner.OnErrorReceived += (error) => OnErrorReceived?.Invoke(pid, error);
            runner.OnExecutionTerminated += (status) => OnExecutionTerminated?.Invoke(pid, status);

            _activeProcesses[pid] = scriptPath;
            _runners[pid] = runner;

            return pid;
        }

        /// <summary>
        /// Stop a specific python execution
        /// </summary>
        /// <param name="processId"></param>
        public void StopExecution(int processId)
        {

            if (_activeProcesses.ContainsKey(processId))
            {
                _activeProcesses.Remove(processId);
            }

            if (_runners.ContainsKey(processId))
            {
                _runners[processId].Stop(); // killing the background python process
                _runners.Remove(processId);
            }
        }

        public void KillAllProcesses()
        {
            var runnersCopy = _runners.Keys.ToList(); // copying list to avoid iteration exeption triggering

            foreach (var key in runnersCopy)
            {
                _runners[key].Stop();
            }
        }
        #endregion
    }
}
