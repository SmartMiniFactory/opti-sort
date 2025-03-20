using Ace.UIBuilder.Client.Controls.Tools.WindowsForms;
using ActiproSoftware.SyntaxEditor;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Security.Policy;
using System.Text;
using System.Text.Json;
using System.Windows.Forms;

namespace OptiSort.userControls
{
    public partial class ucCameraLensCalibration : UserControl
    {
        private optisort_mgr _manager;
        private int _shots;
        private bool _idsShot = false;
        private bool _luxonisShot = false;
        private bool _baslerShot = false;
        private DateTime _elapsedTime;
        private int _pythonProcessId;

        public int Shots
        {
            get => _shots;
            set
            {
                if (_shots != value)
                {
                    _shots = value;
                    RefreshBottomControls(); // Trigger re-evaluation when value changes
                }
            }
        }


        internal ucCameraLensCalibration(optisort_mgr manager)
        {
            InitializeComponent();
            _manager = manager;

            Shots = 0;

            this.Load += ucLensDistortionCalibration_Load; // Attach the Load event to call the method after the control is fully initialized
        }

        // Perform a series of actions on form loading
        private void ucLensDistortionCalibration_Load(object sender, EventArgs e)
        {
            RefreshCalibrationTimestamp();
            LoadExistingThumbnails();

            _manager.SetupTempFolderWatcher(); // create file watcher

            _manager.TempFileDeleted += OnTempFileDeleted; // Subscribe to manager's file deletion event from TEMP folder
            _manager.TempFolderWatcherResumed += OnWatcherResumed; // Subscribe to manager's file watcher resume event
            _manager.PropertyChanged += OnPropertyChange; // Subscribe to manager's propery changed event

            RefreshBottomControls(); 

        }

        /// <summary>
       /// Refresh pushbuttons based on manager's properties status
       /// </summary>
       /// <param name="sender"></param>
       /// <param name="e"></param>
        private void OnPropertyChange(object sender, PropertyChangedEventArgs e)
        {
            if(e.PropertyName == nameof(_manager.StatusMqttClient) || e.PropertyName == nameof(_manager.RequestScreenshots))
            {
                RefreshBottomControls();
            }
        }

        /// <summary>
        /// If an image gets deleted from the folder, the other two are also deleted and the flow panels are updated
        /// </summary>
        private void OnTempFileDeleted()
        {
            DeleteIncompleteTriplets();
            RefreshFlowPanels();
        }

        /// <summary>
        /// Reload flow panels on file watcher resume to catch up with current folder condition
        /// </summary>
        private void OnWatcherResumed()
        {
            LoadExistingThumbnails();
        }

        /// <summary>
        /// Launches calibration output receival
        /// </summary>
        /// <param name="processID"></param>
        /// <param name="output"></param>
        private void MqttMessageReceived(string topic, JsonElement message, int processID)
        {
            // check if mqtt message is sent by expected script
            if (processID == _pythonProcessId)
            {
                // check if message contains results
                if (message.TryGetProperty("result", out JsonElement resultElement))
                {
                    this.Invoke((MethodInvoker)(() =>
                    {
                        CalibrationResult(resultElement);
                    }));
                }

                // in any case: log "message" string coming from the python file
                if (message.TryGetProperty("message", out JsonElement messageElement))
                {
                    string msg = messageElement.GetString();

                    this.Invoke((MethodInvoker)(() =>
                    {
                        _manager.Log(msg, false, false); 
                    }));
                }
            } 
        }

        /// <summary>
        /// Logs python error and shows msgbox
        /// </summary>
        /// <param name="processID"></param>
        /// <param name="output"></param>
        private void PythonErrorHandler(int processID, string output)
        {
            if (processID == _pythonProcessId)
            {
                this.Invoke((MethodInvoker)(() =>
                {
                    _manager.Log("Python camera calibration file threw an error!", true, false);
                    MessageBox.Show("Python camera calibration file threw an error", "Python error!");
                }));
            }
        }

        private void PythonTerminationHandler(int processID, bool executionTerminated)
        {
            if (processID == _pythonProcessId)
            {
                this.Invoke((MethodInvoker)(() =>
                {                
                    _manager.Log("Python camera calibration file has closed!", false, false);
                    _manager.OnErrorReceived -= PythonErrorHandler;
                    _manager.OnExecutionTerminated -= PythonTerminationHandler;
                    _manager.UnsubscribeMqttTopic("optisort/camera_calibration");
                }));
            }
        }






        // ----------------------------------------------------------------------------------------
        // ------------------------------------ PUSHBUTTONS ---------------------------------------
        // ----------------------------------------------------------------------------------------

        private void btn_acquire_Click(object sender, EventArgs e)
        {
            if (!_manager.StatusMqttClient)
            {
                MessageBox.Show("Enable camera streaming first (MQTT service)");
                return;
            }

            Cursor = Cursors.WaitCursor;
            _manager.RequestScreenshots = true;
            _manager.BitmapQueued += SaveShots;
            _elapsedTime = DateTime.Now;
        }


        private void btn_calibrate_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;

            // subscribe to mqtt topic to receive updates from python file
            _manager.SubscribeMqttTopic("optisort/camera_calibration");
            _manager.MqttMessageReceived += MqttMessageReceived;

            // subscribe termination events to handler
            _manager.OnErrorReceived += PythonErrorHandler;
            _manager.OnExecutionTerminated += PythonTerminationHandler;

            // launch python file and memorize processId
            string scriptPath = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\python\other_scripts\cameras_calibration.py"));
            _pythonProcessId = _manager.ExecuteScript(scriptPath);
            _manager.Log("Camera calibration file launched", false, false);
        }

        private void btn_clear_Click(object sender, EventArgs e)
        {
            _manager.ClearTempDirectory();
        }


        private void btn_home_Click(object sender, EventArgs e)
        {
            _manager.DisposeTempFolderWatcher();
            ucManualControl ucManualControl = new ucManualControl(_manager);
            _manager.RequestNewUcLoading(ucManualControl);
        }


        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------



        private void SaveShots(string topic)
        {
            if (_manager.RequestScreenshots) // this check is needed because the event is not unsubscribed immediately and late coming triggers may generate additional screenshots
            {

                Invoke(new Action(() =>
                {
                    int imageIndex = GetEarliestAvailableIndex();

                    string prefix = string.Empty;
                    FlowLayoutPanel panel = null;
                    Bitmap image = null;

                    if (topic == Properties.Settings.Default.mqtt_topic_idsStream && _idsShot == false)
                    {
                        if (_manager._idsQueue.TryDequeue(out var item))
                        {
                            image = item.Frame;
                            _idsShot = true;
                        }
                        else return; // in case queue is empty, saving cannot continue because bitmaps cannot be null to be saved

                        prefix = "ids";
                        panel = flp_ids;
                    }
                    else if (topic == Properties.Settings.Default.mqtt_topic_baslerStream && _baslerShot == false)
                    {
                        if (_manager._baslerQueue.TryDequeue(out var item))
                        {
                            image = item.Frame;
                            _baslerShot = true;
                        }
                        else return;

                        prefix = "basler";
                        panel = flp_basler;
                    }
                    else if (topic == Properties.Settings.Default.mqtt_topic_luxonisStream && _luxonisShot == false)
                    {
                        if (_manager._luxonisQueue.TryDequeue(out var item))
                        {
                            image = item.Frame;
                            _luxonisShot = true;
                        }
                        else return;

                        prefix = "luxonis";
                        panel = flp_luxonis;
                    }

                    if (!string.IsNullOrEmpty(prefix) && panel != null)
                    {
                        string filename = $"{prefix}_CalibrationImage_{imageIndex.ToString("D2")}";
                        _manager.SaveBitmapAsFile(image, filename);
                        AddThumbnailToColumn(panel, image, filename);
                    }

                    if (_idsShot && _baslerShot && _luxonisShot)
                    {
                        _manager.BitmapQueued -= SaveShots;
                        _manager.RequestScreenshots = false;
                        _idsShot = false;
                        _luxonisShot = false;
                        _baslerShot = false;
                        Cursor = Cursors.Default;
                        Shots++;
                    }else if ((DateTime.Now - _elapsedTime).TotalSeconds > 5)
                    {
                        _manager.BitmapQueued -= SaveShots;
                        _manager.RequestScreenshots = false;
                        _idsShot = false;
                        _luxonisShot = false;
                        _baslerShot = false;
                        Cursor = Cursors.Default;
                        Shots++;
                        MessageBox.Show("Timeout reached. MQTT streamings might have a problem: check all the topics or inform the developer.");
                    }
                }));
            }

        }


        /// <summary>
        /// Identifies the earliest available index in the TEMP folder by analyzing existing filenames.
        /// </summary>
        /// <returns>The earliest available numeric index.</returns>
        private int GetEarliestAvailableIndex()
        {
            if (!Directory.Exists(_manager.TempFolder))
                return 1; // Start from 1 if the directory does not exist

            var existingFiles = Directory.GetFiles(_manager.TempFolder, "*_CalibrationImage_*.bmp");

            // Extract indices from filenames
            var indexCounts = existingFiles
                .Select(file =>
                {
                    string fileName = Path.GetFileNameWithoutExtension(file);
                    string[] parts = fileName.Split('_');
                    if (parts.Length == 3 && int.TryParse(parts[2], out int index))
                    {
                        return index;
                    }
                    return -1; // Invalid index
                })
                .Where(index => index > 0) // Filter out invalid indices
                .GroupBy(index => index) // Count occurrences
                .ToDictionary(g => g.Key, g => g.Count());

            // Start checking from index 1
            int i = 1;
            while (true)
            {
                if (!indexCounts.ContainsKey(i) || indexCounts[i] < 3)
                    return i; // Return the first index that has fewer than 3 occurrences

                i++; // Move to the next index
            }
        }



        /// <summary>
        /// Parse py's output and interpret result (calibration failed or succeded)
        /// </summary>
        /// <param name="pythonOutput"></param>
        private void CalibrationResult(JsonElement pythonOutput)
        {

            _manager.PauseFileWatcher();

            try
            {

                // Extracting "bad_image" indexes
                int[] GetIntArray(JsonElement parent, string key) => parent
                    .TryGetProperty(key, out JsonElement element) && element.ValueKind == JsonValueKind.Array ? element
                    .EnumerateArray().Select(e => e.GetInt32()).ToArray() : Array.Empty<int>();

                JsonElement badImage = pythonOutput.GetProperty("bad_image");

                int[] badImage_ids = GetIntArray(badImage, "ids");
                int[] badImage_basler = GetIntArray(badImage, "basler");
                int[] badImage_luxonis = GetIntArray(badImage, "luxonis");


                // extracting cameras who failed calibration
                bool GetBoolValue(JsonElement parent, string key) => parent
                    .TryGetProperty(key, out JsonElement element) && element.ValueKind == JsonValueKind.True?true:false;

                JsonElement calibrationFail = pythonOutput.GetProperty("calibration_fail");

                bool calibrationFail_ids = GetBoolValue(calibrationFail, "ids");
                bool calibrationFail_basler = GetBoolValue(calibrationFail, "basler");
                bool calibrationFail_luxonis = GetBoolValue(calibrationFail, "luxonis");

                if (calibrationFail_ids || calibrationFail_luxonis || calibrationFail_basler)
                {
                    MessageBox.Show($"OPENCV CALIBRATION FAILED!\nIDS: {calibrationFail_ids}\nBASLER: {calibrationFail_basler}\nLUXONIS: {calibrationFail_luxonis}");
                }

                if (badImage_ids.Length == 0 && badImage_basler.Length == 0 && badImage_luxonis.Length == 0)
                {
                    Shots = 0;
                    _manager.ClearTempDirectory();
                    RefreshCalibrationTimestamp();
                    MessageBox.Show($"CALIBRATION SUCCEDED!");
                    _manager.Log("Lens calibration successfull", false, true);
                }
                else
                {
                    // cumulate all bad images IDs
                    List<int> badImages_all = new List<int>();

                    badImages_all.AddRange(badImage_ids);
                    badImages_all.AddRange(badImage_basler.Where(id => !badImages_all.Contains(id)));
                    badImages_all.AddRange(badImage_luxonis.Where(id => !badImages_all.Contains(id)));


                    // deleting bad images IDs for all the cameras
                    foreach (int id in badImages_all)
                    {
                        _manager.DeleteFile("ids_CalibrationImage_" + id.ToString("D2") + ".bmp");
                        _manager.DeleteFile("basler_CalibrationImage_" + id.ToString("D2") + ".bmp");
                        _manager.DeleteFile("luxonis_CalibrationImage_" + id.ToString("D2") + ".bmp");
                    }

                    Shots -= badImages_all.Count;
                    MessageBox.Show($"BAD IMAGES DETECTED!\nCalibration failed because the chess board was not found in {badImages_all.Count} triplets. These were deleted. Please proceed to retaking the images again.");
                    _manager.Log("Bad images detected! Please retake some screenshots", false, false);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error parsing JSON: {ex.Message}");
                _manager.Log("Error parsing calibration result", true, false);
            }

            _manager.ResumeFileWatcher();
            Cursor = Cursors.Default;
            _manager.Log("Calibration process ended", false, false);
        }



        /// <summary>
        /// Deletes incomplete triplets in the TEMP folder.
        /// </summary>
        private void DeleteIncompleteTriplets()
        {
            if (!Directory.Exists(_manager.TempFolder))
                return;

            var imageFiles = Directory.GetFiles(_manager.TempFolder, "*.bmp");

            // Group files by their numeric identifier
            var groupedFiles = imageFiles
                .Select(file =>
                {
                    string fileName = Path.GetFileNameWithoutExtension(file);
                    string[] parts = fileName.Split('_');

                    if (parts.Length == 3 && parts[1] == "CalibrationImage")
                    {
                        return new
                        {
                            FullPath = file,
                            Prefix = parts[0], // "ids", "basler", "luxonis"
                            Id = parts[2]      // The numeric part after "CalibrationImage"
                        };
                    }
                    else
                    {
                        Debug.WriteLine($"Invalid file name format: {fileName}");
                        return null;
                    }
                })
                .Where(x => x != null)
                .GroupBy(file => file.Id)
                .OrderBy(group => group.Key)
                .ToList();

            foreach (var group in groupedFiles)
            {
                var triplet = group.ToList();

                // Check if all three images exist for the same ID
                var idsImage = triplet.FirstOrDefault(f => f.Prefix == "ids");
                var baslerImage = triplet.FirstOrDefault(f => f.Prefix == "basler");
                var luxonisImage = triplet.FirstOrDefault(f => f.Prefix == "luxonis");

                if (idsImage == null || baslerImage == null || luxonisImage == null)
                {
                    // Delete incomplete triplet
                    if (idsImage != null)
                        _manager.DeleteFile(idsImage.FullPath);
                    if (baslerImage != null)
                        _manager.DeleteFile(baslerImage.FullPath);
                    if (luxonisImage != null)
                        _manager.DeleteFile(luxonisImage.FullPath);

                    Debug.WriteLine($"Incomplete triplet for calibration image nr.: {group.Key} - deleted remaining images.");
                }
            }
        }


        /// <summary>
        /// Loads existing valid images into the flow panels.
        /// </summary>
        private void LoadExistingThumbnails()
        {

            if (!Directory.Exists(_manager.TempFolder))
                return;

            Shots = 0;

            // Clear the panels before repopulating
            flp_ids.Controls.Clear();
            flp_basler.Controls.Clear();
            flp_luxonis.Controls.Clear();

            var imageFiles = Directory.GetFiles(_manager.TempFolder, "*.bmp");

            // Group files by their numeric identifier
            var groupedFiles = imageFiles
                .Select(file =>
                {
                    string fileName = Path.GetFileNameWithoutExtension(file);
                    string[] parts = fileName.Split('_');

                    if (parts.Length == 3 && parts[1] == "CalibrationImage")
                    {
                        return new
                        {
                            FullPath = file,
                            Prefix = parts[0], // "ids", "basler", "luxonis"
                            Id = parts[2]      // The numeric part after "CalibrationImage"
                        };
                    }
                    else
                    {
                        Debug.WriteLine($"Invalid file name format: {fileName}");
                        return null;
                    }
                })
                .Where(x => x != null)
                .GroupBy(file => file.Id)
                .OrderBy(group => group.Key)
                .ToList();

            foreach (var group in groupedFiles)
            {
                var triplet = group.ToList();

                // Ensure all three images exist for the same ID
                var idsImage = triplet.FirstOrDefault(f => f.Prefix == "ids");
                var baslerImage = triplet.FirstOrDefault(f => f.Prefix == "basler");
                var luxonisImage = triplet.FirstOrDefault(f => f.Prefix == "luxonis");

                if (idsImage != null && baslerImage != null && luxonisImage != null)
                {
                    try
                    {
                        // Load images into memory streams to avoid locking the original files
                        using (var idsStream = new MemoryStream(File.ReadAllBytes(idsImage.FullPath)))
                        using (var baslerStream = new MemoryStream(File.ReadAllBytes(baslerImage.FullPath)))
                        using (var luxonisStream = new MemoryStream(File.ReadAllBytes(luxonisImage.FullPath)))
                        {
                            var idsBitmap = new Bitmap(idsStream);
                            var baslerBitmap = new Bitmap(baslerStream);
                            var luxonisBitmap = new Bitmap(luxonisStream);

                            // Add thumbnails to the flow panels
                            AddThumbnailToColumn(flp_ids, idsBitmap, idsImage.Prefix + "_CalibrationImage_" + idsImage.Id);
                            AddThumbnailToColumn(flp_basler, baslerBitmap, baslerImage.Prefix + "_CalibrationImage_" + baslerImage.Id);
                            AddThumbnailToColumn(flp_luxonis, luxonisBitmap, luxonisImage.Prefix + "_CalibrationImage_" + luxonisImage.Id);

                            Shots++;
                        }
                    }
                    catch (Exception ex)
                    {
                        Debug.WriteLine($"Failed to load image from triplet {group.Key}: {ex.Message}");
                    }
                }
            }
        }




        /// <summary>
        /// Add thumbnail of the screenshot to a specific flow panel
        /// </summary>
        /// <param name="panel"></param>
        /// <param name="image"></param>
        /// <param name="name"></param>
        private void AddThumbnailToColumn(FlowLayoutPanel panel, Bitmap image, string name)
        {
            if (image == null || image.Width == 0 || image.Height == 0)
                return;

            // Debug.WriteLine($"Image Dimensions: {image.Width}x{image.Height}");

            // Create a thumbnail from the image
            const int thumbnailWidth = 100;
            const int thumbnailHeight = 100;

            Bitmap thumbnail = new Bitmap(thumbnailWidth, thumbnailHeight);
            using (Graphics g = Graphics.FromImage(thumbnail))
            {
                g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                g.CompositingQuality = System.Drawing.Drawing2D.CompositingQuality.HighQuality;
                g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

                // Scale the image to fit within the thumbnail dimensions
                g.Clear(Color.Black); // Background color

                // Draw the image only if dimensions are valid
                Rectangle destRect = new Rectangle(0, 0, thumbnailWidth, thumbnailHeight);
                Rectangle srcRect = new Rectangle(0, 0, image.Width, image.Height);

                g.DrawImage(image, destRect, srcRect, GraphicsUnit.Pixel);
            }

            // Create a PictureBox to hold the thumbnail
            System.Windows.Forms.PictureBox pictureBox = new System.Windows.Forms.PictureBox
            {
                Name = name,
                Image = thumbnail,
                Size = new Size(thumbnailWidth, thumbnailHeight),
                SizeMode = PictureBoxSizeMode.StretchImage,
                BorderStyle = BorderStyle.FixedSingle,
                Margin = new Padding(3)
            };

            // Create a ToolTip for the PictureBox
            var toolTip = new ToolTip
            {
                AutoPopDelay = 5000, // How long the tooltip stays visible
                InitialDelay = 500,  // Delay before it appears
                ReshowDelay = 100,   // Delay before it reappears
                ShowAlways = true    // Show even if the form is inactive
            };

            // Set the ToolTip text to the filename
            toolTip.SetToolTip(pictureBox, name);

            // Add the PictureBox to the FlowLayoutPanel
            panel.Invoke(new Action(() => panel.Controls.Add(pictureBox)));
        }



        private void RefreshCalibrationTimestamp()
        {
            try
            {
                // Define calibration file paths
                var calibrationFiles = new Dictionary<string, System.Windows.Forms.Label>
                {
                    { "ids_calibration.yaml", lbl_idsCalibTimestamp },
                    { "basler_calibration.yaml", lbl_baslerCalibTimestamp },
                    { "luxonis_calibration.yaml", lbl_luxonisCalibTimestamp }
                };

                foreach (var file in calibrationFiles)
                {
                    UpdateCalibrationTimestamp(file.Key, file.Value);
                }
            }
            catch (Exception ex)
            {
                // Fallback in case of an error
                HandleCalibrationError();
                Debug.WriteLine(ex);
            }
        }

        private void UpdateCalibrationTimestamp(string fileName, System.Windows.Forms.Label label)
        {
            // Ensure UI updates are thread-safe
            if (InvokeRequired)
            {
                Invoke(new Action(() => UpdateCalibrationTimestamp(fileName, label)));
                return;
            }

            // Generate file path
            string filePath = Path.Combine(_manager.ConfigFolder, fileName);

            // Update the label text
            if (!File.Exists(filePath))
            {
                label.Text = "Last calibration: NEVER!";
            }
            else
            {
                string timestamp = File.GetLastWriteTime(filePath).ToString("dd/MM/yyyy HH:mm");
                label.Text = $"Last calibration: {timestamp}";
            }
        }

        private void HandleCalibrationError()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(HandleCalibrationError));
                return;
            }

            lbl_idsCalibTimestamp.Text = "Last calibration: dd/mm/yyyy hh:mm";
            lbl_baslerCalibTimestamp.Text = "Last calibration: dd/mm/yyyy hh:mm";
            lbl_luxonisCalibTimestamp.Text = "Last calibration: dd/mm/yyyy hh:mm";
        }


        private void RefreshFlowPanels()
        {
            if (InvokeRequired)
            {
                Invoke(new Action(RefreshFlowPanels)); // Ensure execution on the UI thread
                return;
            }

            flp_ids.Controls.Clear();
            flp_luxonis.Controls.Clear();
            flp_basler.Controls.Clear();

            LoadExistingThumbnails();

            flp_ids.Refresh();
            flp_luxonis.Refresh();
            flp_basler.Refresh();
        }


        private void RefreshBottomControls()
        {
            lbl_shots.Text = $"{Shots}/15";
            btn_calibrate.Enabled = _manager.StatusMqttClient == true && _manager.RequestScreenshots == false && Shots == 15;
            btn_clear.Enabled = Shots > 0;
            btn_acquire.Enabled = _manager.StatusMqttClient == true && _manager.RequestScreenshots == false && Shots < 15;
        }

    }
}
