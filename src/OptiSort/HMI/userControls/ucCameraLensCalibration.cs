using Ace.UIBuilder.Client.Controls.Tools.WindowsForms;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Windows.Forms;

namespace OptiSort.userControls
{
    public partial class ucCameraLensCalibration : UserControl
    {
        private frmMain _frmMain;
        private int _shots;
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

        private FileSystemWatcher _fileWatcher;
        private bool _isFileWatcherPaused;

        private static string _tempFolder = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\Temp"));
        private static string _configFolder = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\Config"));


        // TODO: move general methods to app manager


        public ucCameraLensCalibration(frmMain frmMain)
        {
            InitializeComponent();

            Shots = 0;
            
            _frmMain = frmMain;

            this.Load += ucLensDistortionCalibration_Load; // Attach the Load event to call the method after the control is fully initialized
        }

        private void ucLensDistortionCalibration_Load(object sender, EventArgs e)
        {
            RefreshCalibrationTimestamp();
            LoadExistingThumbnails();
            SetupFileSystemWatcher();
        }

        private void btn_acquire_Click(object sender, EventArgs e)
        {
            if (!_frmMain.StatusMqttClient)
            {
                MessageBox.Show("Enable camera streaming first (MQTT service)");
                return;
            }

            Cursor = Cursors.WaitCursor;
            _frmMain._ucCameraStream.ScreenshotsReady += SaveShot;
            _frmMain._ucCameraStream.RequestScreenshots();
        }

        private void btn_calibrate_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;

            _frmMain.Log("Calibration process in progress...", false, false);

            string script = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\Cameras\workingScripts\camerasCalibration.py"));
            RunPythonScript(script);
        }

        private void btn_clear_Click(object sender, EventArgs e)
        {
            ClearTempDirectory(); 
        }

        private void btn_home_Click(object sender, EventArgs e)
        {
            _fileWatcher.Dispose();
            ucManualControl ucManualControl = new ucManualControl(_frmMain);
            _frmMain.AddNewUc(ucManualControl);
        }


        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------


        /// <summary>
        /// When this is triggered, three screenshots are ready to be processed. The method saves the screenshots as files and adds their thumbnails to the flow panels
        /// </summary>
        /// <param name="screenshots"></param>
        private void SaveShot(Dictionary<string, Bitmap> screenshots)
        {
            // Handle the screenshots when they are ready
            Invoke(new Action(() =>
            {
                // Get the earliest available index (gap in the sequence)
                int earliestAvailableIndex = GetEarliestAvailableIndex();

                foreach (var kvp in screenshots)
                {
                    string topic = kvp.Key;
                    Bitmap image = kvp.Value;

                    string prefix = string.Empty;
                    FlowLayoutPanel panel = null;

                    if (topic == Properties.Settings.Default.mqtt_topic_idsStream)
                    {
                        prefix = "ids";
                        panel = flp_ids;
                    }
                    else if (topic == Properties.Settings.Default.mqtt_topic_baslerStream)
                    {
                        prefix = "basler";
                        panel = flp_basler;
                    }
                    else if (topic == Properties.Settings.Default.mqtt_topic_luxonisStream)
                    {
                        prefix = "luxonis";
                        panel = flp_luxonis;
                    }

                    if (!string.IsNullOrEmpty(prefix) && panel != null)
                    {
                        string filename = $"{prefix}_CalibrationImage_{earliestAvailableIndex.ToString("D2")}";
                        SaveBitmapAsFile(image, filename);
                        AddThumbnailToColumn(panel, image, filename);
                    }
                }

                _frmMain._ucCameraStream.ScreenshotsReady -= SaveShot;
                Shots++;
                Cursor = Cursors.Default;
            }));
        }



        /// <summary>
        /// Identifies the earliest available index in the TEMP folder by analyzing existing filenames.
        /// </summary>
        /// <returns>The earliest available numeric index.</returns>
        private int GetEarliestAvailableIndex()
        {
            
            if (!Directory.Exists(_tempFolder))
                return 1; // Start from 1 if the directory does not exist

            var existingFiles = Directory.GetFiles(_tempFolder, "*_CalibrationImage_*.bmp");

            // Extract indices from filenames
            var indices = existingFiles
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
                .OrderBy(index => index) // Sort in ascending order
                .ToList();

            // If no files exist, start with index 1
            if (indices.Count == 0)
                return 1;

            // Find the first missing index (gap)
            for (int i = 1; i <= indices.Count + 1; i++)
            {
                if (!indices.Contains(i))
                {
                    // Return the first missing index
                    return i;
                }
            }

            // If no gaps, return the next index after the largest one
            return indices.Max() + 1;
        }


        /// <summary>
        /// Parse py's output and interpret result (calibration failed or succeded)
        /// </summary>
        /// <param name="pythonOutput"></param>
        private void CalibrationResult(string pythonOutput)
        {

            PauseFileWatcher();

            try
            {
                var jsonDocument = JsonDocument.Parse(pythonOutput); // Parse JSON
                var root = jsonDocument.RootElement; // Access elements

                // Extract "bad_image" -> "ids", "basler", "luxonis" into C# arrays
                int[] badImage_ids = JsonSerializer.Deserialize<int[]>(root.GetProperty("bad_image").GetProperty("ids").GetRawText());
                int[] badImage_basler = JsonSerializer.Deserialize<int[]>(root.GetProperty("bad_image").GetProperty("basler").GetRawText());
                int[] badImage_luxonis = JsonSerializer.Deserialize<int[]>(root.GetProperty("bad_image").GetProperty("luxonis").GetRawText());

                // Extract "calibration_fail" -> "ids", "basler", "luxonis" into booleans
                bool calibrationFail_ids = root.GetProperty("calibration_fail").GetProperty("ids").GetBoolean();
                bool calibrationFail_basler = root.GetProperty("calibration_fail").GetProperty("basler").GetBoolean();
                bool calibrationFail_luxonis = root.GetProperty("calibration_fail").GetProperty("luxonis").GetBoolean();

                if (calibrationFail_ids || calibrationFail_luxonis || calibrationFail_basler)
                {
                    MessageBox.Show($"OPENCV CALIBRATION FAILED!\nIDS: {calibrationFail_ids}\nBASLER: {calibrationFail_basler}\nLUXONIS: {calibrationFail_luxonis}");
                }

                if (badImage_ids.Length == 0 && badImage_basler.Length == 0 && badImage_luxonis.Length == 0)
                {
                    Shots = 0;
                    ClearTempDirectory();
                    RefreshCalibrationTimestamp();
                    MessageBox.Show($"CALIBRATION SUCCEDED!");
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
                        DeleteImage("ids_CalibrationImage_" + id.ToString("D2") + ".bmp");
                        DeleteImage("basler_CalibrationImage_" + id.ToString("D2") + ".bmp");
                        DeleteImage("luxonis_CalibrationImage_" + id.ToString("D2") + ".bmp");
                    }

                    Shots -= badImages_all.Count;
                    MessageBox.Show($"BAD IMAGES DETECTED!\nCalibration failed because the chess board was not found in {badImages_all.Count} triplets. These were deleted. Please proceed to retaking the images again.");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error parsing JSON: {ex.Message}");
            }
        
            ResumeFileWatcher();
            Cursor = Cursors.Default;
            _frmMain.Log("Calibration process ended", false, false);
        }

        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------

        /// <summary>
        /// Deletes incomplete triplets in the TEMP folder.
        /// </summary>
        private void DeleteIncompleteTriplets()
        {
            if (!Directory.Exists(_tempFolder))
                return;

            var imageFiles = Directory.GetFiles(_tempFolder, "*.bmp");

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
                        DeleteImage(idsImage.FullPath);
                    if (baslerImage != null)
                        DeleteImage(baslerImage.FullPath);
                    if (luxonisImage != null)
                        DeleteImage(luxonisImage.FullPath);

                    Debug.WriteLine($"Incomplete triplet for calibration image nr.: {group.Key} - deleted remaining images.");
                }
            }
        }

        /// <summary>
        /// Loads existing valid images into the flow panels.
        /// </summary>
        private void LoadExistingThumbnails()
        {
            if (!Directory.Exists(_tempFolder))
                return;

            // Clear the panels before repopulating
            flp_ids.Controls.Clear();
            flp_basler.Controls.Clear();
            flp_luxonis.Controls.Clear();

            var imageFiles = Directory.GetFiles(_tempFolder, "*.bmp");

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

       
        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------

        private void RefreshCalibrationTimestamp()
        {
            try
            {

                if (!File.Exists(Path.Combine(_configFolder, "ids_calibration.yaml")))
                    this.Invoke(new Action(() => lbl_idsCalibTimestamp.Text = "Last calibration: NEVER!"));
                else
                    this.Invoke(new Action(() => lbl_idsCalibTimestamp.Text = "Last calibration: " + File.GetLastWriteTime(Path.Combine(_configFolder, "ids_calibration.yaml")).ToString("dd/MM/yyyy HH:mm")));

                if (!File.Exists(Path.Combine(_configFolder, "basler_calibration.yaml")))
                    this.Invoke(new Action(() => lbl_baslerCalibTimestamp.Text = "Last calibration: NEVER!"));
                else
                    this.Invoke(new Action(() => lbl_baslerCalibTimestamp.Text = "Last calibration: " + File.GetLastWriteTime(Path.Combine(_configFolder, "basler_calibration.yaml")).ToString("dd/MM/yyyy HH:mm")));

                if (!File.Exists(Path.Combine(_configFolder, "luxonis_calibration.yaml")))
                    this.Invoke(new Action(() => lbl_luxonisCalibTimestamp.Text = "Last calibration: NEVER!"));
                else
                    this.Invoke(new Action(() => lbl_luxonisCalibTimestamp.Text = "Last calibration: " + File.GetLastWriteTime(Path.Combine(_configFolder, "luxonis_calibration.yaml")).ToString("dd/MM/yyyy HH:mm")));
            }
            catch (Exception ex)
            {
                this.Invoke(new Action(() => lbl_idsCalibTimestamp.Text = "Last calibration: dd/mm/yyyy hh:mm" ));
                this.Invoke(new Action(() => lbl_baslerCalibTimestamp.Text = "Last calibration: dd/mm/yyyy hh:mm"));
                this.Invoke(new Action(() => lbl_luxonisCalibTimestamp.Text = "Last calibration: dd/mm/yyyy hh:mm"));
                Debug.WriteLine(ex);
            }
        }

        private void RefreshFlowPanels()
        {
            flp_ids.Controls.Clear();
            flp_luxonis.Controls.Clear();
            flp_basler.Controls.Clear();
            Shots = 0;

            LoadExistingThumbnails();
            
            flp_ids.Refresh();
            flp_luxonis.Refresh();
            flp_basler.Refresh();
        }

        private void RefreshBottomControls()
        {
            lbl_shots.Text = $"{Shots}/15";
            btn_acquire.Enabled = Shots < 15;
            btn_calibrate.Enabled = Shots == 15;
            btn_clear.Enabled = Shots > 0;
        }

        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------

        /// <summary>
        /// Creates an agent that checks on updates within the TEMP folder to update the flow panels respectively
        /// </summary>
        private void SetupFileSystemWatcher()
        {
            _fileWatcher = new FileSystemWatcher
            {
                Path = _tempFolder, // Directory to watch
                Filter = "*.bmp", // Monitor only BMP files
                NotifyFilter = NotifyFilters.FileName | NotifyFilters.LastWrite | NotifyFilters.CreationTime
            };

            // Subscribe to events
            //_fileWatcher.Created += OnFileChanged; // too aggressive (three images cannot be added simultaneously, not even by pasting)
            _fileWatcher.Deleted += OnFileChanged;
            //_fileWatcher.Changed += OnFileChanged; // too aggressive (three images cannot changed simultaneously)
            //_fileWatcher.Renamed += OnFileRenamed; // too aggressive (three images cannot renamed simultaneously)

            // Enable the watcher
            _fileWatcher.EnableRaisingEvents = true;
        }

        private void PauseFileWatcher()
        {
            _isFileWatcherPaused = true;
        }

        private void ResumeFileWatcher()
        {
            _isFileWatcherPaused = false;
            LoadExistingThumbnails();
        }

        private void OnFileChanged(object sender, FileSystemEventArgs e)
        {
            if (_isFileWatcherPaused)
                return;

            if (InvokeRequired)
            {
                Invoke(new Action(() => OnFileChanged(sender, e)));
                return;
            }

            Debug.WriteLine($"File {e.ChangeType}: {e.FullPath}");

            DeleteIncompleteTriplets();
            RefreshFlowPanels();
        }

        /*
        private void OnFileRenamed(object sender, RenamedEventArgs e)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => OnFileRenamed(sender, e)));
                return;
            }

            Debug.WriteLine($"File Renamed: {e.OldFullPath} -> {e.FullPath}");

            // Refresh thumbnails to reflect the current folder's content
            RefreshFlowPanels();
        }
        */

        /// <summary>
        /// Export screenshots as bmp files to the TEMP folder
        /// </summary>
        /// <param name="bitmap"></param>
        /// <param name="fileName"></param>
        /// <exception cref="ArgumentNullException"></exception>
        /// <exception cref="ArgumentException"></exception>
        private static void SaveBitmapAsFile(Bitmap bitmap, string fileName)
        {
            if (bitmap == null)
            {
                throw new ArgumentNullException(nameof(bitmap), "Bitmap cannot be null.");
            }

            if (string.IsNullOrWhiteSpace(fileName))
            {
                throw new ArgumentException("File name cannot be null or empty.", nameof(fileName));
            }


            if (!Directory.Exists(_tempFolder))
            {
                Directory.CreateDirectory(_tempFolder);
            }

            // Combine the output folder with the file name
            string tempFilePath = Path.Combine(_tempFolder, fileName + ".bmp");

            // Save the bitmap as a BMP file
            bitmap.Save(tempFilePath, ImageFormat.Bmp);

            Console.WriteLine($"Bitmap saved as BMP at: {tempFilePath}");
        }


        /// <summary>
        /// Delete all the contents of the TEMP folder (calibration screenshots of the cameras)
        /// </summary>
        /// <exception cref="ArgumentException"></exception>
        private static void ClearTempDirectory()
        {
            if (string.IsNullOrWhiteSpace(_tempFolder))
            {
                throw new ArgumentException("Temporary directory path cannot be null or empty.", nameof(_tempFolder));
            }

            try
            {
                // Ensure the temporary directory exists
                if (Directory.Exists(_tempFolder))
                {
                    // Get all files in the temporary directory
                    var files = Directory.GetFiles(_tempFolder);

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
                    var directories = Directory.GetDirectories(_tempFolder);
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
                    Directory.CreateDirectory(_tempFolder);
                    Console.WriteLine($"Temporary directory created: {_tempFolder}");
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


        /// <summary>
        /// Delete specific image from the TEMP folder, shift other indexes to occupy that groupId
        /// </summary>
        /// <param name="name"></param>
        private static void DeleteImage(string name)
        {
            try
            {
                string filePath = Path.Combine(_tempFolder, name);
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

        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------

        /// <summary>
        /// Run a python script in background and collect output
        /// </summary>
        /// <param name="scriptPath"></param>
        private void RunPythonScript(string scriptPath)
        {
            var pythonExe = @"C:\Users\dylan\AppData\Local\Programs\Python\Python312\python.exe"; // Path to Python executable
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
                CalibrationResult(outputBuilder.ToString()); // call explicitly the calibration result method
            }
        }
    }
}
