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
    public partial class ucLensDistortionCalibration : UserControl
    {
        private frmMain _frmMain;
        private int _shots = 0;

        private static string _tempFolder = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\Temp"));
        private static string _configFolder = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\Config"));

        // TODO: create dedicated method to update/invoke shots label update and buttons enable/disable
        // TODO: test overall functioning after declaring path singletones
        // TODO: move general methods to app manager

        public ucLensDistortionCalibration(frmMain frmMain)
        {
            InitializeComponent();
            
            _frmMain = frmMain;

            this.Load += ucLensDistortionCalibration_Load; // Attach the Load event to call the method after the control is fully initialized
        }

        private void ucLensDistortionCalibration_Load(object sender, EventArgs e)
        {
            RefreshCalibrationTimestamp();
            LoadExistingThumbnails();
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
            string script = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\Cameras\workingScripts\camerasCalibration.py"));
            RunPythonScript(script);
        }

        private void btn_clear_Click(object sender, EventArgs e)
        {
            ClearTempDirectory(); 
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
                foreach (var kvp in screenshots)
                {
                    string topic = kvp.Key;
                    Bitmap image = kvp.Value;

                    if (topic == Properties.Settings.Default.mqtt_topic_idsStream)
                    {
                        string filename = "ids_CalibrationImage_" + (_shots + 1);
                        SaveBitmapAsFile(image, filename);
                        AddThumbnailToColumn(flp_ids, image, filename);
                    }

                    if (topic == Properties.Settings.Default.mqtt_topic_baslerStream)
                    {
                        string filename = "basler_CalibrationImage_" + (_shots + 1);
                        SaveBitmapAsFile(image, filename);
                        AddThumbnailToColumn(flp_basler, image, filename);
                    }

                    if (topic == Properties.Settings.Default.mqtt_topic_luxonisStream)
                    {
                        string filename = "luxonis_CalibrationImage_" + (_shots + 1);
                        SaveBitmapAsFile(image, filename);
                        AddThumbnailToColumn(flp_luxonis, image, filename);
                    }
                }

                _frmMain._ucCameraStream.ScreenshotsReady -= SaveShot;

                _shots++;
                lbl_shots.Text = _shots.ToString() + "/15";

                if (_shots == 15)
                {
                    btn_acquire.Enabled = false;
                    btn_calibrate.Enabled = true;
                }

                Cursor = Cursors.Default;
 
            }));
        }


        /// <summary>
        /// Parse py's output and interpret result (calibration failed or succeded)
        /// </summary>
        /// <param name="pythonOutput"></param>
        private void CalibrationResult(string pythonOutput)
        {
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
                        DeleteImage("ids_CalibrationImage_" + id);
                        RemoveThumbnailFromColumn(flp_ids, "ids_CalibrationImage_" + id);

                        DeleteImage("basler_CalibrationImage_" + id);
                        RemoveThumbnailFromColumn(flp_basler, "basler_CalibrationImage_" + id);

                        DeleteImage("luxonis_CalibrationImage_" + id);
                        RemoveThumbnailFromColumn(flp_luxonis, "luxonis_CalibrationImage_" + id);
                    }

                    flp_basler.Refresh();
                    flp_ids.Refresh();
                    flp_luxonis.Refresh();

                    _shots -= badImages_all.Count;
                    
                    this.Invoke(new Action(() => lbl_shots.Text = _shots.ToString() + "/15"));
                    this.Invoke(new Action(() => btn_acquire.Enabled = true));
                    this.Invoke(new Action(() => btn_calibrate.Enabled = false));

                    MessageBox.Show($"BAD IMAGES DETECTED!\nCalibration failed because the grid was not found in {badImages_all.Count} images. Please retake these images.");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error parsing JSON: {ex.Message}");
            }
        
            Cursor = Cursors.Default;
        }

        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------

        /// <summary>
        /// Check TEMP folder, if some items with a specific name format are present, load them into the flow panels
        /// </summary>
        private void LoadExistingThumbnails()
        {
            string tempPath = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\Temp"));

            if (!Directory.Exists(tempPath))
                return;

            // Get all image files in the Temp folder
            var imageFiles = Directory.GetFiles(tempPath, "*.bmp");

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
                .ToDictionary(group => group.Key, group => group.ToList());

            foreach (var group in groupedFiles)
            {
                var triplet = group.Value;

                // Check if all three images exist for the same ID
                var idsImage = triplet.FirstOrDefault(f => f.Prefix == "ids");
                var baslerImage = triplet.FirstOrDefault(f => f.Prefix == "basler");
                var luxonisImage = triplet.FirstOrDefault(f => f.Prefix == "luxonis");

                if (idsImage != null && baslerImage != null && luxonisImage != null)
                {
                    try
                    {
                        // All three images exist, add their thumbnails
                        AddThumbnailToColumn(flp_ids, new Bitmap(idsImage.FullPath), idsImage.Prefix + "_" + idsImage.Id);
                        AddThumbnailToColumn(flp_basler, new Bitmap(baslerImage.FullPath), baslerImage.Prefix + "_" + baslerImage.Id);
                        AddThumbnailToColumn(flp_luxonis, new Bitmap(luxonisImage.FullPath), luxonisImage.Prefix + "_" + luxonisImage.Id);

                        _shots++;
                    }
                    catch (Exception ex)
                    {
                        Debug.WriteLine($"Failed to load image from triplet {group.Key}: {ex.Message}");
                    }
                }
                else
                {
                    // If any image is missing, delete the remaining images in the triplet
                    if (idsImage != null)
                        DeleteImage(idsImage.FullPath);
                    if (baslerImage != null)
                        DeleteImage(baslerImage.FullPath);
                    if (luxonisImage != null)
                        DeleteImage(luxonisImage.FullPath);

                    Debug.WriteLine($"Incomplete triplet for ID {group.Key} - deleted remaining images.");
                }
            }

            // Update the shots label and enable/disable buttons based on the count
            lbl_shots.Text = $"{_shots}/15";
            btn_acquire.Enabled = _shots < 15;
            btn_calibrate.Enabled = _shots == 15;
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

            Debug.WriteLine($"Image Dimensions: {image.Width}x{image.Height}");

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

            // Add the PictureBox to the FlowLayoutPanel
            panel.Invoke(new Action(() => panel.Controls.Add(pictureBox)));
        }

        /// <summary>
        /// Remove specific thumbnail from a specific flow panel
        /// </summary>
        /// <param name="panel"></param>
        /// <param name="name"></param>
        private static void RemoveThumbnailFromColumn(FlowLayoutPanel panel, string name)
        {
            foreach (Control ctrl in panel.Controls)
            {
                if (ctrl.Name == name) // Identify the control by name
                {
                    panel.Controls.Remove(ctrl);
                    ctrl.Dispose();
                    break;
                }
            }
        }

        

        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------

        private void RefreshCalibrationTimestamp()
        {
            try
            {

                if (!File.Exists(Path.Combine(_configFolder, "ids_calibration.yaml")))
                    this.Invoke(new Action(() => lbl_idsCalibTimestamp.Text = "Last calibration: dd/mm/yyyy hh:mm" ));
                else
                    this.Invoke(new Action(() => lbl_idsCalibTimestamp.Text = "Last calibration: " + File.GetLastWriteTime(Path.Combine(_configFolder, "ids_calibration.yaml")).ToString("dd/MM/yyyy HH:mm")));

                if (!File.Exists(Path.Combine(_configFolder, "basler_calibration.yaml")))
                    this.Invoke(new Action(() => lbl_baslerCalibTimestamp.Text = "Last calibration: dd/mm/yyyy hh:mm"));
                else
                    this.Invoke(new Action(() => lbl_baslerCalibTimestamp.Text = "Last calibration: " + File.GetLastWriteTime(Path.Combine(_configFolder, "basler_calibration.yaml")).ToString("dd/MM/yyyy HH:mm")));

                if (!File.Exists(Path.Combine(_configFolder, "luxonis_calibration.yaml")))
                    this.Invoke(new Action(() => lbl_luxonisCalibTimestamp.Text = "Last calibration: dd/mm/yyyy hh:mm"));
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

        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------


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
            string tempFilePath = Path.Combine(_tempFolder, fileName + ".png");

            // Save the bitmap as a PNG file
            bitmap.Save(tempFilePath, ImageFormat.Bmp);

            Console.WriteLine($"Bitmap saved as PNG at: {tempFilePath}");
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
        /// Delete specific image from the TEMP folder
        /// </summary>
        /// <param name="name"></param>
        private static void DeleteImage(string name)
        {
            try
            {
                string filePath = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\Temp\" + name + ".bmp"));
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
