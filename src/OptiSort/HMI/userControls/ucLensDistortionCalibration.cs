using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Windows.Forms;

namespace OptiSort.userControls
{
    public partial class ucLensDistortionCalibration : UserControl
    {
        private frmMain _frmMain;
        private int _shots = 0;

        public ucLensDistortionCalibration(frmMain frmMain)
        {
            InitializeComponent();
            
            _frmMain = frmMain;

            ClearTempDirectory();
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
            string script = Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\Cameras\tests\test_VideoStreamingPublisher.py"));
            RunPythonScript(script);
        }

        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------


        // Event handler to add images to the queue
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
                        AddThumbnailToColumn(flp_ids, image);
                        string filename = "ids_CalibrationImage_" + (_shots + 1);
                        SaveBitmapAsFile(image, filename);
                    }

                    if (topic == Properties.Settings.Default.mqtt_topic_baslerStream)
                    {
                        AddThumbnailToColumn(flp_basler, image);
                        string filename = "basler_CalibrationImage_" + (_shots + 1);
                        SaveBitmapAsFile(image, filename);
                    }

                    if (topic == Properties.Settings.Default.mqtt_topic_luxonisStream)
                    {
                        AddThumbnailToColumn(flp_luxonis, image);
                        string filename = "luxonis_CalibrationImage_" + (_shots + 1);
                        SaveBitmapAsFile(image, filename);
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



        private void AddThumbnailToColumn(FlowLayoutPanel panel, Bitmap image)
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
            PictureBox pictureBox = new PictureBox
            {
                Image = thumbnail,
                Size = new Size(thumbnailWidth, thumbnailHeight),
                SizeMode = PictureBoxSizeMode.StretchImage,
                BorderStyle = BorderStyle.FixedSingle,
                Margin = new Padding(3)
            };

            // Add the PictureBox to the FlowLayoutPanel
            panel.Invoke(new Action(() => panel.Controls.Add(pictureBox)));
        }


        public static void SaveBitmapAsFile(Bitmap bitmap, string fileName)
        {
            if (bitmap == null)
            {
                throw new ArgumentNullException(nameof(bitmap), "Bitmap cannot be null.");
            }

            if (string.IsNullOrWhiteSpace(fileName))
            {
                throw new ArgumentException("File name cannot be null or empty.", nameof(fileName));
            }

            // Create a folder named "Temp" if it doesn't exist
            string tempPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\Temp");

            if (!Directory.Exists(tempPath))
            {
                Directory.CreateDirectory(tempPath);
            }

            // Combine the output folder with the file name
            string tempFilePath = Path.Combine(tempPath, fileName + ".png");

            // Save the bitmap as a PNG file
            bitmap.Save(tempFilePath, ImageFormat.Bmp);

            Console.WriteLine($"Bitmap saved as PNG at: {tempFilePath}");
        }


        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------

        public static void ClearTempDirectory()
        {

            string directoryPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\Temp");

            if (string.IsNullOrWhiteSpace(directoryPath))
            {
                throw new ArgumentException("Temporary directory path cannot be null or empty.", nameof(directoryPath));
            }

            try
            {
                // Ensure the temporary directory exists
                if (Directory.Exists(directoryPath))
                {
                    // Get all files in the temporary directory
                    var files = Directory.GetFiles(directoryPath);

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
                    var directories = Directory.GetDirectories(directoryPath);
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
                    Directory.CreateDirectory(directoryPath);
                    Console.WriteLine($"Temporary directory created: {directoryPath}");
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

        public static void RunPythonScript(string scriptPath)
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

            process.OutputDataReceived += (sender, e) => Console.WriteLine(e.Data);
            process.ErrorDataReceived += (sender, e) => Console.WriteLine("ERROR: " + e.Data);

            process.Start();
            process.BeginOutputReadLine();
            process.BeginErrorReadLine();
            process.WaitForExit();
        }
    }
}
