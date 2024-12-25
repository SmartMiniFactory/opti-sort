using Microsoft.Win32;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
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
                        AddThumbnailToColumn(flp_ids, image);

                    if (topic == Properties.Settings.Default.mqtt_topic_baslerStream)
                        AddThumbnailToColumn(flp_basler, image);

                    if (topic == Properties.Settings.Default.mqtt_topic_luxonisStream)
                        AddThumbnailToColumn(flp_luxonis, image);
                }

                _frmMain._ucCameraStream.ScreenshotsReady -= SaveShot;

                _shots++;
                lbl_shots.Text = _shots.ToString() + "/15";
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

    }
}
