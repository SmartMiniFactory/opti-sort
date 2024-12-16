using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace OptiSort.userControls
{
    public partial class ucLensDistortionCalibration : UserControl
    {

        private int _shots = 0;

        public ucLensDistortionCalibration()
        {
            InitializeComponent();
        }

        private void btn_acquire_Click(object sender, EventArgs e)
        {
            Image img1 = CaptureImageFromCamera(1);
            AddThumbnailToColumn(flp_ids, img1);
            AddThumbnailToColumn(flp_basler, img1);
            AddThumbnailToColumn(flp_luxonis, img1);

            _shots++;
            lbl_shots.Text = _shots.ToString() + "/15";

        }

        private Image CaptureImageFromCamera(int cameraId)
        {
            // Replace this with your camera capture logic
            Bitmap dummyImage = new Bitmap(100, 100);
            using (Graphics g = Graphics.FromImage(dummyImage))
            {
                g.Clear(Color.AliceBlue);
                g.DrawString($"Cam {cameraId}", new Font("Arial", 12), Brushes.Black, new PointF(10, 40));
            }
            return dummyImage;
        }

        private void AddThumbnailToColumn(FlowLayoutPanel column, Image img)
        {
            var thumbnail = img.GetThumbnailImage(100, 100, null, IntPtr.Zero);
            PictureBox pb = new PictureBox();
            pb.Image = thumbnail;
            pb.SizeMode = PictureBoxSizeMode.Zoom;
            pb.Size = new Size(100, 100);
            pb.Margin = new Padding(5);
            column.Controls.Add(pb);
        }
    }
}
