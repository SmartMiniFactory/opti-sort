using System.ComponentModel;
using System.Drawing;
using System.Windows.Forms;

namespace OptiSort.userControls
{
    public partial class ucRobotSimulation : UserControl
    {

        private optisort_mgr _manager;

        internal ucRobotSimulation(optisort_mgr manager)
        {
            InitializeComponent();
            _manager = manager;

            PaintIndicationRobot3DView();
            
            // Attach to property changed event to detect change in scara status and repaint robot panel
            _manager.PropertyChanged += ScaraConnectionRefresh;
        }


        private void ScaraConnectionRefresh(object sender, PropertyChangedEventArgs e)
        {
            if(e.PropertyName == nameof(_manager.StatusScara))
                PaintIndicationRobot3DView();
        }

        private void PaintIndicationRobot3DView()
        {
            // Create a bitmap to draw on, using the panel's size
            Bitmap bmp = new Bitmap(pnlRobotView.Width, pnlRobotView.Height);
            using (Graphics g = Graphics.FromImage(bmp))
            {
                g.Clear(Color.Transparent); // Clear with transparent background

                // Draw the semi-transparent black background rectangle
                using (Brush backgroundBrush = new SolidBrush(Color.FromArgb(128, Color.Black)))
                {
                    g.FillRectangle(backgroundBrush, new Rectangle(0, (bmp.Height - 40) / 2, bmp.Width, 40));
                }

                // Set up font and brush for the message
                using (Font font = new Font("Arial", 12, FontStyle.Bold))
                using (Brush textBrush = new SolidBrush(Color.White))
                {
                    string message = "Connect ACE Server for 3D render";

                    // Measure the size of the message to center it
                    SizeF textSize = g.MeasureString(message, font);
                    PointF textLocation = new PointF((bmp.Width - textSize.Width) / 2, (bmp.Height - textSize.Height) / 2);

                    // Draw the message centered on the bitmap
                    g.DrawString(message, font, textBrush, textLocation);
                }
            }

            // Create a PictureBox to display the bitmap
            System.Windows.Forms.PictureBox pictureBox = new System.Windows.Forms.PictureBox
            {
                Dock = DockStyle.Fill,   // Fill the panel
                BackColor = Color.Transparent,
                Image = bmp,
                SizeMode = PictureBoxSizeMode.Zoom
            };

            // Add the PictureBox to the panel's controls
            pnlRobotView.Controls.Add(pictureBox);
        }
    }
}
