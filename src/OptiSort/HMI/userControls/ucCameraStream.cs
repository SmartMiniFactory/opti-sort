using ActiproSoftware.SyntaxEditor;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Windows.Forms;
using static System.Net.Mime.MediaTypeNames;

namespace OptiSort
{
    /// <summary>
    /// This is the last version I updated. However, this could never work, because OnMessageReceived is triggered by
    /// </summary>

    public partial class ucCameraStream : UserControl
    {
        private optisort_mgr _manager;

        public Bitmap Image { get; private set; }
        //private DateTime _imgDate = DateTime.MinValue;
        private readonly System.Threading.Timer _timeoutTimer;

        // Performance computation
        private int _frameCount = 0;
        private int _fps = 0;
        private DateTime _lastFpsUpdate = DateTime.Now;
        private int _latencyMilliseconds = 0;

        // Screenshot requests
        private Dictionary<string, Bitmap> _screenshotBuffer = new Dictionary<string, Bitmap>();
        private Dictionary<string, DateTime> _topicLastReceived = new Dictionary<string, DateTime>();
        private TimeSpan _topicTimeoutThreshold = TimeSpan.FromSeconds(2);

        internal ucCameraStream(optisort_mgr manager)
        {
            InitializeComponent();
            _manager = manager;

            // Create a transparent placeholder Bitmap
            Image = CreateTransparentBitmap(pictureBox.Width, pictureBox.Height);

            // Subscribe to bitmap queuing updates
            _manager.BitmapQueued += OnBitmapQueued;

            
        }


        
        private void OnBitmapQueued(string messageFromTopic)
        {
            // render Bitmap only when coming from the camera selected for streaming
            if (messageFromTopic == _manager.StreamingTopic)
            {
                pictureBox.Invalidate(); // Trigger re-paint
            }
        }

        


        private void pictureBox_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            Bitmap currentImage = null;
            DateTime published = DateTime.MinValue;
            DateTime received = DateTime.MinValue;

            if (_manager.StreamingTopic == Properties.Settings.Default.mqtt_topic_idsStream)
            {
                if (_manager.idsQueue.TryDequeue(out var item))
                {
                    currentImage = item.Frame;
                    published = item.messageTimestamp;
                    received = item.receptionTimestamp;
                }
                else // Queue is empty
                {
                    RenderTimeoutOverlay(g); 
                    return;
                }
            }

            if (_manager.StreamingTopic == Properties.Settings.Default.mqtt_topic_baslerStream)
            {
                if (_manager.baslerQueue.TryDequeue(out var item))
                {
                    currentImage = item.Frame;
                    published = item.messageTimestamp;
                    received = item.receptionTimestamp;
                }
                else // Queue is empty
                {
                    RenderTimeoutOverlay(g);
                    return;
                }
            }

            if (_manager.StreamingTopic == Properties.Settings.Default.mqtt_topic_luxonisStream)
            {
                if (_manager.luxonisQueue.TryDequeue(out var item))
                {
                    currentImage = item.Frame;
                    published = item.messageTimestamp;
                    received = item.receptionTimestamp;
                }
                else // Queue is empty
                {
                    RenderTimeoutOverlay(g);
                    return;
                }
            }

            // Check for timeout condition
            if (received.AddSeconds(2) < DateTime.Now)
            {
                RenderTimeoutOverlay(g);
            }
            else
            {
                if (currentImage != null && !currentImage.Size.IsEmpty) // Only draw the image if it's not null
                {
                    try
                    {
                        g.DrawImage(currentImage, new Rectangle(0, 0, pictureBox.Width, pictureBox.Height));
                        g.DrawString($"FPS: {_fps}", new Font("Arial", 16), Brushes.White, new PointF(10, 10)); // Draw FPS overlay
                        g.DrawString($"Latency: {_latencyMilliseconds} ms", new Font("Arial", 16), Brushes.White, new PointF(10, 40)); // Draw Latency overlay 
                        g.DrawString($"Topic: {_manager.StreamingTopic}", new Font("Arial", 16), Brushes.White, new PointF(10, 70)); // Draw Latency overlay 
                    }
                    catch (Exception ex)
                    {
                        Debug.WriteLine($"Error in pictureBox_Paint: {ex.Message}");
                    }
                }
            }
        }


        /// <summary>
        /// Rendering image 
        /// </summary>
        /// <param name="g"></param>
        private void RenderTimeoutOverlay(Graphics g)
        {
            if (_manager.StatusMqttClient)
            {
                g.Clear(Color.Black);
                using (var pen = new Pen(Color.Red, 10))
                {
                    g.DrawLine(pen, pictureBox.Width / 3, pictureBox.Height / 3, pictureBox.Width * 2 / 3, pictureBox.Height * 2 / 3);
                    g.DrawLine(pen, pictureBox.Width / 3, pictureBox.Height * 2 / 3, pictureBox.Width * 2 / 3, pictureBox.Height / 3);
                }
            }
            else
            {
                g.Clear(Color.Black);
                using (Brush backgroundBrush = new SolidBrush(Color.FromArgb(128, Color.Black)))
                {
                    g.FillRectangle(backgroundBrush, new Rectangle(0, (pictureBox.Height - 40) / 2, pictureBox.Width, 40));
                }

                using (Font font = new Font("Arial", 12, FontStyle.Bold))
                using (Brush textBrush = new SolidBrush(Color.White))
                {
                    string message = "Connect MQTT client for camera streaming";
                    SizeF textSize = g.MeasureString(message, font);
                    PointF textLocation = new PointF((pictureBox.Width - textSize.Width) / 2, (pictureBox.Height - textSize.Height) / 2);
                    g.DrawString(message, font, textBrush, textLocation);
                }
            }
        }



        /// <summary>
        /// Creates bitmap placeholder to instance an empty pictureBox
        /// </summary>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <returns></returns>
        private Bitmap CreateTransparentBitmap(int width, int height)
        {
            var bitmap = new Bitmap(width, height);
            using (var g = Graphics.FromImage(bitmap))
            {
                g.Clear(Color.Transparent);
            }
            return bitmap;
        }


        private void CalculatePerformance(DateTime published, DateTime received)
        {

            TimeSpan latency = DateTime.Now - received;
            _latencyMilliseconds = (int)latency.TotalMilliseconds;
            _frameCount++; // TODO: suspicius


            // Calculate FPS every second
            if ((DateTime.Now - _lastFpsUpdate).TotalSeconds >= 1)
            {
                _fps = _frameCount;
                _frameCount = 0;
                _lastFpsUpdate = DateTime.Now;
            }
        }


    }
}
