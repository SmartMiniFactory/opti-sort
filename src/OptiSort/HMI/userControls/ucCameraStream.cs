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
        private readonly object _lock = new object(); // should be used each time different threads try access (write or read) shared resources

        public Bitmap Image { get; private set; }

        private DateTime _imgDate = DateTime.MinValue;
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

            // Attach user control to mqtt-generated events
            _manager.MqttClient.MessageReceived += OnMessageReceived;

            // Create timer
            _timeoutTimer = new System.Threading.Timer(OnTimeout, null, TimeSpan.Zero, _topicTimeoutThreshold);
        }


        /// <summary>
        /// Called each time a new MQTT message is received, based on the topic reconstructs the image streaming and paints the control
        /// </summary>
        /// <param name="topic"></param>
        /// <param name="message"></param>
        public void OnMessageReceived(string topic, JsonElement message)
        {
            // coverting json into bitmap
            string base64Image = message.GetProperty("image").GetString();
            Bitmap image = JsonToBitmap(base64Image); // Decode the base64 image

            string timestampString = message.GetProperty("timestamp").GetString();
            DateTime messageTimestamp = DateTime.Parse(timestampString, null, System.Globalization.DateTimeStyles.RoundtripKind); // Decode timestamp

            lock (_lock)
            {
                // Regular streaming logic
                if (topic == _manager.StreamingTopic)
                {
                    Bitmap previousImage = Image;
                    Image = image;
                    _imgDate = DateTime.Now;
                    previousImage?.Dispose();

                    TimeSpan latency = DateTime.Now - messageTimestamp;
                    _latencyMilliseconds = (int)latency.TotalMilliseconds;
                    _frameCount++;
                }

                // Handle screenshot request
                if (_manager.RequestScreenshots)
                {
                    // Add the current image to the buffer for the topic
                    _screenshotBuffer[topic] = image;
                    _topicLastReceived[topic] = DateTime.Now;

                    // Check if all required topics have been captured
                    var requiredTopics = new[]
                    {
                        Properties.Settings.Default.mqtt_topic_idsStream,
                        Properties.Settings.Default.mqtt_topic_luxonisStream,
                        Properties.Settings.Default.mqtt_topic_baslerStream
                    };

                    if (requiredTopics.All(_screenshotBuffer.ContainsKey))
                    {
                        // Notify that screenshots are ready
                        _manager.NotifyScreenshotsReady(_screenshotBuffer);
                        _manager.RequestScreenshots = false;
                    }
                }
            }

            // Calculate FPS every second
            if ((DateTime.Now - _lastFpsUpdate).TotalSeconds >= 1)
            {
                _fps = _frameCount;
                _frameCount = 0;
                _lastFpsUpdate = DateTime.Now;
            }

            // trigger repaint
            pictureBox.Invalidate();
        }


        /// <summary>
        /// Method to trigger screenshot requests
        /// </summary>
        public void RequestScreenshots()
        {
            lock (_lock)
            {
                _manager.RequestScreenshots = true;
                _screenshotBuffer.Clear(); // Clear buffer for new screenshots

                // Initialize last received times for all required topics
                var requiredTopics = new[]
                {
                    Properties.Settings.Default.mqtt_topic_idsStream,
                    Properties.Settings.Default.mqtt_topic_luxonisStream,
                    Properties.Settings.Default.mqtt_topic_baslerStream
                };

                // save creation time in timeout buffer
                foreach (var topic in requiredTopics)
                {
                    if (!_topicLastReceived.ContainsKey(topic))
                    {
                        _topicLastReceived[topic] = DateTime.Now;
                    }
                }
            }
        }


        private void pictureBox_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            Bitmap currentImage;

            lock (_lock) // Lock for thread safety
            {
                currentImage = Image; // Read the image safely
            }

            // Check for timeout condition
            if (_imgDate.AddSeconds(2) < DateTime.Now)
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
        /// This method will be called periodically by the timer
        /// </summary>
        /// <param name="state"></param>
        private void OnTimeout(object state)
        {
            if (_manager.RequestScreenshots)
            {
                lock (_lock)
                {
                    DateTime currentTime = DateTime.Now;

                    var requiredTopics = new[]
                    {
                        Properties.Settings.Default.mqtt_topic_idsStream,
                        Properties.Settings.Default.mqtt_topic_luxonisStream,
                        Properties.Settings.Default.mqtt_topic_baslerStream
                    };

                    // Check if any topic has timed out
                    foreach (var topic in requiredTopics)
                    {
                        if (!_topicLastReceived.ContainsKey(topic) || currentTime - _topicLastReceived[topic] > TimeSpan.FromSeconds(4))
                        {
                            // Replace the image with a placeholder bitmap for this topic
                            Console.WriteLine($"{topic} timed out");
                            _screenshotBuffer[topic] = CreateTransparentBitmap(pictureBox.Width, pictureBox.Height);
                        }
                    }

                    // If all required topics have been captured (including placeholders), invoke ScreenshotsReady
                    if (requiredTopics.All(t => _screenshotBuffer.ContainsKey(t)))
                    {
                        // Reset request flag
                        _manager.NotifyScreenshotsReady(_screenshotBuffer);
                        _manager.RequestScreenshots = false;
                    }
                }
            }

            pictureBox.Invalidate(); // Trigger re-paint
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
        /// Creates bitmap placeholder to instance an empty pictureBox for the constructor
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

    }
}
