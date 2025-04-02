using ActiproSoftware.SyntaxEditor;
using Newtonsoft.Json.Linq;
using System;
using System.Collections.Generic;
using System.ComponentModel;
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
        private string _mqttClient = Properties.Settings.Default.mqtt_client;

        // Performance computation
        private Stopwatch _stopwatch = Stopwatch.StartNew();
        private int _frameCount = 0;
        private double _mqttLatency;
        private double _renderingLatency;
        private double _totalLatency;
        private double _fps;
        private double _maxAllowedLatency = 5000; // milliseconds
        
        private bool _mqttLatencyError = false;

        // watchdog
        private Classes.Watchdog _watchdog;


        internal ucCameraStream(optisort_mgr manager)
        {
            InitializeComponent();
            _manager = manager;

            // Subscribe to bitmap queuing updates
            _manager.BitmapQueued += OnBitmapQueued;

            // Subscribe to propery changed
            _manager.PropertyChanged += OnPropertyChange;

            // Create watchdog to trigger repaint in case of mqtt connection loss
            _watchdog = new Classes.Watchdog(500);
            _watchdog.Elapsed += Watchdog_Elapsed;
            _watchdog.Start();

            // Start the timer to reset FPS metrics periodically
            StartMetricsResetTimer();
        }

        private void Watchdog_Elapsed(object sender, EventArgs e)
        {
            pictureBox.Invalidate();
        }

        private void OnPropertyChange(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(_manager.StatusMqttClient))
            {
                pictureBox.Invalidate(); // Trigger re-paint
                _mqttLatencyError = false;
            }
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
            int queueCount = 0;


            if (_manager.StreamingTopic == Properties.Settings.Default.mqtt_topic_idsStream)
            {
                if (_manager._idsQueue.TryDequeue(out var item))
                {
                    queueCount = _manager._idsQueue.Count;
                    currentImage = item.Frame;
                    published = item.messageTimestamp;
                    received = item.receptionTimestamp;
                    CalculateLatency(received, published);
                }
                else // Queue is empty
                {
                    RenderTimeoutOverlay(g);
                    return;
                }
            }

            if (_manager.StreamingTopic == Properties.Settings.Default.mqtt_topic_baslerStream)
            {
                if (_manager._baslerQueue.TryDequeue(out var item))
                {
                    queueCount = _manager._baslerQueue.Count;
                    currentImage = item.Frame;
                    published = item.messageTimestamp;
                    received = item.receptionTimestamp;
                    CalculateLatency(received, published);

                }
                else // Queue is empty
                {
                    RenderTimeoutOverlay(g);
                    return;
                }
            }

            if (_manager.StreamingTopic == Properties.Settings.Default.mqtt_topic_luxonisStream)
            {
                if (_manager._luxonisQueue.TryDequeue(out var item))
                {
                    queueCount = _manager._luxonisQueue.Count;
                    currentImage = item.Frame;
                    published = item.messageTimestamp;
                    received = item.receptionTimestamp;
                    CalculateLatency(received, published);
                }
                else // Queue is empty
                {
                    RenderTimeoutOverlay(g);
                    return;
                }
            }

            _frameCount++;

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

                        double overload = Math.Round(_totalLatency / _maxAllowedLatency * 100, 1, MidpointRounding.AwayFromZero);
                        if (overload > 100 && _mqttLatencyError == false)
                        {
                            _mqttLatencyError = true;
                            _ = _manager.DisconnectMqttClient(_mqttClient);
                            _manager.NonBlockingMessageBox("MQTT service interrupted because of a latency overload. Please try restarting the service", "Overload!", MessageBoxIcon.Warning);
                            return;
                        }

                        int y = 10;
                        int increment = 20;
                        g.DrawImage(currentImage, new Rectangle(0, 0, pictureBox.Width, pictureBox.Height));
                        g.DrawString($"Topic: {_manager.StreamingTopic}", new Font("Arial", 10), Brushes.White, new PointF(10, y));
                        g.DrawString($"FPS: {_fps}", new Font("Arial", 10), Brushes.White, new PointF(10, y = y + increment));
                        g.DrawString($"Mqtt latency: {_mqttLatency} ms", new Font("Arial", 10), Brushes.White, new PointF(10, y = y + increment));
                        g.DrawString($"Rendering latency: {_renderingLatency} ms", new Font("Arial", 10), Brushes.White, new PointF(10, y = y + increment));
                        
                        if (overload <= 20)
                            g.DrawString($"Latency overload: {overload}%", new Font("Arial", 10), Brushes.White, new PointF(10, y = y + increment));
                        else if(overload > 20 && overload <= 60)
                            g.DrawString($"Latency overload: {overload}%", new Font("Arial", 10), Brushes.Yellow, new PointF(10, y = y + increment));
                        else
                            g.DrawString($"Latency overload: {overload}%", new Font("Arial", 10), Brushes.Red, new PointF(10, y = y + increment));

                        g.DrawString($"Queued frames: {queueCount}", new Font("Arial", 10), Brushes.White, new PointF(10, y = y + increment));
                        _watchdog.Reset();
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


        private void CalculateLatency(DateTime rxTimestamp, DateTime msgTimestamp)
        {
            _mqttLatency = Math.Round((rxTimestamp - msgTimestamp).TotalMilliseconds, 2, MidpointRounding.AwayFromZero);
            _renderingLatency = Math.Round((DateTime.Now - rxTimestamp).TotalMilliseconds, 2, MidpointRounding.AwayFromZero);
            _totalLatency = Math.Round((DateTime.Now - msgTimestamp).TotalMilliseconds, 2, MidpointRounding.AwayFromZero);
        }

        // Periodic reset to ensure FPS calculation is accurate over time
        private void ResetMetrics()
        {
            _fps = Math.Round(_frameCount / _stopwatch.Elapsed.TotalSeconds, 0, MidpointRounding.AwayFromZero);
            _frameCount = 0;
            _stopwatch.Restart();
        }

        // Start a timer to reset FPS metrics every second
        private void StartMetricsResetTimer()
        {
            var timer = new System.Timers.Timer(1000); // 1 second interval
            timer.Elapsed += (sender, e) => ResetMetrics();
            timer.Start();
        }

    }
}
