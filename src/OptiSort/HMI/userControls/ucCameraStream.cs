using System;
using System.Drawing;
using System.IO;
using System.Text.Json;
using System.Windows.Forms;

namespace OptiSort
{
    public partial class ucCameraStream : UserControl
    {
        // streaming
        public string StreamTopic { get; set; }
        private readonly object _lock = new object(); // should be used each time different threads try access (write or read) shared resources
        private Bitmap _image;
        private DateTime _imgDate = DateTime.MinValue;
        private readonly System.Threading.Timer _timeoutTimer;

        // performance
        private int _frameCount = 0;
        private int _fps = 0;
        private DateTime _lastFpsUpdate = DateTime.Now;
        private int _latencyMilliseconds = 0;

        internal ucCameraStream()
        {
            InitializeComponent();

            // Create a transparent placeholder Bitmap
            _image = CreateTransparentBitmap(pictureBox.Width, pictureBox.Height);

            // Create timer
            _timeoutTimer = new System.Threading.Timer(OnTimeout, null, TimeSpan.Zero, TimeSpan.FromSeconds(2));
        }



        // This method will be called when a new MQTT message is received
        public void OnMessageReceived(string topic, JsonElement message)
        {
            if (topic == StreamTopic)
            {
                string base64Image = message.GetProperty("image").GetString();
                Bitmap image = JsonToBitmap(base64Image); // Decode the base64 image

                string timestampString = message.GetProperty("timestamp").GetString();
                DateTime messageTimestamp = DateTime.Parse(timestampString, null, System.Globalization.DateTimeStyles.RoundtripKind); // Decode timestamp

                lock (_lock) // Lock for thread safety
                {
                    // Dispose the old image if necessary
                    if (_image != null)
                    {
                        _image.Dispose(); // Dispose previous bitmap if needed
                    }
                    _image = image; // Set new image
                    _imgDate = DateTime.Now; // Update image date

                    // Calculate latency
                    TimeSpan latency = DateTime.Now - messageTimestamp;
                    _latencyMilliseconds = (int)latency.TotalMilliseconds;
                    _frameCount++; // Increment frame count
                }

                // Calculate FPS every second
                if ((DateTime.Now - _lastFpsUpdate).TotalSeconds >= 1)
                {
                    _fps = _frameCount;
                    _frameCount = 0;
                    _lastFpsUpdate = DateTime.Now;
                }

                // Only invalidate once per message to reduce excessive repaints
                pictureBox.Invalidate();
            }
        }



        private void pictureBox_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            Bitmap image;

            lock (_lock) // Lock for thread safety
            {
                image = _image; // Read the image safely
            }

            // Check for timeout condition
            if (_imgDate.AddSeconds(2) < DateTime.Now)
            {
                // Painting red cross on black background
                g.Clear(Color.Black);
                g.DrawLine(new Pen(Color.Red, 10), pictureBox.Width / 3, pictureBox.Height / 3, pictureBox.Width * 2 / 3, pictureBox.Height * 2 / 3);
                g.DrawLine(new Pen(Color.Red, 10), pictureBox.Width / 3, pictureBox.Height * 2 / 3, pictureBox.Width * 2 / 3, pictureBox.Height / 3);
            }
            else
            {
                if (image != null) // Only draw the image if it's not null
                {
                    g.DrawImage(image, new Rectangle(0, 0, pictureBox.Width, pictureBox.Height));
                    g.DrawString($"FPS: {_fps}", new Font("Arial", 16), Brushes.White, new PointF(10, 10)); // Draw FPS overlay
                    g.DrawString($"Latency: {_latencyMilliseconds} ms", new Font("Arial", 16), Brushes.White, new PointF(10, 40)); // Draw Latency overlay
                }
            }
        }


        // ------ SUPPORT ---------


        /// <summary>
        /// This method will be called periodically by the timer
        /// </summary>
        /// <param name="state"></param>
        private void OnTimeout(object state)
        {
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


        // ------- OLD METHOD ---------

        //private Task Client_ApplicationMessageReceivedAsync(MQTTnet.Client.MqttApplicationMessageReceivedEventArgs arg)
        //{
        //    try
        //    {
        //        if (arg.ReasonCode == MqttApplicationMessageReceivedReasonCode.Success)
        //            if (arg.ApplicationMessage.PayloadSegment.Count > 0)
        //                lock (_imgReceived)
        //                {
        //                    _imgLen = arg.ApplicationMessage.PayloadSegment.Count;
        //                    if (_imgLen > _imgReceived.Length)
        //                        _imgReceived = new byte[_imgLen];
        //                    Array.Copy(arg.ApplicationMessage.PayloadSegment.ToArray(), _imgReceived, _imgLen);
        //                    _imgReady = true;
        //                    _imgDate = DateTime.Now;
        //                }
        //    }
        //    catch (Exception) { }
        //    return Task.CompletedTask;
        //}

        //private void DrawImage()
        //{
        //    byte[] imgToDraw = new byte[1];
        //    int imgLen;
        //    int busy = 0;
        //    while (_stop == false)
        //    {
        //        try
        //        {
        //            if (_imgReady)
        //            {
        //                if (busy == 0)
        //                {
        //                    busy++;
        //                    lock (_imgReceived)
        //                    {
        //                        imgLen = _imgLen;
        //                        if (imgToDraw.Length < _imgLen)
        //                            imgToDraw = new byte[_imgLen];
        //                        Array.Copy(_imgReceived, imgToDraw, _imgLen);
        //                        _imgReady = false;
        //                    }

        //                    using (MemoryStream ms = new MemoryStream(imgToDraw, 0, imgLen))
        //                    {
        //                        _img = Image.FromStream(ms);
        //                        IAsyncResult r = BeginInvoke((MethodInvoker)delegate
        //                        {
        //                            using (Graphics g = pnlStream.CreateGraphics())
        //                            {
        //                                if (_imgDstRectangle.Width == 0)
        //                                    CalcSize(_img);

        //                                g.DrawImage(_img, _imgDstRectangle);
        //                            }
        //                            busy = 0;
        //                        });
        //                        if (r.AsyncWaitHandle.WaitOne(100))
        //                            busy = 0;
        //                        else
        //                            busy++;
        //                    }
        //                }
        //            }
        //            else if (_imgDate.AddSeconds(_timeout) < DateTime.Now)
        //            {
        //                if (busy == 0)
        //                {
        //                    busy++;
        //                    _imgDate = DateTime.Now.AddSeconds(5);
        //                    //se non ricevo immagini da più di 2 secondi disegno una grande X rossa
        //                    IAsyncResult r = BeginInvoke((MethodInvoker)delegate
        //                    {
        //                        using (Graphics g = pnlStream.CreateGraphics())
        //                        {
        //                            g.Clear(Color.Black);
        //                            g.DrawLine(new Pen(Color.Red, 10), pnlStream.Width / 3, pnlStream.Height / 3, pnlStream.Width * 2 / 3, pnlStream.Height * 2 / 3);
        //                            g.DrawLine(new Pen(Color.Red, 10), pnlStream.Width / 3, pnlStream.Height * 2 / 3, pnlStream.Width * 2 / 3, pnlStream.Height / 3);
        //                        }
        //                        busy = 0;
        //                    });
        //                    if (r.AsyncWaitHandle.WaitOne(100))
        //                        busy = 0;
        //                    else
        //                        busy++;
        //                }
        //            }
        //            if (busy > 20)
        //                busy = 0;
        //            else if (busy > 0)
        //                busy++;
        //        }
        //        catch (Exception ex)
        //        {
        //            if (ex is ObjectDisposedException)
        //                break;
        //        }
        //        Thread.Sleep(10);
        //    }
        //}

        //private void CalcSize(Image image)
        //{
        //    float nPercent = 1;
        //    int destX = 0;
        //    int destY = 0;
        //    float nPercentW = (float)pnlStream.Width / (float)image.Width;
        //    float nPercentH = (float)pnlStream.Height / (float)image.Height;
        //    if (nPercentH < nPercentW)
        //    {
        //        nPercent = nPercentH;
        //        destX = (int)((pnlStream.Width - image.Width * nPercent) / 2);
        //    }
        //    else
        //    {
        //        nPercent = nPercentW;
        //        destY = (int)((pnlStream.Height - image.Height * nPercent) / 2);
        //    }
        //    //_imgSrcRectangle = pnlStream.ClientRectangle;
        //    _imgDstRectangle = new Rectangle(destX, destY, (int)(image.Width * nPercent), (int)(image.Height * nPercent));
        //}
    }
}
