using Ace.Process.Server;
using MQTTnet.Client;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace OptiSort
{
    public partial class ucCameraStream : UserControl
    {
        // MQTT settings
        string _topic = "optisort/luxonis/stream";
        string _broker = "localhost"; //"10.10.238.20";
        int _port = 1883;
        string _clientId = Guid.NewGuid().ToString();
        int _timeout = 2; // seconds
        IMqttClient _client;


        // Image decoding settings
        byte[] _imgReceived = new byte[1];
        int _imgLen;
        bool _imgReady = false;
        DateTime _imgDate = DateTime.MinValue;
        Image _img;
        Rectangle _imgRectengle;
        Rectangle _imgDstRectangle;
        bool _stop = false;

        Thread _thDrawImage;

        public ucCameraStream()
        {
            InitializeComponent();
            InitializeVideoStream();

            _stop = false;
            _thDrawImage = new Thread(DrawImage) { IsBackground = true };
            _thDrawImage.Start();
        }

        public async void InitializeVideoStream()
        {
            await Connect();
        }

        private async Task Connect()
        {
            try
            {
                if (_client != null)
                {
                    if (_client.IsConnected)
                        await _client.DisconnectAsync();
                    _client.ApplicationMessageReceivedAsync -= Client_ApplicationMessageReceivedAsync;
                    _client.Dispose();
                    _client = null;

                    _imgDstRectangle.Width = 0;
                }

                var mqttFactory = new MQTTnet.MqttFactory();
                _client = mqttFactory.CreateMqttClient();
                var options = new MQTTnet.Client.MqttClientOptionsBuilder()
                    .WithClientId(_clientId)
                    .WithTcpServer(_broker, _port)
                    .WithProtocolVersion(MQTTnet.Formatter.MqttProtocolVersion.V500)
                    .WithWillRetain(false)
                    .WithWillQualityOfServiceLevel(MQTTnet.Protocol.MqttQualityOfServiceLevel.AtMostOnce)
                    .WithCleanSession()
                    .Build();

                _client.ConnectedAsync += new Func<MQTTnet.Client.MqttClientConnectedEventArgs, Task>(arg =>
                {
                    var topicFilter = new MQTTnet.MqttTopicFilterBuilder().WithTopic(_topic).Build();
                    MQTTnet.Client.MqttClientSubscribeOptions subOptions = new MQTTnet.Client.MqttClientSubscribeOptions();
                    subOptions.TopicFilters.Add(topicFilter);
                    return _client.SubscribeAsync(subOptions);
                });

                _client.DisconnectedAsync += new Func<MqttClientDisconnectedEventArgs, Task>(arg =>
                {
                    try
                    {
                        _imgRectengle.Width = 0;
                    }
                    catch (Exception) { }
                    return Task.CompletedTask;
                });

                _client.ApplicationMessageReceivedAsync += Client_ApplicationMessageReceivedAsync;
                var tokenSource = new CancellationTokenSource(TimeSpan.FromSeconds(2));
                await _client.ConnectAsync(options, tokenSource.Token);
                tokenSource.Dispose();
            }
            catch (Exception)
            {

            }
        }

        private Task Client_ApplicationMessageReceivedAsync(MQTTnet.Client.MqttApplicationMessageReceivedEventArgs arg)
        {
            try
            {
                if (arg.ReasonCode == MqttApplicationMessageReceivedReasonCode.Success)
                    if (arg.ApplicationMessage.PayloadSegment.Count > 0)
                        lock (_imgReceived)
                        {
                            _imgLen = arg.ApplicationMessage.PayloadSegment.Count;
                            if (_imgLen > _imgReceived.Length)
                                _imgReceived = new byte[_imgLen];
                            Array.Copy(arg.ApplicationMessage.PayloadSegment.ToArray(), _imgReceived, _imgLen);
                            _imgReady = true;
                            _imgDate = DateTime.Now;
                        }
            }
            catch (Exception) { }
            return Task.CompletedTask;
        }

        private void DrawImage()
        {
            byte[] imgToDraw = new byte[1];
            int imgLen;
            int busy = 0;
            while (_stop == false)
            {
                try
                {
                    if (_imgReady)
                    {
                        if (busy == 0)
                        {
                            busy++;
                            lock (_imgReceived)
                            {
                                imgLen = _imgLen;
                                if (imgToDraw.Length < _imgLen)
                                    imgToDraw = new byte[_imgLen];
                                Array.Copy(_imgReceived, imgToDraw, _imgLen);
                                _imgReady = false;
                            }

                            using (MemoryStream ms = new MemoryStream(imgToDraw, 0, imgLen))
                            {
                                _img = Image.FromStream(ms);
                                IAsyncResult r = BeginInvoke((MethodInvoker)delegate
                                {
                                    using (Graphics g = pnlStream.CreateGraphics())
                                    {
                                        if (_imgDstRectangle.Width == 0)
                                            CalcSize(_img);

                                        g.DrawImage(_img, _imgDstRectangle);
                                    }
                                    busy = 0;
                                });
                                if (r.AsyncWaitHandle.WaitOne(100))
                                    busy = 0;
                                else
                                    busy++;
                            }
                        }
                    }
                    else if (_imgDate.AddSeconds(_timeout) < DateTime.Now)
                    {
                        if (busy == 0)
                        {
                            busy++;
                            _imgDate = DateTime.Now.AddSeconds(5);
                            //se non ricevo immagini da più di 2 secondi disegno una grande X rossa
                            IAsyncResult r = BeginInvoke((MethodInvoker)delegate
                            {
                                using (Graphics g = pnlStream.CreateGraphics())
                                {
                                    g.Clear(Color.Black);
                                    g.DrawLine(new System.Drawing.Pen(Color.Red, 10), pnlStream.Width / 3, pnlStream.Height / 3, pnlStream.Width * 2 / 3, pnlStream.Height * 2 / 3);
                                    g.DrawLine(new System.Drawing.Pen(Color.Red, 10), pnlStream.Width / 3, pnlStream.Height * 2 / 3, pnlStream.Width * 2 / 3, pnlStream.Height / 3);
                                }
                                busy = 0;
                            });
                            if (r.AsyncWaitHandle.WaitOne(100))
                                busy = 0;
                            else
                                busy++;
                        }
                    }
                    if (busy > 20)
                        busy = 0;
                    else if (busy > 0)
                        busy++;
                }
                catch (Exception ex)
                {
                    if (ex is System.ObjectDisposedException)
                        break;
                }
                Thread.Sleep(10);
            }
        }

        private void CalcSize(Image image)
        {
            float nPercent = 1;
            int destX = 0;
            int destY = 0;
            float nPercentW = (float)pnlStream.Width / (float)image.Width;
            float nPercentH = (float)pnlStream.Height / (float)image.Height;
            if (nPercentH < nPercentW)
            {
                nPercent = nPercentH;
                destX = (int)((pnlStream.Width - image.Width * nPercent) / 2);
            }
            else
            {
                nPercent = nPercentW;
                destY = (int)((pnlStream.Height - image.Height * nPercent) / 2);
            }
            //_imgSrcRectangle = pnlStream.ClientRectangle;
            _imgDstRectangle = new Rectangle(destX, destY, (int)(image.Width * nPercent), (int)(image.Height * nPercent));
        }
    }
}
