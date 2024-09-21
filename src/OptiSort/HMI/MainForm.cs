using System;
using System.Linq;
using System.Text;
using System.Threading;
using System.Windows.Forms;
using MQTTnet.Client;
using Ace.Adept.Server.Controls;
using Ace.Adept.Server.Motion;
using Ace.Core.Client;
using Ace.Core.Server.Device;
using Ace.Core.Server;
using Ace.Core.Client.Sim3d.Controls;
using CobraLibrary;
using System.ComponentModel;
using ActiproSoftware.Drawing;
using System.Collections.Concurrent;
using OptiSort;
using static HMI.HMI;
using System.Collections.Generic;
using System.Threading.Tasks;
using ActiproSoftware.SyntaxEditor.Addons.DotNet.Ast;
using Ace.Adept.Server.Device;
using Ace.Adept.Server.Motion.Robots;
using Ace.Core.Server.Motion;
using Ace.Core.Util;
using Ace.Core.Server.Event;
using System.Net.Sockets;

namespace HMI
{
    public partial class ucOptiSort : Form
    {

        /// <summary>
        /// single MQTT broker where at least two topics are present: 
        /// 1. image streaming (one per camera, incoming); python file
        /// 2. scara's pick location streaming (incoming); same python file that operates with openCV on image
        /// 
        /// UDP protocol for flexibowl's communication (outgoing); flexibowl library
        /// 
        /// TCP connection with scara robot (in/out); cobra library
        /// </summary>
        /// 

        // MQTT 
        IMqttClient _mqttClient;
        byte[] _byteReceived = new byte[1];
        string _msgReceived = string.Empty;
        bool _msgReady = false;
        int _byteLen;
        string _broker = "localhost";
        string _topic = "optisort/scara/target";
        int _port = 1883;


        public ucOptiSort()
        {
            InitializeComponent();

            // init combobox
            Log("Initializing cameras combobox");
            List<Cameras> camerasList = new List<Cameras>
            {
                new Cameras { ID = 0, Text = "Integrated Camera" },
                new Cameras { ID = 1, Text = "IDS" },
                new Cameras { ID = 2, Text = "Luxonics" },
                new Cameras { ID = 3, Text = "???" }
            };
            cmbCameras.DataSource = camerasList;
            cmbCameras.DisplayMember = "Text";
            cmbCameras.ValueMember = "ID";    
            cmbCameras.DataSource = camerasList;


            // init scara dgv
            Log("Initializing scara datagridview");
            Cobra cobra600 = new Cobra();
            ucScara ucScara = new ucScara(this);
            ucScara.Cobra600 = cobra600;
            ucScara.Dock = DockStyle.Fill;
            pnlScara.Controls.Clear();
            pnlScara.Controls.Add(ucScara);


            // init camera view
            // TODO: add selection from txtbox
            Log("Initializing cameras view");
            ucCameraStream cameraUC = new ucCameraStream();
            cameraUC.Dock = DockStyle.Fill;
            pnlCameraStream.Controls.Clear();
            pnlCameraStream.Controls.Add(cameraUC);

            InitializeConnections();
        }


        private async void InitializeConnections()
        {
            // MQTT connection
            // TODO: create dedicated mqtt connection class, organize better and standardize robot/camera
            lstLog.Items.Add("Connecting to MQTT broker");
            Task<bool> mqtt = ConnectMqtt(_broker, _port, _topic);
            bool connected = await mqtt;
            if (!connected)
            {
                Log("Failed to connect to MQTT broker");
                return;
            }
            Log("Connected to MQTT broker");            
        }


        public void Log(string msg)
        {
            lstLog.Items.Add(msg);
            lstLog.TopIndex = lstLog.Items.Count - 1;
        }


        // TODO: add summary
        private async Task<bool> ConnectMqtt(string broker, int port, string topic)
        {
            try
            {
                if (_mqttClient != null)
                {
                    if (_mqttClient.IsConnected)
                        await _mqttClient.DisconnectAsync();
                    _mqttClient.ApplicationMessageReceivedAsync -= Client_ApplicationMessageReceivedAsync;
                    _mqttClient.Dispose();
                    _mqttClient = null;
                }

                var mqttFactory = new MQTTnet.MqttFactory();
                _mqttClient = mqttFactory.CreateMqttClient();
                var options = new MqttClientOptionsBuilder()
                    .WithClientId("Cobra600")
                    .WithTcpServer(broker, port)
                    .WithProtocolVersion(MQTTnet.Formatter.MqttProtocolVersion.V500)
                    .WithWillRetain(false)
                    .WithWillQualityOfServiceLevel(MQTTnet.Protocol.MqttQualityOfServiceLevel.AtMostOnce)
                    .WithCleanSession()
                    .Build();

                _mqttClient.ConnectedAsync += new Func<MqttClientConnectedEventArgs, Task>(arg =>
                {
                    Log($"Listening on {topic} from {broker}");
                    var topicFilter = new MQTTnet.MqttTopicFilterBuilder().WithTopic(topic).Build();
                    MQTTnet.Client.MqttClientSubscribeOptions subOptions = new MqttClientSubscribeOptions();
                    subOptions.TopicFilters.Add(topicFilter);
                    return _mqttClient.SubscribeAsync(subOptions);
                });

                _mqttClient.DisconnectedAsync += new Func<MqttClientDisconnectedEventArgs, Task>(arg =>
                {
                    try
                    {
                        Log($"Stop control robot");
                    }
                    catch (Exception) { }
                    return Task.CompletedTask;
                });

                _mqttClient.ApplicationMessageReceivedAsync += Client_ApplicationMessageReceivedAsync;
                var tokenSource = new CancellationTokenSource(TimeSpan.FromSeconds(2));
                await _mqttClient.ConnectAsync(options, tokenSource.Token);
                tokenSource.Dispose();
                return true;
            }
            catch (Exception)
            {
                Log($"Error connecting to {broker}");
                return false;
            }
        }


        private void DisconnectMqtt()
        {
            try
            {
                if (_mqttClient != null)
                {
                    // TODO: l'arrivo delle coordinate sono legate alla connessione dell'mqtt. Se questo viene connesso, forse va pulita la tabella? Indagare sul senso della cosa di cui sotto
                    //_stop = true;
                    //// Stop the threads
                    //if (_thDefineLocation != null && _thDefineLocation.IsAlive)
                    //    _thDefineLocation.Join(2000);
                    //_thDefineLocation = null;

                    //if (_thReachLocation != null && _thReachLocation.IsAlive)
                    //    _thReachLocation.Join(2000);
                    //_thReachLocation = null;

                    // Disconnect the MQTT client
                    _mqttClient.ApplicationMessageReceivedAsync -= Client_ApplicationMessageReceivedAsync;
                    _mqttClient.DisconnectAsync().Wait();
                    _mqttClient.Dispose();
                    Log("Mqtt disconnected");
                }
            }
            catch (Exception ex)
            {
                Log(ex.ToString());
            }
        }


       

        // TODO: add summary
        private Task Client_ApplicationMessageReceivedAsync(MqttApplicationMessageReceivedEventArgs arg)
        {
            try
            {
                if (arg.ReasonCode == MqttApplicationMessageReceivedReasonCode.Success)
                    if (arg.ApplicationMessage.PayloadSegment.Count > 0)
                        lock (_byteReceived)
                        {
                            _byteLen = arg.ApplicationMessage.PayloadSegment.Count;
                            if (_byteLen > _byteReceived.Length)
                                _byteReceived = new byte[_byteLen];
                            Array.Copy(arg.ApplicationMessage.PayloadSegment.ToArray(), _byteReceived, _byteLen);
                            _msgReceived = Encoding.UTF8.GetString(_byteReceived, 0, _byteLen);
                            _msgReady = true;
                        }
            }
            catch (Exception) { }
            return Task.CompletedTask;
        }
    }
}
