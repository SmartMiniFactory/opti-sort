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

namespace HMI
{
    public partial class ucOptiSort : Form
    {   
        // MQTT
        IMqttClient _client;
        byte[] _byteReceived = new byte[1];
        string _msgReceived = string.Empty;
        int _byteLen;
        bool _msgReady = false;
        DateTime _msgDate = DateTime.MinValue;
        bool _stop = false;
        string _brokerIp = "localhost";
        string _topic = "optisort/scara/target";
        int _port = 1883;
        Thread _thDefineLocation;

        // Omron ACE
        static private IAceServer _aceServer;
        static private IAceClient _aceClient;
        static private IAdeptController _aceController;
        static private IAdeptRobot _aceRobot;
        static private IAbstractEndEffector endEffector;
        const string _robotIP = "10.90.90.60";
        const string _controllerIP = "127.0.0.1";
        static bool _robotIsMoving = false;
        static Thread _thReachLocation;
        static private SimulationContainerControl _simulationControl;
        static private ControlPanelManager _pendantManager;
        static private Transform3D _lastTarget = new Transform3D(0, 0, 0, 0, 0, 0);
        static private BindingList<Transform3D> _targetQueueList;


        public ucOptiSort()
        {
            InitializeComponent();

            // Create a binding list to store the target locations
            _targetQueueList = new BindingList<Transform3D>();

            // Set up the DataGridView
            this._targetQueue.AutoGenerateColumns = false;

            // Create and bind columns to properties
            DataGridViewTextBoxColumn xColumn = new DataGridViewTextBoxColumn
            {
                HeaderText = "DX",
                DataPropertyName = "DX",
                Width = 100 // Set fixed width
            };
            this._targetQueue.Columns.Add(xColumn);

            DataGridViewTextBoxColumn yColumn = new DataGridViewTextBoxColumn
            {
                HeaderText = "DY",
                DataPropertyName = "DY",
                Width = 100 // Set fixed width
            };
            this._targetQueue.Columns.Add(yColumn);

            DataGridViewTextBoxColumn zColumn = new DataGridViewTextBoxColumn
            {
                HeaderText = "DZ",
                DataPropertyName = "DZ",
                Width = 100 // Set auto-size mode
            };
            this._targetQueue.Columns.Add(zColumn);

            DataGridViewTextBoxColumn yawColumn = new DataGridViewTextBoxColumn
            {
                HeaderText = "Yaw",
                DataPropertyName = "Yaw",
                Width = 100 // Set auto-size mode
            };
            this._targetQueue.Columns.Add(yawColumn);

            DataGridViewTextBoxColumn pitchColumn = new DataGridViewTextBoxColumn
            {
                HeaderText = "Pitch",
                DataPropertyName = "Pitch",
                Width = 100 // Set auto-size mode
            };
            this._targetQueue.Columns.Add(pitchColumn);

            DataGridViewTextBoxColumn rollColumn = new DataGridViewTextBoxColumn
            {
                HeaderText = "Roll",
                DataPropertyName = "Roll",
                AutoSizeMode = DataGridViewAutoSizeColumnMode.Fill // Set auto-size mode

            };
            this._targetQueue.Columns.Add(rollColumn);

            // Bind the BindingList to the DataGridView
            this._targetQueue.DataSource = _targetQueueList;

            this.Disposed += UcScara_Disposed;
            this.HandleCreated += UcScara_HandleCreated;
            _targetQueue.Rows.Clear();

        }
        private void UcScara_HandleCreated(object sender, EventArgs e)
        {
            Restart();

            _stop = false;
            // Start a new thread to add the target locations to the queue
            _thDefineLocation = new Thread(AddToLocationQueue) { IsBackground = true };
            _thDefineLocation.Start();
            
            // Start a new thread to move the robot to the target locations
            _thReachLocation = new Thread(MoveToLoc) { IsBackground = true };
            _thReachLocation.Start();
        }
        private void UcScara_Disposed(object sender, EventArgs e)
        {
            try
            {
                if (_client != null)
                {
                    _stop = true;
                    // Stop the threads
                    if (_thDefineLocation != null && _thDefineLocation.IsAlive)
                        _thDefineLocation.Join(2000);
                    _thDefineLocation = null;

                    if (_thReachLocation != null && _thReachLocation.IsAlive)
                        _thReachLocation.Join(2000);
                    _thReachLocation = null;

                    // Disconnect the MQTT client
                    _client.ApplicationMessageReceivedAsync -= Client_ApplicationMessageReceivedAsync;
                    _client.DisconnectAsync().Wait();
                    _client.Dispose();

                    // Disconnect from Omron ACE
                    Cobra.Init.Disconnet(_aceController, _aceRobot);
                }
            }
            catch (Exception ex)
            {
                listBox.Items.Add(ex.ToString());
            }
        }
        public async void Restart()
        {
            // Connect to the MQTT broker
            await ConnectMqtt(_brokerIp, _port, _topic);

            // Connect to Omron ACE
            await ConnectAce();
        }
        private async System.Threading.Tasks.Task ConnectAce()
        {
            listBox.Items.Add("Connecting to the robot");
            (_aceController, _aceRobot, _aceServer) = Cobra.Init.Connect(_controllerIP, "OptiSort");
            listBox.Items.Add("Robot connected");
        }
        private async System.Threading.Tasks.Task ConnectMqtt(string broker, int port, string topic)
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
                }

                var mqttFactory = new MQTTnet.MqttFactory();
                _client = mqttFactory.CreateMqttClient();
                var options = new MQTTnet.Client.MqttClientOptionsBuilder()
                    .WithClientId("Cobra600")
                    .WithTcpServer(broker, port)
                    .WithProtocolVersion(MQTTnet.Formatter.MqttProtocolVersion.V500)
                    .WithWillRetain(false)
                    .WithWillQualityOfServiceLevel(MQTTnet.Protocol.MqttQualityOfServiceLevel.AtMostOnce)
                    .WithCleanSession()
                    .Build();

                _client.ConnectedAsync += new Func<MQTTnet.Client.MqttClientConnectedEventArgs, System.Threading.Tasks.Task>(arg =>
                {
                    listBox.Items.Add($"Listening on {topic} from {broker}");
                    var topicFilter = new MQTTnet.MqttTopicFilterBuilder().WithTopic(topic).Build();
                    MQTTnet.Client.MqttClientSubscribeOptions subOptions = new MQTTnet.Client.MqttClientSubscribeOptions();
                    subOptions.TopicFilters.Add(topicFilter);
                    return _client.SubscribeAsync(subOptions);
                });

                _client.DisconnectedAsync += new Func<MqttClientDisconnectedEventArgs, System.Threading.Tasks.Task>(arg =>
                {
                    try
                    {
                        listBox.Items.Add($"Stop control robot");
                    }
                    catch (Exception) { }
                    return System.Threading.Tasks.Task.CompletedTask;
                });

                _client.ApplicationMessageReceivedAsync += Client_ApplicationMessageReceivedAsync;
                var tokenSource = new CancellationTokenSource(TimeSpan.FromSeconds(2));
                await _client.ConnectAsync(options, tokenSource.Token);
                tokenSource.Dispose();
            }
            catch (Exception)
            {
                listBox.Items.Add($"Error connecting to {broker}");
            }
        }
        private System.Threading.Tasks.Task Client_ApplicationMessageReceivedAsync(MQTTnet.Client.MqttApplicationMessageReceivedEventArgs arg)
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
            return System.Threading.Tasks.Task.CompletedTask;
        }
        private void AddToLocationQueue()
        {
            int busy = 0;
            while (_stop == false)
            {
                try
                {
                    if (_msgReady)
                    {
                        if (busy == 0)
                        {
                            busy++;
                            lock (_msgReceived)
                            {   
                                // Remove the brackets from the message
                                string _msgClean = _msgReceived.Replace("[", "").Replace("]", "");
                                // Split the message into an array using the comma as separator
                                string[] _msgTarget = _msgClean.Split(',');

                                // Convert the string to double
                                double.TryParse(_msgTarget[0], out double _x);
                                double.TryParse(_msgTarget[1], out double _y);
                                double.TryParse(_msgTarget[2], out double _z);
                                double.TryParse(_msgTarget[3], out double _yaw);
                                double.TryParse(_msgTarget[4], out double _pitch);
                                double.TryParse(_msgTarget[5], out double _roll);                            

                                // Define a new target location
                                Transform3D _locTarget = new Transform3D(_x, _y, _z, _yaw, _pitch, _roll);
                                {
                                    if (_targetQueueList.Count == 0)
                                    {
                                        try
                                        {
                                            //_targetQueueList.Add(_locTarget);
                                            _targetQueueList.Add(_locTarget);
                                        }
                                        catch (Exception ex)
                                        {
                                            MessageBox.Show($"Error adding new entry: " + ex.ToString());
                                        }
                                        _lastTarget = _locTarget;
                                    }
                                    else if (_targetQueueList.Count > 0)
                                    {

                                        if (_lastTarget != _locTarget)
                                        {
                                            try
                                            {
                                                _targetQueueList.Add(_locTarget);
                                            }
                                            catch (Exception ex)
                                            {
                                                MessageBox.Show($"Error adding new entry: " + ex.ToString());
                                            }
                                            _lastTarget = _locTarget;
                                        }
                                    }
                                }
                                _msgReady = false;
                            }
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
        private void MoveToLoc()
        {
            int busy = 0;
            while (_stop == false)
            {
                    try
                    {
                    if (_targetQueueList.Count > 0 && _robotIsMoving == false)
                    {
                        _robotIsMoving = true;
                        if (busy == 0)
                        {
                                                        // Get the first element of the queue
                            Transform3D _locTarget = _targetQueueList[0];

                            textBoxX.Text = _locTarget.DX.ToString();
                            textBoxY.Text = _locTarget.DY.ToString();
                            textBoxZ.Text = _locTarget.DZ.ToString();
                            textBoxRoll.Text = _locTarget.Roll.ToString();
                            textBoxPitch.Text = _locTarget.Pitch.ToString();
                            textBoxYaw.Text = _locTarget.Yaw.ToString();

                            listBox.Items.Add("Moving to target: " + _locTarget);

                            
                            Cobra.Motion.Approach(_aceServer, _aceRobot, _locTarget, 20);
                            Cobra.Motion.CartesianMove(_aceServer, _aceRobot, _locTarget, true);
                            Cobra.Motion.Approach(_aceServer, _aceRobot, _locTarget, 20);

                            Thread.Sleep(1000);

                            lock (_targetQueueList)
                            {
                                try
                                {
                                    _targetQueueList.RemoveAt(0);
                                }
                                catch (Exception ex)
                                {
                                    MessageBox.Show($"Error removing an entry: " + ex.ToString());
                                }
                            }
                            
                            
                            listBox.Items.Add("Target reached");
                            busy++;
                        }
                        _robotIsMoving = false;  
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
    }
}
