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

        bool _stop = false;
        string _broker = "localhost";
        string _topic = "optisort/scara/target";
        int _port = 1883;
        Thread _thDefineLocation; 

        // Omron ACE
        static private IAceServer _aceServer;
        static private IAceClient _aceClient;
        static private IAdeptController _aceController;
        static private IAdeptRobot _aceRobot;
        static private IAbstractEndEffector _endEffector;
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

            // init combobox
            lstLog.Items.Add("Initializing cameras combobox");
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


            // init dgv
            lstLog.Items.Add("Initializing datagridview");
            _targetQueueList = new BindingList<Transform3D>();
            _targetQueue.AutoGenerateColumns = false;
            DataGridViewTextBoxColumn xColumn = new DataGridViewTextBoxColumn
                {HeaderText = "DX", DataPropertyName = "DX", Width = 100};
            DataGridViewTextBoxColumn yColumn = new DataGridViewTextBoxColumn
                {HeaderText = "DY", DataPropertyName = "DY", Width = 100};
            DataGridViewTextBoxColumn zColumn = new DataGridViewTextBoxColumn
                {HeaderText = "DZ", DataPropertyName = "DZ", Width = 100};
            DataGridViewTextBoxColumn yawColumn = new DataGridViewTextBoxColumn
                {HeaderText = "Yaw", DataPropertyName = "Yaw", Width = 100};
            DataGridViewTextBoxColumn pitchColumn = new DataGridViewTextBoxColumn
                {HeaderText = "Pitch", DataPropertyName = "Pitch", Width = 100};
            DataGridViewTextBoxColumn rollColumn = new DataGridViewTextBoxColumn
                {HeaderText = "Roll", DataPropertyName = "Roll", AutoSizeMode = DataGridViewAutoSizeColumnMode.Fill};
            
            _targetQueue.Columns.Add(xColumn);
            _targetQueue.Columns.Add(yColumn);
            _targetQueue.Columns.Add(zColumn);
            _targetQueue.Columns.Add(yawColumn);
            _targetQueue.Columns.Add(pitchColumn);
            _targetQueue.Columns.Add(rollColumn);
            _targetQueue.DataSource = _targetQueueList;
            _targetQueue.Rows.Clear();

            // init camera view
            // TODO: add selection from txtbox
            lstLog.Items.Add("Initializing cameras view");
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
                lstLog.Items.Add("Failed to connect to MQTT broker");
                return;
            }
            lstLog.Items.Add("Connected to MQTT broker");


            lstLog.Items.Add("Connecting to ACE Server");
            Task<bool> robot = Cobra.Init.Connect(_controllerIP, "OptiSort");

        }


        private async Task<bool> ConnectMqtt(string broker, int port, string topic)
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
                    lstLog.Items.Add($"Listening on {topic} from {broker}");
                    var topicFilter = new MQTTnet.MqttTopicFilterBuilder().WithTopic(topic).Build();
                    MQTTnet.Client.MqttClientSubscribeOptions subOptions = new MQTTnet.Client.MqttClientSubscribeOptions();
                    subOptions.TopicFilters.Add(topicFilter);
                    return _client.SubscribeAsync(subOptions);
                });

                _client.DisconnectedAsync += new Func<MqttClientDisconnectedEventArgs, System.Threading.Tasks.Task>(arg =>
                {
                    try
                    {
                        lstLog.Items.Add($"Stop control robot");
                    }
                    catch (Exception) { }
                    return System.Threading.Tasks.Task.CompletedTask;
                });

                _client.ApplicationMessageReceivedAsync += Client_ApplicationMessageReceivedAsync;
                var tokenSource = new CancellationTokenSource(TimeSpan.FromSeconds(2));
                await _client.ConnectAsync(options, tokenSource.Token);
                tokenSource.Dispose();
                return true;
            }
            catch (Exception)
            {
                lstLog.Items.Add($"Error connecting to {broker}");
                return false;
            }
        }


        // TODO: temporary copied from Cobra class
        public static (IAdeptController, IAdeptRobot, IAceServer) Connect(string _controllerIP, string _controllerName, string _endEffectorName = "Suction Cup", string _remotingName = "ace")
        {
            // Initialize remoting infrastructure
            RemotingUtil.InitializeRemotingSubsystem(true, _callbackPort);

            // Connect to ACE
            IAceServer aceServer = (IAceServer)RemotingUtil.GetRemoteServerObject(typeof(IAceServer), _remotingName, _controllerIP, _remotingPort);
            Console.WriteLine("Connected to server");

            IAceClient aceClient = new AceClient(aceServer);
            Console.WriteLine("Connected to client");

            aceClient.InitializePlugins(null);

            if (_newWorkspace)
            {
                // Clear the workspace
                aceServer.Clear();
            }

            aceServer.EmulationMode = _emulation;

            // Get the available controllers
            IList<IAceObject> availableControllers = aceServer.Root.Filter(new ObjectTypeFilter(typeof(IAdeptController)), true);
            if (availableControllers.Count == 0)
            {
                // Handle the case when no controllers are available
                Console.WriteLine("No controllers available.\nCreating a new one.");

                // Create a controller and robot and establish the connection
                controller = aceServer.Root.AddObjectOfType(typeof(IAdeptController), _controllerName) as IAdeptController;

            }
            else
            {
                // Connect to the first available controller
                Console.WriteLine("Connecting to {0}", availableControllers.FirstOrDefault());
                _controller = availableControllers.FirstOrDefault() as IAdeptController;
            }

            // Get the available robots
            IList<IAceObject> availableRobots = aceServer.Root.Filter(new ObjectTypeFilter(typeof(IAdeptRobot)), true);
            if (availableRobots.Count == 0)
            {
                // Handle the case when no controllers are available
                Console.WriteLine("No robots available.\nCreating a new one.");

                // Create a controller and robot and establish the connection
                robot = aceServer.Root.AddObjectOfType(typeof(ICobra600), "R1 Cobra600") as IAdeptRobot;
            }
            else
            {
                // Connect to the first available robot
                Console.WriteLine("Connecting to {0}", availableRobots.FirstOrDefault());
                robot = availableRobots.FirstOrDefault() as IAdeptRobot;
            }

            // Configure the robot and controller IP addresses
            controller.Address = RobotIP;
            robot.Controller = controller;

            // Get the available end-effectors
            IList<IAceObject> availableEndEffectors = aceServer.Root.Filter(new ObjectTypeFilter(typeof(IIODrivenEndEffector)), true);
            if (availableEndEffectors.Count == 0)
            {
                // Handle the case when no controllers are available
                Console.WriteLine("No End-effectors available.\nCreating a new one.");

                // Create an end-effector object
                endEffector = aceServer.Root.AddObjectOfType(typeof(IIODrivenEndEffector), _endEffectorName) as IAbstractEndEffector;
            }
            else
            {
                // Connect to the first available end-effector
                Console.WriteLine("Connecting to {0}", availableEndEffectors.FirstOrDefault());
                endEffector = availableEndEffectors.FirstOrDefault() as IIODrivenEndEffector;
            }

            // Associate the end-effector with the robot
            robot.SelectedEndEffector = endEffector;
            robot.EndEffectorGripSignal = 98;
            robot.EndEffectorReleaseSignal = 97;

            // Enable the controller
            controller.Enabled = true;

            // If not calibrated, calibrate the robot
            try
            {
                if (robot.IsCalibrated == false)
                {
                    robot.Power = true;
                    controller.HighPower = true;
                    Console.WriteLine("Enabling power to {0}. Please press the button on the front panel.", robot);
                    robot.Calibrate();
                    controller.Calibrate();

                }
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
            }

            return (controller, robot, aceServer);
        }







        //private void UcScara_HandleCreated(object sender, EventArgs e)
        //{
        //    Restart();

        //    _stop = false;
        //    // Start a new thread to add the target locations to the queue
        //    _thDefineLocation = new Thread(AddToLocationQueue) { IsBackground = true };
        //    _thDefineLocation.Start();
            
        //    // Start a new thread to move the robot to the target locations
        //    _thReachLocation = new Thread(MoveToLoc) { IsBackground = true };
        //    _thReachLocation.Start();
        //}

        //private void UcScara_Disposed(object sender, EventArgs e)
        //{
        //    try
        //    {
        //        if (_client != null)
        //        {
        //            _stop = true;
        //            // Stop the threads
        //            if (_thDefineLocation != null && _thDefineLocation.IsAlive)
        //                _thDefineLocation.Join(2000);
        //            _thDefineLocation = null;

        //            if (_thReachLocation != null && _thReachLocation.IsAlive)
        //                _thReachLocation.Join(2000);
        //            _thReachLocation = null;

        //            // Disconnect the MQTT client
        //            _client.ApplicationMessageReceivedAsync -= Client_ApplicationMessageReceivedAsync;
        //            _client.DisconnectAsync().Wait();
        //            _client.Dispose();

        //            // Disconnect from Omron ACE
        //            Cobra.Init.Disconnet(_aceController, _aceRobot);
        //        }
        //    }
        //    catch (Exception ex)
        //    {
        //        lstLog.Items.Add(ex.ToString());
        //    }
        //}

        //public async void Restart()
        //{
        //    // Connect to the MQTT broker
        //    await ConnectMqtt("localhost", _port, _topic);

        //    // Connect to Omron ACE
        //    await ConnectAce();
        //}

        //private async Task ConnectAce()
        //{
        //    lstLog.Items.Add("Connecting to the robot");
        //    await (_aceController, _aceRobot, _aceServer) = Cobra.Init.Connect(_controllerIP, "OptiSort");
        //    lstLog.Items.Add("Robot connected");
        //}
        

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

                            lstLog.Items.Add("Moving to target: " + _locTarget);

                            
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
                            
                            
                            lstLog.Items.Add("Target reached");
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

        private void btnConnect_Click(object sender, EventArgs e)
        {
            ConnectAce();
        }
    }
}
