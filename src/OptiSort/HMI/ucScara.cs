using Ace.Adept.Server.Controls;
using Ace.Adept.Server.Device;
using Ace.Adept.Server.Motion.Robots;
using Ace.Adept.Server.Motion;
using Ace.Core.Client;
using Ace.Core.Server;
using Ace.Core.Server.Device;
using Ace.Core.Util;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Ace.Core.Server.Event;
using CobraLibrary;
using Ace.Core.Client.Sim3d.Controls;
using System.Drawing.Text;
using HMI;

namespace OptiSort
{
    /// <summary>
    /// Collects picking coordinates calculated by the openCV library which elaborate the camera image (incoming over MQTT)
    /// Two buttons are used to connect/disconnect from the robot and open the manual commands
    /// </summary>
    public partial class ucScara : UserControl
    {
        public Cobra Cobra600 { get; set; }

        private HMI.ucOptiSort _frmMain;

        // ROBOT INSTANCE
        //private const int CallbackPort = 43431;
        //private const int RemotingPort = 43434;
        //private const string ProgramFile = "robot.v2";
        //private const int VPlusRobotTask = 1;
        private IAceClient _client;
        private IAceServer _server;
        private IAdeptController _controller;
        private IAdeptRobot _robot;
        //private RemoteAceObjectEventHandler generalEventHandler;
        //private RemoteApplicationEventHandler applicationEventHandler;
        //private SimulationContainerControl simulationControl;
        //private ControlPanelManager pendantManager;

        // LOCATIONS
        Thread _thDefineLocation;
        static Thread _thReachLocation;
        bool _stop = false;
        bool _msgReady = false;
        string _msgReceived = string.Empty;
        static bool _robotIsMoving = false;
        private Transform3D _lastTarget = new Transform3D(0, 0, 0, 0, 0, 0);
        private BindingList<Transform3D> _targetQueueList;


        public ucScara(ucOptiSort ucOptiSort)
        {
            InitializeComponent();

            _frmMain = ucOptiSort;

            // Initialize the remoting subsystem
            RemotingUtil.InitializeRemotingSubsystem(true, 0);
        }

        private void ucScara_Load(object sender, EventArgs e)
        {
            // init dgv
            _targetQueueList = new BindingList<Transform3D>();
            dgvTargetQueue.AutoGenerateColumns = false;
            DataGridViewTextBoxColumn xColumn = new DataGridViewTextBoxColumn
            { HeaderText = "DX", DataPropertyName = "DX", Width = 100 };
            DataGridViewTextBoxColumn yColumn = new DataGridViewTextBoxColumn
            { HeaderText = "DY", DataPropertyName = "DY", Width = 100 };
            DataGridViewTextBoxColumn zColumn = new DataGridViewTextBoxColumn
            { HeaderText = "DZ", DataPropertyName = "DZ", Width = 100 };
            DataGridViewTextBoxColumn yawColumn = new DataGridViewTextBoxColumn
            { HeaderText = "Yaw", DataPropertyName = "Yaw", Width = 100 };
            DataGridViewTextBoxColumn pitchColumn = new DataGridViewTextBoxColumn
            { HeaderText = "Pitch", DataPropertyName = "Pitch", Width = 100 };
            DataGridViewTextBoxColumn rollColumn = new DataGridViewTextBoxColumn
            { HeaderText = "Roll", DataPropertyName = "Roll", AutoSizeMode = DataGridViewAutoSizeColumnMode.Fill };

            dgvTargetQueue.Columns.Add(xColumn);
            dgvTargetQueue.Columns.Add(yColumn);
            dgvTargetQueue.Columns.Add(zColumn);
            dgvTargetQueue.Columns.Add(yawColumn);
            dgvTargetQueue.Columns.Add(pitchColumn);
            dgvTargetQueue.Columns.Add(rollColumn);
            dgvTargetQueue.DataSource = _targetQueueList;
            dgvTargetQueue.Rows.Clear();
        }



        // ---------------------------------------------------------------------------

        private void btnConnect_Click(object sender, EventArgs e)
        {
            _frmMain.Log("Connecting to Cobra600...");
            (_controller, _robot, _server, _client) = Cobra600.Connect(chkEmulate.Checked);
            
            if (_controller != null && _robot != null && _server != null)
            {
                _frmMain.Log("Connected to Cobra600");

                _stop = false;

                // Start a new thread to add the target locations to the queue
                _thDefineLocation = new Thread(AddToLocationQueue) { IsBackground = true };
                _thDefineLocation.Start();

                // Start a new thread to move the robot to the target locations
                _thReachLocation = new Thread(MoveToLoc) { IsBackground = true };
                _thReachLocation.Start();

                _frmMain.Log("Location threads started");
            }
        }

        private void btnDisconnect_Click(object sender, EventArgs e)
        {
            // TODO: add disconnection check
            _frmMain.Log("Disconnecting from Cobra600");
            Cobra600.Disconnect(_controller, _server);
        }

        // ---------------------------------------------------------------------------

        // TODO: add summary
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


        // TODO: add summary
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

                            // TODO: anzichè fare questo, evidenziare la row sulla dgv
                            //textBoxX.Text = _locTarget.DX.ToString();
                            //textBoxY.Text = _locTarget.DY.ToString();
                            //textBoxZ.Text = _locTarget.DZ.ToString();
                            //textBoxRoll.Text = _locTarget.Roll.ToString();
                            //textBoxPitch.Text = _locTarget.Pitch.ToString();
                            //textBoxYaw.Text = _locTarget.Yaw.ToString();

                            _frmMain.Log("Moving to target: " + _locTarget);


                            Cobra.Motion.Approach(_server, _robot, _locTarget, 20);
                            Cobra.Motion.CartesianMove(_server, _robot, _locTarget, true);
                            Cobra.Motion.Approach(_server, _robot, _locTarget, 20);

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

                            _frmMain.Log("Target reached");
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
