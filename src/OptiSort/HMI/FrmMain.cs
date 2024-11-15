using System.Linq;
using System.Windows.Forms;
using System.Threading.Tasks;
using OptiSort.userControls;
using System;
using System.ComponentModel;
using Ace.Core.Util;
using System.Drawing;
using System.Diagnostics;
using System.Collections.Generic;
using FlexibowlLibrary;
using Ace.UIBuilder.Client.Controls.Tools.WindowsForms;
using static System.Net.Mime.MediaTypeNames;
using Ace.Adept.Server.Motion;

namespace OptiSort
{
    public partial class frmMain : Form, INotifyPropertyChanged
    {

        // -----------------------------------------------------------------------------------
        // ---------------------------------- DECLARATIONS -----------------------------------
        // -----------------------------------------------------------------------------------

        #region declarations

        //NOTE: remember to reactivate exception settings > managed debugging assistant > loader lock for release to prod

        public MQTT MqttClient { get; set; }
        public Cobra600 Cobra600 { get; set; }
        public Flexibowl Flexibowl { get; set; }

        private class Cameras
        {
            public int ID { get; set; }
            public string Text { get; set; }
            public string mqttTopic { get; set; }
        }
        List<Cameras> _camerasList = new List<Cameras> { }; // TODO: review definition

        private ucCameraStream _ucCameraStream; // TODO: review definition


        // status
        public event PropertyChangedEventHandler PropertyChanged; // declare propertyChanged event (required by the associated interface)
        private bool _statusScara = false;
        private bool _statusScaraEmulation = true;
        private bool _statusFelxibowl = false;
        private bool _statusMqttClient = false;

        // custom defined properties to trigger events on status changed
        public bool StatusScara
        {
            get { return _statusScara; }
            set
            {
                if (_statusScara != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusScara = value;
                    OnPropertyChanged(nameof(StatusScara));
                }
            }
        }
        public bool StatusFlexibowl
        {
            get { return _statusFelxibowl; }
            set
            {
                if (_statusFelxibowl != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusFelxibowl = value;
                    OnPropertyChanged(nameof(StatusFlexibowl));
                }
            }
        }
        public bool StatusMqttClient
        {
            get { return _statusMqttClient; }
            set
            {
                if (_statusMqttClient != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusMqttClient = value;
                    OnPropertyChanged(nameof(StatusMqttClient));
                }
            }
        }
        public bool StatusScaraEmulation
        {
            get { return _statusScaraEmulation; }
            set
            {
                if (_statusScaraEmulation != value) // setting value different from actual value -> store new value and trigger event
                {
                    _statusScaraEmulation = value;
                    OnPropertyChanged(nameof(StatusScaraEmulation));
                }
            }
        }
        protected void OnPropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        #endregion

        // -----------------------------------------------------------------------------------
        // ---------------------------------- CONSTRUCTOR ------------------------------------
        // -----------------------------------------------------------------------------------

        public frmMain()
        {
            InitializeComponent();

            // Instance MQTT class
            string mqttBroker = Properties.Settings.Default.mqtt_broker;
            string mqttPort = Properties.Settings.Default.mqtt_port;
            MqttClient = new MQTT(mqttBroker, mqttPort);

            // Instance cobra class
            string serverIP = "localhost"; // because the initial status of the emulation mode is enabled
            string remotingPort = Properties.Settings.Default.scara_port;
            Cobra600 = new Cobra600("ace", serverIP, remotingPort);

            // Instance flexibowl class
            string flexibowlIP = Properties.Settings.Default.flexibowl_IP;
            string flexibowlPort = Properties.Settings.Default.flexibowl_port;
            Flexibowl = new Flexibowl(flexibowlIP, flexibowlPort);


            // Initialize the remoting subsystem
            RemotingUtil.InitializeRemotingSubsystem(true, 0);


            // Display default user control
            ucManualControl ucManualControl = new ucManualControl(this);
            ucManualControl.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Clear();
            pnlCurrentUc.Controls.Add(ucManualControl);

            // Activate default pushbuttons
            btnAuto.Enabled = true;
            btnManual.Enabled = false;
            btnConfig.Enabled = true;

            // init combobox
            _camerasList.Add(new Cameras { ID = 0, Text = "Basler", mqttTopic = Properties.Settings.Default.mqtt_topic_baslerStream });
            _camerasList.Add(new Cameras { ID = 1, Text = "IDS", mqttTopic = Properties.Settings.Default.mqtt_topic_idsStream });
            _camerasList.Add(new Cameras { ID = 2, Text = "Luxonics", mqttTopic = Properties.Settings.Default.mqtt_topic_luxonisStream });
            cmbCameras.DataSource = _camerasList;
            cmbCameras.DisplayMember = "Text";
            cmbCameras.ValueMember = "ID";

            // init camera view
            _ucCameraStream = new ucCameraStream(this);
            _ucCameraStream.StreamTopic = _camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex).mqttTopic;
            _ucCameraStream.Dock = DockStyle.Fill;
            pnlCameraStream.Controls.Clear();
            pnlCameraStream.Controls.Add(_ucCameraStream);

            // attach events
            this.PropertyChanged += RefreshStatusBar; // trigger statusbar buttons refresh
            MqttClient.MessageReceived += _ucCameraStream.OnMessageReceived; // enable MQTT messages to trigger the user control

            // Initialize robot panel view
            PaintIndicationRobot3DView();

            Log("OptiSort ready for operation: please connect systems (Scara robot, flexibowl, MQTT service) to begin");

        }

        // -----------------------------------------------------------------------------------
        // ---------------------------------- STATUS BAR -------------------------------------
        // -----------------------------------------------------------------------------------

        #region connection buttons (statusbar)

        private void RefreshStatusBar(object sender, PropertyChangedEventArgs e)
        {
            // Check which property changed and trigger corresponding logic

            if (e.PropertyName == nameof(StatusScara))
            {
                if (StatusScara == true)
                {
                    lblScaraStatusValue.Text = "Online";
                    lblScaraStatusValue.ForeColor = Color.Green;
                    btnScaraConnect.Enabled = false;
                    btnScaraDisconnect.Enabled = true;
                }
                else
                {
                    lblScaraStatusValue.Text = "Offline";
                    lblScaraStatusValue.ForeColor = SystemColors.Desktop;
                    btnScaraConnect.Enabled = true;
                    btnScaraDisconnect.Enabled = false;
                }
            }

            if (e.PropertyName == nameof(StatusFlexibowl))
            {
                if (StatusFlexibowl == true)
                {
                    lblFlexibowlStatusValue.Text = "Online";
                    lblFlexibowlStatusValue.ForeColor = Color.Green;
                    btnFlexibowlConnect.Enabled = false;
                    btnFlexibowlDisconnect.Enabled = true;
                }
                else
                {
                    lblFlexibowlStatusValue.Text = "Offline";
                    lblFlexibowlStatusValue.ForeColor = SystemColors.Desktop;
                    btnFlexibowlConnect.Enabled = true;
                    btnFlexibowlDisconnect.Enabled = false;
                }
            }

            if (e.PropertyName == nameof(StatusMqttClient))
            {
                if (StatusMqttClient == true)
                {
                    lblMqttStatusValue.Text = "Online";
                    lblMqttStatusValue.ForeColor = Color.Green;
                    btnMqttConnect.Enabled = false;
                    btnMqttDisconnect.Enabled = true;
                }
                else
                {
                    lblMqttStatusValue.Text = "Offline";
                    lblMqttStatusValue.ForeColor = SystemColors.Desktop;
                    btnMqttConnect.Enabled = true;
                    btnMqttDisconnect.Enabled = false;
                }
            }
        }

        private void btnScaraConnect_Click(object sender, EventArgs e)
        {
            // check if ACE is running
            Process[] ProcessList = Process.GetProcessesByName("Ace");
            if (ProcessList.Length != 1)
            {
                MessageBox.Show("ACE is not running: please open the robot server");
                return;
            }
            Cursor = Cursors.WaitCursor;
            Log("Trying to connect to robot");
            ConnectScara();
        }

        private void btnScaraDisconnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            Log("Trying to disconnect from robot");
            DisconnectScara();
        }

        private void btnFlexibowlConnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            Log("Trying to connect to flexibowl");
            ConnectFlexibowl();
        }

        private void btnFlexibowlDisconnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            Log("Trying to disconnect to flexibowl");
            DisconnectFlexibowl();
        }

        private void btnMqttConnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            Log("Trying to create new MQTT client");
            ConnectMQTTClient();
        }

        private void btnMqttDisconnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            Log("Trying to destroy MQTT client");
            DisconnectMqttClient(Properties.Settings.Default.mqtt_client);
        }

        private void btnEmulateScara_Click(object sender, EventArgs e)
        {
            if (StatusScaraEmulation)
            {
                btnEmulateScara.BackgroundImage = Properties.Resources.robot_2x2_pptx;
                StatusScaraEmulation = false;
                Log("Scara emulation mode disabled");
            }
            else
            {
                btnEmulateScara.BackgroundImage = Properties.Resources.emulation_2x2_pptx;
                StatusScaraEmulation = true;
                Log("Scara emulation mode enabled");
            }
        }

        #endregion

        // -----------------------------------------------------------------------------------
        // -------------------------------- FORM NAVIGATION ----------------------------------
        // -----------------------------------------------------------------------------------

        #region navigation buttons (form bottom)

        private void CleanPnlCurrentUc()
        {
            Control previousControl = pnlCurrentUc.Controls.Cast<Control>().FirstOrDefault(c => c.Dock == DockStyle.Fill); // get control docked in pnlCurrentUc
            pnlCurrentUc.Controls.Remove(previousControl);
            pnlCurrentUc.Controls.Clear();

            previousControl.Dispose();
        }

        private void btnAuto_Click(object sender, System.EventArgs e)
        {

            if (!StatusScara || !StatusMqttClient || !StatusFlexibowl)
            {
                MessageBox.Show("You should connect all the entities (Scara robot, Flexibowl, MQTT Client) to start the automatic process");
                return;
            }

            CleanPnlCurrentUc();

            ucProcessView ucProcessView = new ucProcessView(this);
            ucProcessView.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Add(ucProcessView);

            btnAuto.Enabled = false;
            btnAuto.BackgroundImage = Properties.Resources.autoEnabled_2x2_pptx;
            btnManual.Enabled = true;
            btnManual.BackgroundImage = Properties.Resources.manualDisabled_2x2_pptx;
            btnConfig.Enabled = true;
            btnConfig.BackgroundImage = Properties.Resources.configDisabled_2x2_pptx;

            btnRun.Enabled = true;
            btnRun.BackgroundImage = Properties.Resources.playPassivated_2x2_pptx;
            btnStop.Enabled = false;
            btnStop.BackgroundImage = Properties.Resources.stopEnabled_2x2_pptx;
        }

        private void btnRun_Click(object sender, EventArgs e)
        {
            btnRun.Enabled = false;
            btnRun.BackgroundImage = Properties.Resources.playEnabled_2x2_pptx;
            btnStop.Enabled = true;
            btnStop.BackgroundImage = Properties.Resources.stopPassivated_2x2_pptx;
        }

        private void btnStop_Click(object sender, EventArgs e)
        {
            btnRun.Enabled = true;
            btnRun.BackgroundImage = Properties.Resources.playPassivated_2x2_pptx;
            btnStop.Enabled = false;
            btnStop.BackgroundImage = Properties.Resources.stopEnabled_2x2_pptx;
        }

        private void btnManual_Click(object sender, System.EventArgs e)
        {
            CleanPnlCurrentUc();

            ucManualControl ucManualControl = new ucManualControl(this);
            ucManualControl.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Add(ucManualControl);

            btnAuto.Enabled = true;
            btnAuto.BackgroundImage = Properties.Resources.autoDisabled_2x2_pptx;
            btnManual.Enabled = false;
            btnManual.BackgroundImage = Properties.Resources.manualEnabled_2x2_pptx;
            btnConfig.Enabled = true;
            btnConfig.BackgroundImage = Properties.Resources.configDisabled_2x2_pptx;

            btnRun.Enabled = false;
            btnRun.BackgroundImage = Properties.Resources.playDisabled_2x2_pptx;
            btnStop.Enabled = false;
            btnStop.BackgroundImage = Properties.Resources.stopDisabled_2x2_pptx;
        }

        private void btnConfig_Click(object sender, System.EventArgs e)
        {
            CleanPnlCurrentUc();

            ucConfiguration ucConfiguration = new ucConfiguration(this);
            ucConfiguration.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Add(ucConfiguration);

            btnAuto.Enabled = true;
            btnAuto.BackgroundImage = Properties.Resources.autoDisabled_2x2_pptx;
            btnManual.Enabled = true;
            btnManual.BackgroundImage = Properties.Resources.manualDisabled_2x2_pptx;
            btnConfig.Enabled = false;
            btnConfig.BackgroundImage = Properties.Resources.configEnabled_2x2_pptx;

            btnRun.Enabled = false;
            btnRun.BackgroundImage = Properties.Resources.playDisabled_2x2_pptx;
            btnStop.Enabled = false;
            btnStop.BackgroundImage = Properties.Resources.stopDisabled_2x2_pptx;
        }

        #endregion

        // -----------------------------------------------------------------------------------
        // ------------------------------------ SCARA ----------------------------------------
        // -----------------------------------------------------------------------------------

        #region ACE scara

        // TODO: is possible to use async?

        private void ConnectScara()
        {
            string controllerName = Properties.Settings.Default.scara_controllerName;
            string robotName = Properties.Settings.Default.scara_robotName;
            string endEffectorName = Properties.Settings.Default.scara_endEffectorName;

            if (!StatusScaraEmulation)
                Cobra600.ServerIP = Properties.Settings.Default.scara_serverIP;

            if (Cobra600.Connect(StatusScaraEmulation, controllerName, robotName, endEffectorName))
            {
                Log("Robot successfully connected");
                btnScaraConnect.Enabled = false;
                btnScaraConnect.BackgroundImage = Properties.Resources.connectedDisabled_2x2_pptx;
                btnScaraDisconnect.Enabled = true;
                btnScaraDisconnect.BackgroundImage = Properties.Resources.disconnectedEnabled_2x2_pptx;
                btnEmulateScara.Enabled = false;
                StatusScara = true;
                pnlRobotView.Controls.Clear();
                pnlRobotView.Controls.Add(Cobra600.SimulationContainerControl); // add robot rendering to panel
                pnlRobotView.Refresh();
                Cursor = Cursors.Default;
            }
            else
            {
                Log("Failed to connect to robot");
                Cursor = Cursors.Default;
            }
        }

        private void DisconnectScara()
        {
            pnlRobotView.Controls.Remove(Cobra600.SimulationContainerControl);
            pnlRobotView.Controls.Clear();

            if (Cobra600.Disconnect())
            {
                Log("Robot succesfully disconnected");
                btnScaraConnect.Enabled = true;
                btnScaraConnect.BackgroundImage = Properties.Resources.connectedEnabled_2x2_pptx;
                btnScaraDisconnect.Enabled = false;
                btnScaraDisconnect.BackgroundImage = Properties.Resources.disconnectedDisabled_2x2_pptx;
                btnEmulateScara.Enabled = true;
                StatusScara = false;
                Cursor = Cursors.Default;
                PaintIndicationRobot3DView();
            }
            else
            {
                Log("Failed to disconnect from Cobra");
                Cursor = Cursors.Default;
            }
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

        #endregion

        // -----------------------------------------------------------------------------------
        // ---------------------------------- FLEXIBOWL --------------------------------------
        // -----------------------------------------------------------------------------------

        #region flexibowl

        private void ConnectFlexibowl()
        {
            Flexibowl.Connect();
            try
            {
                Flexibowl.Set.Servo(true);
                btnFlexibowlConnect.Enabled = false;
                btnFlexibowlConnect.BackgroundImage = Properties.Resources.connectedDisabled_2x2_pptx;
                btnFlexibowlDisconnect.Enabled = true;
                btnFlexibowlDisconnect.BackgroundImage = Properties.Resources.disconnectedEnabled_2x2_pptx;
                StatusFlexibowl = true;
                Log("Flexibowl connected and servo ON");
            }
            catch (Exception ex)
            {
                Log($"Failed to connect Flexibowl UDP client");
                MessageBox.Show($"{ex}");
            }
            Cursor = Cursors.Default;
        }

        private void DisconnectFlexibowl()
        {
            if (Flexibowl.Disconnect())
            {
                btnFlexibowlConnect.Enabled = true;
                btnFlexibowlConnect.BackgroundImage = Properties.Resources.connectedEnabled_2x2_pptx;
                btnFlexibowlDisconnect.Enabled = false;
                btnFlexibowlDisconnect.BackgroundImage = Properties.Resources.disconnectedDisabled_2x2_pptx;
                StatusFlexibowl = false;
                Cursor = Cursors.Default;
                Log("Flexibowl UDP client disconnected");
            }
            else
            {
                Cursor = Cursors.Default;
                Log("Failed to disconnect from Flexibowl UDP Client");
            }
        }

        #endregion

        // -----------------------------------------------------------------------------------
        // ------------------------------------ MQTT -----------------------------------------
        // -----------------------------------------------------------------------------------

        #region MQTT

        private async void ConnectMQTTClient()
        {
            string mqttClientName = Properties.Settings.Default.mqtt_client;
            Task<bool> createClient = MqttClient.CreateClient(mqttClientName);

            if (await createClient)
            {
                StatusMqttClient = true;
                btnMqttConnect.Enabled = false;
                btnMqttConnect.BackgroundImage = Properties.Resources.connectedDisabled_2x2_pptx;
                btnMqttDisconnect.Enabled = true;
                btnMqttDisconnect.BackgroundImage = Properties.Resources.disconnectedEnabled_2x2_pptx;
                Log($"MQTT client '{mqttClientName}' created");
                SubscribeMqttTopics();
            }
            else
            {
                Log($"Failed to create '{mqttClientName}' MQTT client");
                Cursor = Cursors.Default;
            }
        }

        public async void SubscribeMqttTopics()
        {

            // Retreive topics from configuration file
            string mqttClientName = Properties.Settings.Default.mqtt_client;
            string topicScaraTarget = Properties.Settings.Default.mqtt_topic_scaraTarget;
            string topicIdsStream = Properties.Settings.Default.mqtt_topic_idsStream;
            string topicLuxonisStream = Properties.Settings.Default.mqtt_topic_luxonisStream;
            string topicBaslerStream = Properties.Settings.Default.mqtt_topic_baslerStream;


            // Subscribe to the necessary topics
            bool scaraSubscriberd = await MqttClient.SubscribeClientToTopic(mqttClientName, topicScaraTarget);
            bool idsSubscribed = await MqttClient.SubscribeClientToTopic(mqttClientName, topicIdsStream);
            bool baslerSubscribed = await MqttClient.SubscribeClientToTopic(mqttClientName, topicBaslerStream);
            bool luxonisSubscribed = await MqttClient.SubscribeClientToTopic(mqttClientName, topicLuxonisStream);

            // logging
            if (scaraSubscriberd) Log($"{mqttClientName} subscribed to {topicScaraTarget}");
            else Log($"Unable subscribing {mqttClientName} to {topicScaraTarget}");

            if (idsSubscribed) Log($"{mqttClientName} subscribed to {topicIdsStream}");
            else Log($"Unable subscribing {mqttClientName} to {topicIdsStream}");

            if (baslerSubscribed) Log($"{mqttClientName} subscribed to {topicBaslerStream}");
            else Log($"Unable subscribing {mqttClientName} to {topicBaslerStream}");

            if (luxonisSubscribed) Log($"{mqttClientName} subscribed to {topicLuxonisStream}");
            else Log($"Unable subscribing {mqttClientName} to {topicLuxonisStream}");

            Cursor = Cursors.Default;
        }

        private async void DisconnectMqttClient(string clientName)
        {
            Task<bool> destroyClient = MqttClient.DestroyClient(clientName);
            if (await destroyClient)
            {
                StatusMqttClient = false;
                btnMqttConnect.Enabled = true;
                btnMqttConnect.BackgroundImage = Properties.Resources.connectedEnabled_2x2_pptx;
                btnMqttDisconnect.Enabled = false;
                btnMqttDisconnect.BackgroundImage = Properties.Resources.disconnectedDisabled_2x2_pptx;
                Log($"MQTT client '{clientName}' destroyed");
                Cursor = Cursors.Default;
            }
            else
            {
                Log($"Error destroying '{clientName}'");
                Cursor = Cursors.Default;
            }
        }

        #endregion

        // -----------------------------------------------------------------------------------
        // ------------------------------- CAMERA STREAM -------------------------------------
        // -----------------------------------------------------------------------------------

        private void cmbCameras_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (_ucCameraStream != null)
            {
                var newTopic = _camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex).mqttTopic;
                _ucCameraStream.StreamTopic = newTopic;
                Log($"Streaming topic updated to '{newTopic}'");
            }
        }

        // -----------------------------------------------------------------------------------
        // ----------------------------------- OTHERS ----------------------------------------
        // -----------------------------------------------------------------------------------

        public void Log(string msg)
        {
            msg = DateTime.Now.ToString() + " - " + msg;
            lstLog.Items.Add(msg);
            lstLog.TopIndex = lstLog.Items.Count - 1; // showing last row
        }
    }
}
