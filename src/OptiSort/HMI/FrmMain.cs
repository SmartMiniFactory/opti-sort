﻿using System.Linq;
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
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Header;
using System.Reflection.Emit;

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

        public ucCameraStream _ucCameraStream; // TODO: review definition

        public int CameraIndex { get; set; }

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
            string remotingPort = Properties.Settings.Default.scara_port;
            Cobra600 = new Cobra600("ace", remotingPort);

            // Instance flexibowl class
            string flexibowlIP = Properties.Settings.Default.flexibowl_IP;
            Flexibowl = new Flexibowl(flexibowlIP);


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
            lstLog.DrawItem += LstLog_DrawItem;


            // log box setup
            lstLog.ItemHeight = 15; // adjusting interline between log rows

            // Initialize robot panel view
            PaintIndicationRobot3DView();

            Log("OptiSort ready for operation: please connect systems (Scara robot, flexibowl, MQTT service) to begin", false, false);

        }


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

        public void AddNewUc(Control control)
        {
            CleanPnlCurrentUc();
            control.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Add(control);
        }

        private void btnAuto_Click(object sender, System.EventArgs e)
        {

            //if (!StatusScara || !StatusMqttClient || !StatusFlexibowl)
            //{
            //    MessageBox.Show("You should connect all the entities (Scara robot, Flexibowl, MQTT Client) to start the automatic process");
            //    return;
            //}

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

            if (!ucProcessView.AutomaticProcess)
            {
                MessageBox.Show("Click the start button to activate automatic process");
            }

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
                    lblScaraStatusValue.ForeColor = Color.Red;
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
                    lblFlexibowlStatusValue.ForeColor = Color.Red;
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
                    lblMqttStatusValue.ForeColor = Color.Red;
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
            Log("Trying to connect to robot", false, false);
            ConnectScara();
        }

        private void btnScaraDisconnect_Click(object sender, EventArgs e)
        {
            Log("Trying to disconnect from robot", false, false);
            DisconnectScara();
        }

        private void btnFlexibowlConnect_Click(object sender, EventArgs e)
        {
            Log("Trying to connect to flexibowl", false, false);
            ConnectFlexibowl();
        }

        private void btnFlexibowlDisconnect_Click(object sender, EventArgs e)
        {
            Log("Trying to disconnect to flexibowl", false, false);
            DisconnectFlexibowl();
        }

        private void btnMqttConnect_Click(object sender, EventArgs e)
        {
            Log("Trying to create new MQTT client", false, false);
            ConnectMQTTClient();
        }

        private void btnMqttDisconnect_Click(object sender, EventArgs e)
        {
            Log("Trying to destroy MQTT client", false, false);
            _ = DisconnectMqttClient(Properties.Settings.Default.mqtt_client);
        }

        private void btnEmulateScara_Click(object sender, EventArgs e)
        {
            if (StatusScaraEmulation)
            {
                btnEmulateScara.BackgroundImage = Properties.Resources.robotEnabled_2x2_pptx;
                StatusScaraEmulation = false;
                Log("Scara emulation mode disabled", false, false);
            }
            else
            {
                btnEmulateScara.BackgroundImage = Properties.Resources.emulationEnabled_2x2_pptx;
                StatusScaraEmulation = true;
                Log("Scara emulation mode enabled", false, false);
            }
        }

        #endregion

        // -----------------------------------------------------------------------------------
        // ---------------------------------- CONNECTIONS ------------------------------------
        // -----------------------------------------------------------------------------------

        #region ACE scara

        // TODO: is possible to use async?

        public void ConnectScara()
        {
            Cursor = Cursors.WaitCursor;

            string controllerName = Properties.Settings.Default.scara_controllerName;
            string robotName = Properties.Settings.Default.scara_robotName;
            string endEffectorName = Properties.Settings.Default.scara_endEffectorName;


            if (!StatusScaraEmulation)
                Cobra600.RobotIP = Properties.Settings.Default.scara_controllerIP;

            var ex = Cobra600.Connect(StatusScaraEmulation, controllerName, robotName, endEffectorName);
            if (ex == null)
            {
                Log("Robot successfully connected", false, true);
                btnScaraConnect.Enabled = false;
                btnScaraConnect.BackgroundImage = Properties.Resources.connectedDisabled_2x2_pptx;
                btnScaraDisconnect.Enabled = true;
                btnScaraDisconnect.BackgroundImage = Properties.Resources.disconnectedEnabled_2x2_pptx;
                btnEmulateScara.Enabled = false;
                if (StatusScaraEmulation)
                    btnEmulateScara.BackgroundImage = Properties.Resources.emulationDisabled_2x2_pptx;
                else
                    btnEmulateScara.BackgroundImage = Properties.Resources.robotDisabled_2x2_pptx;
                StatusScara = true;
                pnlRobotView.Controls.Clear();
                pnlRobotView.Controls.Add(Cobra600.SimulationContainerControl); // add robot rendering to panel
                pnlRobotView.Refresh();

                MessageBox.Show("Scara connected, enabling power. Please press the physical button on the front panel!");

            }
            else
            {
                MessageBox.Show($"{ex}");
                Log("Failed to connect to robot", true, false);
            }

            Cursor = Cursors.Default;
        }

        public void DisconnectScara()
        {
            Cursor = Cursors.WaitCursor;

            pnlRobotView.Controls.Remove(Cobra600.SimulationContainerControl);
            pnlRobotView.Controls.Clear();

            if (Cobra600.Disconnect())
            {
                Log("Robot succesfully disconnected", false, true);
                btnScaraConnect.Enabled = true;
                btnScaraConnect.BackgroundImage = Properties.Resources.connectedEnabled_2x2_pptx;
                btnScaraDisconnect.Enabled = false;
                btnScaraDisconnect.BackgroundImage = Properties.Resources.disconnectedDisabled_2x2_pptx;
                btnEmulateScara.Enabled = true;
                if (StatusScaraEmulation)
                    btnEmulateScara.BackgroundImage = Properties.Resources.emulationEnabled_2x2_pptx;
                else
                    btnEmulateScara.BackgroundImage = Properties.Resources.robotEnabled_2x2_pptx;
                StatusScara = false;
                PaintIndicationRobot3DView();
            }
            else
            {
                Log("Failed to disconnect from Cobra", true, false);
            }

            Cursor = Cursors.Default;
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

        #region flexibowl

        public void ConnectFlexibowl()
        {
            Cursor = Cursors.WaitCursor;
            Flexibowl.Connect();
            try
            {
                Flexibowl.Set.Servo(true);
                btnFlexibowlConnect.Enabled = false;
                btnFlexibowlConnect.BackgroundImage = Properties.Resources.connectedDisabled_2x2_pptx;
                btnFlexibowlDisconnect.Enabled = true;
                btnFlexibowlDisconnect.BackgroundImage = Properties.Resources.disconnectedEnabled_2x2_pptx;
                StatusFlexibowl = true;
                Log("Flexibowl connected and servo ON", false, true);
            }
            catch (Exception ex)
            {
                Log($"Failed to connect Flexibowl UDP client", true, false);
                MessageBox.Show($"{ex}");
            }
            Cursor = Cursors.Default;
        }

        public void DisconnectFlexibowl()
        {
            Cursor = Cursors.WaitCursor;
            if (Flexibowl.Disconnect())
            {
                btnFlexibowlConnect.Enabled = true;
                btnFlexibowlConnect.BackgroundImage = Properties.Resources.connectedEnabled_2x2_pptx;
                btnFlexibowlDisconnect.Enabled = false;
                btnFlexibowlDisconnect.BackgroundImage = Properties.Resources.disconnectedDisabled_2x2_pptx;
                StatusFlexibowl = false;
                Log("Flexibowl UDP client disconnected", false, true);
            }
            else
            {
                Log("Failed to disconnect from Flexibowl UDP Client", true, false);
            }
            Cursor = Cursors.Default;
        }

        #endregion

        #region MQTT

        public async void ConnectMQTTClient()
        {
            Cursor = Cursors.WaitCursor;

            if (StatusMqttClient)
            {
                Log("MQTT client already connected, cannot connect another", true, false);
                Cursor = Cursors.Default;
                return;
            }

            string mqttClientName = Properties.Settings.Default.mqtt_client;
            Task<bool> createClient = MqttClient.CreateClient(mqttClientName);

            if (await createClient)
            {
                StatusMqttClient = true;
                btnMqttConnect.Enabled = false;
                btnMqttConnect.BackgroundImage = Properties.Resources.connectedDisabled_2x2_pptx;
                btnMqttDisconnect.Enabled = true;
                btnMqttDisconnect.BackgroundImage = Properties.Resources.disconnectedEnabled_2x2_pptx;
                Log($"MQTT client '{mqttClientName}' created", false, true);

                List<string> topics = new List<string>
                {
                    // Retreive topics from configuration file
                    Properties.Settings.Default.mqtt_topic_scaraTarget,
                    Properties.Settings.Default.mqtt_topic_idsStream,
                    Properties.Settings.Default.mqtt_topic_luxonisStream,
                    Properties.Settings.Default.mqtt_topic_baslerStream
                };

                foreach (string topic in topics)
                {
                    SubscribeMqttTopic(mqttClientName, topic);
                }

            }
            else
            {
                Log($"Failed to create '{mqttClientName}' MQTT client", true, false);
            }
            Cursor = Cursors.Default;
        }

        public async Task<bool> DisconnectMqttClient(string clientName)
        {
            Cursor = Cursors.WaitCursor;
            Task<bool> destroyClient = MqttClient.DestroyClient(clientName);
            if (await destroyClient)
            {
                StatusMqttClient = false;
                btnMqttConnect.Enabled = true;
                btnMqttConnect.BackgroundImage = Properties.Resources.connectedEnabled_2x2_pptx;
                btnMqttDisconnect.Enabled = false;
                btnMqttDisconnect.BackgroundImage = Properties.Resources.disconnectedDisabled_2x2_pptx;
                Cursor = Cursors.Default;
                Log($"MQTT client '{clientName}' destroyed", false, true);
                return true;
            }
            else
            {
                Cursor = Cursors.Default;
                Log($"Error destroying '{clientName}'", true, false);
                return false;
            }
        }

        public async void SubscribeMqttTopic(string client, string topic)
        {
            bool subscribed = await MqttClient.SubscribeClientToTopic(client, topic);
            if (subscribed)
                Log($"{client} subscribed to {topic}", false, true);
            else
                Log($"Unable subscribing {client} to {topic}", true, false);
        }

        public async void UnsubscribeMqttTopic(string client, string topic)
        {
            bool unsubscribed = await MqttClient.UnsubscribeClientFromTopic(client, topic);
            if (unsubscribed)
                Log($"{client} unsubscribed to {topic}", false, true);
            else
                Log($"Unable unsubscribing {client} to {topic}", true, false);
        }

        #endregion

        // -----------------------------------------------------------------------------------
        // ------------------------------- CAMERA STREAM -------------------------------------
        // -----------------------------------------------------------------------------------

        public void ForceStreamingDropDownList(int cameraID)
        {
            cmbCameras.Invoke(new Action(() => cmbCameras.SelectedIndex = cameraID));
            cmbCameras.Invalidate();
        }

        private void cmbCameras_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (_ucCameraStream != null)
            {
                CameraIndex = cmbCameras.SelectedIndex;

                var newTopic = _camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex).mqttTopic;
                _ucCameraStream.StreamTopic = newTopic;
                Log($"Streaming topic updated to '{newTopic}'", false, false);
            }
        }

        // -----------------------------------------------------------------------------------
        // ----------------------------------- OTHERS ----------------------------------------
        // -----------------------------------------------------------------------------------

        private class LogEntry
        {
            public string Message { get; set; }
            public bool IsError { get; set; }
            public bool IsSuccess { get; set; }
            public override string ToString() => Message; // Fallback for ListBox's default behavior
        }

        public void Log(string msg, bool isError, bool isSuccess)
        {
            msg = DateTime.Now.ToString() + " - " + msg; // Prepend timestamp to the message
            lstLog.Items.Add(new LogEntry { Message = msg, IsError = isError, IsSuccess = isSuccess }); // Store the error flag alongside the message
            lstLog.TopIndex = lstLog.Items.Count - 1; // Ensure the last row is visible
        }

        private void LstLog_DrawItem(object sender, DrawItemEventArgs e)
        {
            if (e.Index < 0) return;

            // Retrieve the LogEntry object
            var logEntry = lstLog.Items[e.Index] as LogEntry;
            if (logEntry == null) return;

            // Set the text color based on IsError or isSuccess
            Brush textBrush = logEntry.IsError ? Brushes.Red : logEntry.IsSuccess ? Brushes.Green : Brushes.Black;

            // Draw the background and text
            e.DrawBackground();
            e.Graphics.DrawString(logEntry.Message, e.Font, textBrush, e.Bounds);
            e.DrawFocusRectangle();
        }

    }
}
