using System.Linq;
using System.Windows.Forms;
using System.Threading.Tasks;
using OptiSort.userControls;
using System;
using System.ComponentModel;
using Ace.Core.Util;
using System.Drawing;
using System.Diagnostics;

namespace OptiSort
{
    public partial class frmMain : Form, INotifyPropertyChanged
    {

        public MQTT MqttClient { get; set; }
        public Cobra600 Cobra600 { get; set; }

        // status
        public event PropertyChangedEventHandler PropertyChanged; // declare propertyChanged event (required by the associated interface)
        private bool _statusScara = false;
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
        protected void OnPropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }


        // TODO : change and restrict ucProcessview to contain only coordinate table (rename ucscaratarget) and flexibowl commands??
        // > change buttons from "auto, manual, config" into "process, config"
        // > add two buttons below the 3D view called "auto, manual", to make left side switch with manual commands
        // > when switching to manual process display everyting grayed out if automatic process is inprogress. Maybe some play and stop buttons may be helpful
       

        public frmMain()
        {
            InitializeComponent();

            // Initialize MQTT connection
            MqttClient = new MQTT();

            // Initialize cobra class
            Cobra600 = new Cobra600("ace", "localhost", 43434);

            // Initialize the remoting subsystem
            RemotingUtil.InitializeRemotingSubsystem(true, 0);

            // Subscribe to PropertyChanged event
            this.PropertyChanged += RefreshStatusBar;

            // Display default user control
            ucProcessView ucProcessView = new ucProcessView(this);
            ucProcessView.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Clear();
            pnlCurrentUc.Controls.Add(ucProcessView);

            // Activate default pushbuttons
            btnProcess.Enabled = false;
            btnManual.Enabled = true;
            btnConfig.Enabled = true;
        }


        // -----------------------------------------------------------------------------------
        // ------------------------------------- CONNECTIONS ---------------------------------
        // -----------------------------------------------------------------------------------

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

        }

        private void btnFlexibowlDisconnect_Click(object sender, EventArgs e)
        {

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
            DisconnectMqttClient(Properties.Settings.Default.mqttClientName);
        }


        // -----------------------------------------------------------------------------------
        // -------------------------------------- SCARA --------------------------------------
        // -----------------------------------------------------------------------------------

        // TODO: is possible to do these async?

        private void ConnectScara()
        {
            string controllerName = Properties.Settings.Default.scara_controllerName;
            string robotName = Properties.Settings.Default.scara_robotName;
            string endEffectorName = Properties.Settings.Default.scara_endEffectorName;

            if (Cobra600.Connect(true, controllerName, robotName, endEffectorName))
            {
                Log("Robot successfully connected");
                btnScaraConnect.Enabled = false;
                btnScaraDisconnect.Enabled = true;
                StatusScara = true;
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
            if (Cobra600.Disconnect())
            {
                Log("Robot succesfully disconnected");
                btnScaraConnect.Enabled = true;
                btnScaraDisconnect.Enabled = false;
                StatusScara = false;
                Cursor = Cursors.Default;
            }
            else
            {
                Log("Failed to disconnect from Cobra");
                Cursor = Cursors.Default;
            }
        }

        // -----------------------------------------------------------------------------------
        // -------------------------------------- MQTT ---------------------------------------
        // -----------------------------------------------------------------------------------

        private async void ConnectMQTTClient()
        {
            string mqttClientName = Properties.Settings.Default.mqttClientName;
            Task<bool> createClient = MqttClient.CreateClient(mqttClientName);

            if (await createClient)
            {
                StatusMqttClient = true;
                Log($"MQTT client '{mqttClientName}' created");
                SubscribeMqttTopics();
            }
            else
            {
                Log($"Failed to create '{mqttClientName}' MQTT client");
                Cursor = Cursors.Default;
            }
        }


        private async void SubscribeMqttTopics()
        {

            // Retreive topics from configuration file
            string mqttClientName = Properties.Settings.Default.mqttClientName;
            string topicScaraTarget = Properties.Settings.Default.mqttTopic_scaraTarget;
            string topicIdsStream = Properties.Settings.Default.mqttTopic_idsStream;
            string topicLuxonisStream = Properties.Settings.Default.mqttTopic_luxonisStream;
            string topicBaslerStream = Properties.Settings.Default.mqttTopic_baslerStream;


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
                MqttClient = null;
                Log($"MQTT client '{clientName}' destroyed");
                Cursor = Cursors.Default;
            }
            else
            {
                Log($"Error destroying '{clientName}'");
                Cursor = Cursors.Default;
            }
        }




        // -----------------------------------------------------------------------------------
        // ---------------------------- CHANGING USER CONTROL --------------------------------
        // -----------------------------------------------------------------------------------

        private void CleanPnlCurrentUc()
        {
            Control previousControl = pnlCurrentUc.Controls.Cast<Control>().FirstOrDefault(c => c.Dock == DockStyle.Fill); // get control docked in pnlCurrentUc
            pnlCurrentUc.Controls.Remove(previousControl);
            pnlCurrentUc.Controls.Clear();

            previousControl.Dispose();
        }


        private void btnProcess_Click(object sender, System.EventArgs e)
        {
            CleanPnlCurrentUc();

            ucProcessView ucProcessView = new ucProcessView(this);
            ucProcessView.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Add(ucProcessView);

            btnProcess.Enabled = false;
            btnManual.Enabled = true;
            btnConfig.Enabled = true;
        }

        private void btnManual_Click(object sender, System.EventArgs e)
        {
            CleanPnlCurrentUc();

            ucManualControl ucManualControl = new ucManualControl();
            ucManualControl.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Add(ucManualControl);

            btnProcess.Enabled = true;
            btnManual.Enabled = false;
            btnConfig.Enabled = true;
        }

        private void btnConfig_Click(object sender, System.EventArgs e)
        {
            CleanPnlCurrentUc();

            ucConfiguration ucConfiguration = new ucConfiguration();
            ucConfiguration.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Add(ucConfiguration);

            btnProcess.Enabled = true;
            btnManual.Enabled = true;
            btnConfig.Enabled = false;
        }


        // -----------------------------------------------------------------------------------
        // ---------------------------------------- LOG --------------------------------------
        // -----------------------------------------------------------------------------------

        public void Log(string msg)
        {
            msg = DateTime.Now.ToString() + " - " + msg;
            lstLog.Items.Add(msg);
            lstLog.TopIndex = lstLog.Items.Count - 1; // showing last row
        }


    }
}
