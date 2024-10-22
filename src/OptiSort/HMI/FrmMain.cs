using System.Linq;
using System.Windows.Forms;
using System.Threading.Tasks;
using OptiSort.userControls;
using System.IO;
using System.Text.Json;
using System;
using Ace.HSVision.Client.Wizard.Calibrations.Sequencers;
using System.ComponentModel;

namespace OptiSort
{
    public partial class frmMain : Form, INotifyPropertyChanged
    {

        public MQTT _mqttClient;

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


        public frmMain()
        {
            InitializeComponent();

            // Initialize MQTT connection
            _mqttClient = new MQTT();

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
                    this.lblScaraStatusValue.Text = "Online";
                    this.btnScaraConnect.Enabled = false;
                    this.btnScaraDisconnect.Enabled = true;
                }
                else
                {
                    this.lblScaraStatusValue.Text = "Offline";
                    this.btnScaraConnect.Enabled = true;
                    this.btnScaraDisconnect.Enabled = false;
                }
            }

            if (e.PropertyName == nameof(StatusFlexibowl))
            {
                if (StatusFlexibowl == true)
                {
                    this.lblFlexibowlStatusValue.Text = "Online";
                    this.btnFlexibowlConnect.Enabled = false;
                    this.btnFlexibowlDisconnect.Enabled = true;
                }
                else
                {
                    this.lblFlexibowlStatusValue.Text = "Offline";
                    this.btnFlexibowlConnect.Enabled = true;
                    this.btnFlexibowlDisconnect.Enabled = false;
                }
            }

            if (e.PropertyName == nameof(StatusMqttClient))
            {
                if (StatusMqttClient == true)
                {
                    this.lblMqttStatusValue.Text = "Online";
                    this.btnMqttConnect.Enabled = false;
                    this.btnMqttDisconnect.Enabled = true;
                }
                else
                {
                    this.lblMqttStatusValue.Text = "Offline";
                    this.btnMqttConnect.Enabled = true;
                    this.btnMqttDisconnect.Enabled = false;
                }
            }
        }

        private void btnScaraConnect_Click(object sender, EventArgs e)
        {

        }

        private void btnScaraDisconnect_Click(object sender, EventArgs e)
        {

        }

        private void btnFlexibowlConnect_Click(object sender, EventArgs e)
        {

        }

        private void btnFlexibowlDisconnect_Click(object sender, EventArgs e)
        {

        }

        private void btnMqttConnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor; // mouse cursor to loading wheel
            ConnectMQTTClient();
        }

        private void btnMqttDisconnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor; // mouse cursor to loading wheel
            DisconnectMqttClient(Properties.Settings.Default.mqttClientName);
        }



        // -----------------------------------------------------------------------------------
        // -------------------------------------- MQTT ---------------------------------------
        // -----------------------------------------------------------------------------------

        private async void ConnectMQTTClient()
        {
            string mqttClientName = Properties.Settings.Default.mqttClientName;
            Task<bool> createClient = _mqttClient.CreateClient(mqttClientName);

            if (await createClient)
            {
                StatusMqttClient = true;
                Log($"MQTT client '{mqttClientName}' created");
                subscribeMqttTopics();
            }
            else
            {
                Log($"Failed to create '{mqttClientName}' MQTT client");
                Cursor = Cursors.Default;
            }
        }


        private async void subscribeMqttTopics()
        {

            // Retreive topics from configuration file
            string mqttClientName = Properties.Settings.Default.mqttClientName;
            string topicScaraTarget = Properties.Settings.Default.mqttTopic_scaraTarget;
            string topicIdsStream = Properties.Settings.Default.mqttTopic_idsStream;
            string topicLuxonisStream = Properties.Settings.Default.mqttTopic_luxonisStream;
            string topicBaslerStream = Properties.Settings.Default.mqttTopic_baslerStream;


            // Subscribe to the necessary topics
            bool scaraSubscriberd = await _mqttClient.SubscribeClientToTopic(mqttClientName, topicScaraTarget);
            bool idsSubscribed = await _mqttClient.SubscribeClientToTopic(mqttClientName, topicIdsStream);
            bool baslerSubscribed = await _mqttClient.SubscribeClientToTopic(mqttClientName, topicBaslerStream);
            bool luxonisSubscribed = await _mqttClient.SubscribeClientToTopic(mqttClientName, topicLuxonisStream);

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
            Task<bool> destroyClient = _mqttClient.DestroyClient(clientName);
            if (destroyClient != null)
            {
                StatusMqttClient = false;
                _mqttClient = null;
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
