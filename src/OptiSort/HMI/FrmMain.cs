using System.Linq;
using System.Windows.Forms;
using System.Threading.Tasks;
using OptiSort.userControls;
using System.IO;
using System.Text.Json;

namespace OptiSort
{
    public partial class frmMain : Form
    {

        /// <summary
        /// single MQTT broker where at least two topics are present: 
        /// 1. image streaming (one per camera, incoming); python file
        /// 2. scara's pick location streaming (incoming); same python file that operates with openCV on image
        /// 
        /// UDP protocol for flexibowl's communication (outgoing); flexibowl library
        /// 
        /// TCP connection with scara robot (in/out); cobra library
        /// </summary>
        /// 

        public MQTT _mqttClient;


        public frmMain()
        {
            InitializeComponent();


            // Initialize MQTT connection
            _mqttClient = new MQTT();
            InitializeMQTTClient();


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
        // -------------------------------------- MQTT ---------------------------------------
        // -----------------------------------------------------------------------------------

        private async void InitializeMQTTClient()
        {
            string mqttClientName = Properties.Settings.Default.mqttClientName;
            Task<bool> createClient = _mqttClient.CreateClient(mqttClientName);

            if (await createClient)
            {
                Log($"MQTT client '{mqttClientName}' created");
                subscribeMqttTopics();
            }
            else
                Log($"Failed to create '{mqttClientName}' MQTT client");
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

            // Subscribe user controls to the message received event
            //_mqttClient.MessageReceived += _ucScara.OnMessageReceived;
            //_mqttClient.MessageReceived += _ucCameraStream.OnMessageReceived;
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
            // TODO: detach active events
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

            Log("Switched to automatic process view");
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

            Log("Switched to manual control view");
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

            Log("Switched to configuration view");
        }


        // -----------------------------------------------------------------------------------
        // -------------------------------- SUPPORT FUNCTIONS --------------------------------
        // -----------------------------------------------------------------------------------
        public void Log(string msg)
        {
            lstLog.Items.Add(msg);
            lstLog.TopIndex = lstLog.Items.Count - 1; // showing last row
        }

    }
}
