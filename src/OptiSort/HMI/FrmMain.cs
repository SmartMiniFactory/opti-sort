using System.Linq;
using System.Windows.Forms;
using CobraLibrary;
using static OptiSort.Program;
using System.Collections.Generic;
using System.Threading.Tasks;

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

        MQTT _mqttClient;
        ucScara _ucScara;
        ucCameraStream _ucCameraStream;

        List<Cameras> _camerasList = new List<Cameras> { };

        string _scaraTopic = "optisort/scara/target";
        string _idsTopic = "optisort/ids/stream";
        string _luxonisTopic = "optisort/luxonis/stream";
        string _baslerTopic = "optisort/basler/stream";



        public frmMain()
        {
            InitializeComponent();
            InitializeMQTTClient();

            // init combobox
            Log("Initializing cameras combobox");
            _camerasList.Add(new Cameras { ID = 0, Text = "Basler", mqttTopic = _baslerTopic });
            _camerasList.Add(new Cameras { ID = 1, Text = "IDS", mqttTopic = _idsTopic });
            _camerasList.Add(new Cameras { ID = 2, Text = "Luxonics", mqttTopic = _luxonisTopic });
            cmbCameras.DataSource = _camerasList;
            cmbCameras.DisplayMember = "Text";
            cmbCameras.ValueMember = "ID";


            // init scara dgv
            Log("Initializing scara datagridview");
            Cobra cobra600 = new Cobra();
            _ucScara = new ucScara(this);
            _ucScara.ScaraTarget = _scaraTopic;
            _ucScara.Cobra600 = cobra600;
            _ucScara.Dock = DockStyle.Fill;
            pnlScara.Controls.Clear();
            pnlScara.Controls.Add(_ucScara);


            // init robot3D view
            Log("Initializing scara 3D View");
            ucRobotView ucRobotView = new ucRobotView(this);
            ucRobotView.Dock = DockStyle.Fill;
            pnlRobot3D.Controls.Clear();
            pnlRobot3D.Controls.Add(ucRobotView);


            // init camera view
            Log("Initializing cameras view");
            _ucCameraStream = new ucCameraStream(_camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex));
            _ucCameraStream.StreamTopic = _camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex).mqttTopic;
            _ucCameraStream.Dock = DockStyle.Fill;
            pnlCameraStream.Controls.Clear();
            pnlCameraStream.Controls.Add(_ucCameraStream);

            Log("Initialization complete");
        }

        private async void InitializeMQTTClient()
        {
            _mqttClient = new MQTT();

            // create new client
            var clientID = "OptiSort";
            Task<bool> client = _mqttClient.CreateClient(clientID);
            bool clientCreated = await client; // await (optional) pauses the execution here until the async task completes

            if (clientCreated)
            {
                Log($"MQTT client '{clientID}' created");


                // Subscribe to the necessary topics
                bool scaraSubscriberd = await _mqttClient.SubscribeClientToTopic(clientID, _scaraTopic);
                bool idsSubscribed = await _mqttClient.SubscribeClientToTopic(clientID, _idsTopic);
                bool baslerSubscribed = await _mqttClient.SubscribeClientToTopic(clientID, _baslerTopic);
                bool luxonisSubscribed = await _mqttClient.SubscribeClientToTopic(clientID, _luxonisTopic);

                // logging
                if (scaraSubscriberd) Log($"{clientID} subscribed to {_scaraTopic}");
                else Log($"Unable subscribing {clientID} to {_scaraTopic}");

                if (idsSubscribed) Log($"{clientID} subscribed to {_idsTopic}");
                else Log($"Unable subscribing {clientID} to {_idsTopic}");

                if (baslerSubscribed) Log($"{clientID} subscribed to {_baslerTopic}");
                else Log($"Unable subscribing {clientID} to {_baslerTopic}");

                if (luxonisSubscribed) Log($"{clientID} subscribed to {_luxonisTopic}");
                else Log($"Unable subscribing {clientID} to {_luxonisTopic}");


                // Subscribe user controls to the message received event
                _mqttClient.MessageReceived += _ucScara.OnMessageReceived;
                _mqttClient.MessageReceived += _ucCameraStream.OnMessageReceived;
            }
            else
                Log($"Failed to create '{clientID}' MQTT client");
        }

        public void Log(string msg)
        {
            lstLog.Items.Add(msg);
            lstLog.TopIndex = lstLog.Items.Count - 1;
        }

        private void cmbCameras_SelectedIndexChanged(object sender, System.EventArgs e)
        {
            if (_ucCameraStream != null)
            {
                var newTopic = _camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex).mqttTopic;
                _ucCameraStream.StreamTopic = newTopic;
                Log($"Streaming topic updated to '{newTopic}'");
            }
        }
    }
}
