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
            List<Cameras> camerasList = new List<Cameras>
            {
                new Cameras { ID = 0, Text = "Basler", mqttTopic = _baslerTopic },
                new Cameras { ID = 1, Text = "IDS", mqttTopic = _idsTopic },
                new Cameras { ID = 2, Text = "Luxonics", mqttTopic = _luxonisTopic }
            };
            cmbCameras.DataSource = camerasList;
            cmbCameras.DisplayMember = "Text";
            cmbCameras.ValueMember = "ID";
            cmbCameras.DataSource = camerasList;


            // init scara dgv
            Log("Initializing scara datagridview");
            Cobra cobra600 = new Cobra();
            _ucScara = new ucScara(this);
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
            // TODO: add selection from txtbox
            Log("Initializing cameras view");
            ucCameraStream ucCameraStream = new ucCameraStream(camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex), _mqttClient);
            ucCameraStream.Dock = DockStyle.Fill;
            pnlCameraStream.Controls.Clear();
            pnlCameraStream.Controls.Add(ucCameraStream);
            Log("Initialization complete"); 
        }

        private async void InitializeMQTTClient()
        {
            _mqttClient = new MQTT();

            // create new client
            var clientID = "Optisort";
            Task<bool> client = _mqttClient.CreateClient(clientID);
            bool clientCreated = await client; // await (optional) pauses the execution here until the async task completes

            if (clientCreated)
            {
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
                //_mqttClient.MessageReceived += ucCameraStream.OnMessageReceived;

               
            }
            else
                Log($"Failed to create '{clientID}' MQTT client");
        }

        // TODO: come connettersi alla stream della telecamera? Cosa inviare all'ucCameraStream per decodificare l'immagine? Rivedere funzioni li dentro??


        public void Log(string msg)
        {
            lstLog.Items.Add(msg);
            lstLog.TopIndex = lstLog.Items.Count - 1;
        }
    }
}
