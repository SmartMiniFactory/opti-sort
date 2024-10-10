using CobraLibrary;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using static OptiSort.Program;

namespace OptiSort.userControls
{
    public partial class ucProcessView : UserControl
    {

        private frmMain _frmMain;

        List<Cameras> _camerasList = new List<Cameras> { };

        // TODO: insert in ucConfig
        string _scaraTopic = "optisort/scara/target";
        string _idsTopic = "optisort/ids/stream";
        string _luxonisTopic = "optisort/luxonis/stream";
        string _baslerTopic = "optisort/basler/stream";

        ucScara _ucScara;
        ucCameraStream _ucCameraStream;

        public ucProcessView(frmMain frmMain)
        {
            InitializeComponent();
            _frmMain = frmMain;

            // init combobox
            _frmMain.Log("Initializing cameras combobox");
            _camerasList.Add(new Cameras { ID = 0, Text = "Basler", mqttTopic = _baslerTopic });
            _camerasList.Add(new Cameras { ID = 1, Text = "IDS", mqttTopic = _idsTopic });
            _camerasList.Add(new Cameras { ID = 2, Text = "Luxonics", mqttTopic = _luxonisTopic });
            cmbCameras.DataSource = _camerasList;
            cmbCameras.DisplayMember = "Text";
            cmbCameras.ValueMember = "ID";


            // init scara dgv
            _frmMain.Log("Initializing scara datagridview");
            Cobra cobra600 = new Cobra();
            _ucScara = new ucScara(_frmMain);
            _ucScara.ScaraTarget = _scaraTopic;
            _ucScara.Cobra600 = cobra600;
            _ucScara.Dock = DockStyle.Fill;
            pnlScara.Controls.Clear();
            pnlScara.Controls.Add(_ucScara);


            // init robot3D view
            _frmMain.Log("Initializing scara 3D View");
            ucRobotView ucRobotView = new ucRobotView(_frmMain);
            ucRobotView.Dock = DockStyle.Fill;
            pnlRobot3D.Controls.Clear();
            pnlRobot3D.Controls.Add(ucRobotView);

            _ucScara.RobotConnected += ucRobotView.Create3DDisplay;


            // init flexibowl
            _frmMain.Log("Initializing flexibowl");
            ucFlexibowl ucFlexibowl = new ucFlexibowl();
            ucFlexibowl.Dock = DockStyle.Fill;
            pnlFlexibowl.Controls.Clear();
            pnlFlexibowl.Controls.Add(ucFlexibowl);


            // init camera view
            _frmMain.Log("Initializing cameras view");
            _ucCameraStream = new ucCameraStream(_camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex));
            _ucCameraStream.StreamTopic = _camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex).mqttTopic;
            _ucCameraStream.Dock = DockStyle.Fill;
            pnlCameraStream.Controls.Clear();
            pnlCameraStream.Controls.Add(_ucCameraStream);

            _frmMain.Log("Initialization complete");


            if (_frmMain._clientCreated)
                subscribeMqttTopics();

        }



        private async void subscribeMqttTopics()
        {
            // Subscribe to the necessary topics
            bool scaraSubscriberd = await _frmMain._mqttClient.SubscribeClientToTopic(_frmMain._mqttClientName, _scaraTopic);
            bool idsSubscribed = await _frmMain._mqttClient.SubscribeClientToTopic(_frmMain._mqttClientName, _idsTopic);
            bool baslerSubscribed = await _frmMain._mqttClient.SubscribeClientToTopic(_frmMain._mqttClientName, _baslerTopic);
            bool luxonisSubscribed = await _frmMain._mqttClient.SubscribeClientToTopic(_frmMain._mqttClientName, _luxonisTopic);

            // logging
            if (scaraSubscriberd) _frmMain.Log($"{_frmMain._mqttClientName} subscribed to {_scaraTopic}");
            else _frmMain.Log($"Unable subscribing {_frmMain._mqttClientName} to {_scaraTopic}");

            if (idsSubscribed) _frmMain.Log($"{_frmMain._mqttClientName} subscribed to {_idsTopic}");
            else _frmMain.Log($"Unable subscribing {_frmMain._mqttClientName} to {_idsTopic}");

            if (baslerSubscribed) _frmMain.Log($"{_frmMain._mqttClientName} subscribed to {_baslerTopic}");
            else _frmMain.Log($"Unable subscribing {_frmMain._mqttClientName} to {_baslerTopic}");

            if (luxonisSubscribed) _frmMain.Log($"{_frmMain._mqttClientName} subscribed to {_luxonisTopic}");
            else _frmMain.Log($"Unable subscribing {_frmMain._mqttClientName} to {_luxonisTopic}");

            // Subscribe user controls to the message received event
            _frmMain._mqttClient.MessageReceived += _ucScara.OnMessageReceived;
            _frmMain._mqttClient.MessageReceived += _ucCameraStream.OnMessageReceived;
        }



        private void cmbCameras_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (_ucCameraStream != null)
            {
                var newTopic = _camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex).mqttTopic;
                _ucCameraStream.StreamTopic = newTopic;
                _frmMain.Log($"Streaming topic updated to '{newTopic}'");
            }
        }

    }
}
