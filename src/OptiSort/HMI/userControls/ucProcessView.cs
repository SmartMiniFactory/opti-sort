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

        private class Cameras
        {
            public int ID { get; set; }
            public string Text { get; set; }
            public string mqttTopic { get; set; }
        }
        List<Cameras> _camerasList = new List<Cameras> { };

        private ucCameraStream _ucCameraStream; 
        private ucScaraTargets _ucScaraTargets;

        public ucProcessView(frmMain frmMain)
        {
            InitializeComponent();
            _frmMain = frmMain;

            // init combobox
            _camerasList.Add(new Cameras { ID = 0, Text = "Basler", mqttTopic = Properties.Settings.Default.mqttTopic_baslerStream });
            _camerasList.Add(new Cameras { ID = 1, Text = "IDS", mqttTopic = Properties.Settings.Default.mqttTopic_idsStream });
            _camerasList.Add(new Cameras { ID = 2, Text = "Luxonics", mqttTopic = Properties.Settings.Default.mqttTopic_luxonisStream });
            cmbCameras.DataSource = _camerasList;
            cmbCameras.DisplayMember = "Text";
            cmbCameras.ValueMember = "ID";
            

            // init camera view
            _ucCameraStream = new ucCameraStream();
            _ucCameraStream.StreamTopic = _camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex).mqttTopic;
            _ucCameraStream.Dock = DockStyle.Fill;
            pnlCameraStream.Controls.Clear();
            pnlCameraStream.Controls.Add(_ucCameraStream);
            _frmMain._mqttClient.MessageReceived += _ucCameraStream.OnMessageReceived; // enable MQTT messages to trigger the user control
            _frmMain.Log("Camera stream attached to MQTT messages");

            // init scara dgv
            _ucScaraTargets = new ucScaraTargets(_frmMain); // using log function
            _ucScaraTargets.Dock = DockStyle.Fill;
            pnlScara.Controls.Clear();
            pnlScara.Controls.Add(_ucScaraTargets);
            _frmMain._mqttClient.MessageReceived += _ucScaraTargets.OnMessageReceived; // enable MQTT messages to trigger the user control
            _frmMain.Log("Scara targets dgv attached to MQTT messages");

            // init robot3D view
            ucRobotView ucRobotView = new ucRobotView();
            ucRobotView.Dock = DockStyle.Fill;
            pnlRobot3D.Controls.Clear();
            pnlRobot3D.Controls.Add(ucRobotView);

            // connect scara and 3D view (TO REVIEW)
            _ucScaraTargets.RobotConnected += ucRobotView.Create3DDisplay;


            // init flexibowl
            ucFlexibowl ucFlexibowl = new ucFlexibowl();
            ucFlexibowl.Dock = DockStyle.Fill;
            pnlFlexibowl.Controls.Clear();
            pnlFlexibowl.Controls.Add(ucFlexibowl);
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
