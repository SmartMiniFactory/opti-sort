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

        ucScara _ucScara;
        ucCameraStream _ucCameraStream;

        public ucProcessView(frmMain frmMain)
        {
            InitializeComponent();
            _frmMain = frmMain;

            // init combobox
            _frmMain.Log("Initializing cameras combobox");
            _camerasList.Add(new Cameras { ID = 0, Text = "Basler", mqttTopic = Properties.Settings.Default.mqttTopic_baslerStream });
            _camerasList.Add(new Cameras { ID = 1, Text = "IDS", mqttTopic = Properties.Settings.Default.mqttTopic_idsStream });
            _camerasList.Add(new Cameras { ID = 2, Text = "Luxonics", mqttTopic = Properties.Settings.Default.mqttTopic_luxonisStream });
            cmbCameras.DataSource = _camerasList;
            cmbCameras.DisplayMember = "Text";
            cmbCameras.ValueMember = "ID";


            // init scara dgv
            _frmMain.Log("Initializing scara datagridview");
            Cobra cobra600 = new Cobra();
            _ucScara = new ucScara(_frmMain);
            _ucScara.ScaraTarget = Properties.Settings.Default.mqttTopic_scaraTarget;
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
