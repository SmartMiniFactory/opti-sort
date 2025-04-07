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
using System.Drawing.Text;
using System.IO;
using OptiSort.systems;


namespace OptiSort
{
    public partial class frmMain : Form
    {

        // -----------------------------------------------------------------------------------
        // ---------------------------------- DECLARATIONS -----------------------------------
        // -----------------------------------------------------------------------------------

        //NOTE: remember to reactivate exception settings > managed debugging assistant > loader lock for release to prod


        internal optisort_mgr manager;

        private class Cameras
        {
            public int ID { get; set; }
            public string Text { get; set; }
            public string mqttTopic { get; set; }
        }
        List<Cameras> _camerasList = new List<Cameras> { }; // TODO: review definition

        private string _mqttClient = Properties.Settings.Default.mqtt_client;
        private string _mqttBroker = Properties.Settings.Default.mqtt_broker;



        // -----------------------------------------------------------------------------------
        // ---------------------------------- CONSTRUCTOR ------------------------------------
        // -----------------------------------------------------------------------------------

        public frmMain()
        {
            InitializeComponent();

            manager = new optisort_mgr(this);

            // Display default user control
            ucManualControl ucManualControl = new ucManualControl(manager);
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
            manager.StreamingTopic = _camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex).mqttTopic;

            // init camera view
            ucCameraStream ucCameraStream = new ucCameraStream(manager);
            ucCameraStream.Dock = DockStyle.Fill;
            pnlCameraStream.Controls.Clear();
            pnlCameraStream.Controls.Add(ucCameraStream);


            // Initialize robot panel view
            ucRobotSimulation ucRobotSimulation = new ucRobotSimulation(manager);
            ucRobotSimulation.Dock = DockStyle.Fill;
            pnlRobotView.Controls.Clear();
            pnlRobotView.Controls.Add(ucRobotSimulation);


            // attach events
            manager.PropertyChanged += RefreshStatusBar;
            manager.NewUserControlRequested += AddNewUc;


            // log box setup
            manager.LogEvent += OnLogEvent;
            lstLog.ItemHeight = 15; // adjusting interline between log rows

            // non blocking message box setup
            manager.MessageBoxEvent += ShowMessage;

            manager.Log("OptiSort ready for operation: please connect systems (Scara robot, flexibowl, MQTT service) to begin", false, false);

            // attach form closing event to process killer
            this.FormClosing += FrmMain_FormClosing;
        }

        private void FrmMain_FormClosing(object sender, FormClosingEventArgs e)
        {
            manager.KillAllProcesses(); // terminate all the python-related background processes
        }


        // -----------------------------------------------------------------------------------
        // -------------------------------- FORM NAVIGATION ----------------------------------
        // -----------------------------------------------------------------------------------



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
            //    manager.NonBlockingMessageBox("You should connect all the entities (Scara robot, Flexibowl, MQTT Client) to start the automatic process");
            //    return;
            //}

            CleanPnlCurrentUc();

            ucProcessView ucProcessView = new ucProcessView(manager);
            ucProcessView.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Add(ucProcessView);

            btnAuto.Enabled = false;
            btnAuto.BackgroundImage = Properties.Resources.autoEnabled_2x2_pptx;
            btnManual.Enabled = true;
            btnManual.BackgroundImage = Properties.Resources.manualDisabled_2x2_pptx;
            btnConfig.Enabled = true;
            btnConfig.BackgroundImage = Properties.Resources.configDisabled_2x2_pptx;

            if (!ucProcessView.AutomaticProcess)
            {
                manager.NonBlockingMessageBox("Click the start button to activate automatic process", "Interlock!", MessageBoxIcon.Hand);
            }

        }


        private void btnManual_Click(object sender, System.EventArgs e)
        {
            CleanPnlCurrentUc();

            ucManualControl ucManualControl = new ucManualControl(manager);
            ucManualControl.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Add(ucManualControl);

            btnAuto.Enabled = true;
            btnAuto.BackgroundImage = Properties.Resources.autoDisabled_2x2_pptx;
            btnManual.Enabled = false;
            btnManual.BackgroundImage = Properties.Resources.manualEnabled_2x2_pptx;
            btnConfig.Enabled = true;
            btnConfig.BackgroundImage = Properties.Resources.configDisabled_2x2_pptx;

        }

        private void btnConfig_Click(object sender, System.EventArgs e)
        {
            CleanPnlCurrentUc();

            ucConfiguration ucConfiguration = new ucConfiguration(manager);
            ucConfiguration.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Add(ucConfiguration);

            btnAuto.Enabled = true;
            btnAuto.BackgroundImage = Properties.Resources.autoDisabled_2x2_pptx;
            btnManual.Enabled = true;
            btnManual.BackgroundImage = Properties.Resources.manualDisabled_2x2_pptx;
            btnConfig.Enabled = false;
            btnConfig.BackgroundImage = Properties.Resources.configEnabled_2x2_pptx;

        }



        // -----------------------------------------------------------------------------------
        // ---------------------------------- STATUS BAR -------------------------------------
        // -----------------------------------------------------------------------------------



        private void RefreshStatusBar(object sender, PropertyChangedEventArgs e)
        {
            // Update connect/disconnect button states based on statuses
            UpdateConnectionButtons();

            switch (e.PropertyName)
            {
                case nameof(manager.StatusScara):
                    UpdateScaraStatus();
                    break;

                case nameof(manager.StatusScaraEmulation):
                    UpdateScaraEmulation();
                    break;

                case nameof(manager.StatusFlexibowl):
                    UpdateFlexibowlStatus();
                    break;

                case nameof(manager.StatusMqttClient):
                    UpdateMqttStatus();
                    break;

                case nameof(manager.StatusCameraManager):
                    UpdateCameraStatus();
                    break;

                case nameof(manager.StatusCameraTesting):
                    UpdateCameraTesting();
                    break;

                case nameof(manager.StatusDigitalTwin):
                    UpdateDigitalTwin();
                    break;
            }
        }

        private void UpdateConnectionButtons()
        {
            btnScaraConnect.Enabled = !manager.StatusScara;
            btnEmulateScara.Enabled = !manager.StatusScara;
            btnFlexibowlConnect.Enabled = !manager.StatusFlexibowl;
            btnMqttConnect.Enabled = !manager.StatusMqttClient;
            btnCamerasConnect.Enabled = !manager.StatusCameraManager;
            btnCameraTesting.Enabled = !manager.StatusCameraManager;
            btnDtConnect.Enabled = !manager.StatusDigitalTwin;

            btnScaraDisconnect.Enabled = manager.StatusScara;
            btnFlexibowlDisconnect.Enabled = manager.StatusFlexibowl;
            btnMqttDisconnect.Enabled = manager.StatusMqttClient;
            btnCamerasDisconnect.Enabled = manager.StatusCameraManager;
            btnDtDisconnect.Enabled = manager.StatusDigitalTwin;
        }

        private void UpdateScaraStatus()
        {
            pbScaraStatus.Image = manager.StatusScara ? Properties.Resources.on_2x2_pptx : Properties.Resources.off_2x2_pptx;
            btnScaraConnect.BackgroundImage = manager.StatusScara ? Properties.Resources.connectedDisabled_2x2_pptx : Properties.Resources.connectedEnabled_2x2_pptx;
            btnScaraDisconnect.BackgroundImage = manager.StatusScara ? Properties.Resources.disconnectedEnabled_2x2_pptx : Properties.Resources.disconnectedDisabled_2x2_pptx;

            UpdateScaraEmulation(); // StatusEmulation also affects this
        }

        private void UpdateScaraEmulation()
        {
            if (manager.StatusScara)
            {
                btnEmulateScara.BackgroundImage = manager.StatusScaraEmulation
                    ? Properties.Resources.emulationDisabled_2x2_pptx
                    : Properties.Resources.robotDisabled_2x2_pptx;
            }
            else
            {
                btnEmulateScara.BackgroundImage = manager.StatusScaraEmulation
                    ? Properties.Resources.emulationEnabled_2x2_pptx
                    : Properties.Resources.robotEnabled_2x2_pptx;
            }
        }

        private void UpdateFlexibowlStatus()
        {
            pbFlexibowlStatus.Image = manager.StatusFlexibowl ? Properties.Resources.on_2x2_pptx : Properties.Resources.off_2x2_pptx;
            btnFlexibowlConnect.BackgroundImage = manager.StatusFlexibowl ? Properties.Resources.connectedDisabled_2x2_pptx : Properties.Resources.connectedEnabled_2x2_pptx;
            btnFlexibowlDisconnect.BackgroundImage = manager.StatusFlexibowl ? Properties.Resources.disconnectedEnabled_2x2_pptx : Properties.Resources.disconnectedDisabled_2x2_pptx;
        }

        private void UpdateMqttStatus()
        {
            pbMqttStatus.Image = manager.StatusMqttClient ? Properties.Resources.on_2x2_pptx : Properties.Resources.off_2x2_pptx;
            btnMqttConnect.BackgroundImage = manager.StatusMqttClient ? Properties.Resources.connectedDisabled_2x2_pptx : Properties.Resources.connectedEnabled_2x2_pptx;
            btnMqttDisconnect.BackgroundImage = manager.StatusMqttClient ? Properties.Resources.disconnectedEnabled_2x2_pptx : Properties.Resources.disconnectedDisabled_2x2_pptx;
        }

        private void UpdateCameraStatus()
        {
            pbCameraStatus.Image = manager.StatusCameraManager ? Properties.Resources.on_2x2_pptx : Properties.Resources.off_2x2_pptx;
            btnCamerasConnect.BackgroundImage = manager.StatusCameraManager ? Properties.Resources.connectedDisabled_2x2_pptx : Properties.Resources.connectedEnabled_2x2_pptx;
            btnCamerasDisconnect.BackgroundImage = manager.StatusCameraManager ? Properties.Resources.disconnectedEnabled_2x2_pptx : Properties.Resources.disconnectedDisabled_2x2_pptx;

            UpdateCameraTesting(); // StatusCameraTesting also affects this
        }

        private void UpdateCameraTesting()
        {
            if (manager.StatusCameraManager)
            {
                btnCameraTesting.BackgroundImage = manager.StatusCameraTesting
                    ? Properties.Resources.webcamDisabled_2x2_pptx
                    : Properties.Resources.camerasDisabled_2x2_pptx;
            }
            else
            {
                btnCameraTesting.BackgroundImage = manager.StatusCameraTesting
                    ? Properties.Resources.webcamEnabled_2x2_pptx
                    : Properties.Resources.camerasEnabled_2x2_pptx;
            }
        }

        private void UpdateDigitalTwin()
        {
            pbDtStatus.Image = manager.StatusDigitalTwin ? Properties.Resources.on_2x2_pptx : Properties.Resources.off_2x2_pptx;
            btnDtConnect.BackgroundImage = manager.StatusDigitalTwin ? Properties.Resources.connectedDisabled_2x2_pptx : Properties.Resources.connectedEnabled_2x2_pptx;
            btnDtDisconnect.BackgroundImage = manager.StatusDigitalTwin ? Properties.Resources.disconnectedEnabled_2x2_pptx : Properties.Resources.disconnectedDisabled_2x2_pptx;
        }

        // -----------------------------------------------------------------------------------
        // ---------------------------------- CONNECTIONS ------------------------------------
        // -----------------------------------------------------------------------------------

        private void btnScaraConnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            bool success = manager.ConnectScara();

            if (success)
            {
                // add robot rendering to panel
                pnlRobotView.Controls.Clear();
                pnlRobotView.Controls.Add(manager.Cobra600.SimulationContainerControl);
                pnlRobotView.Refresh();
            }

            Cursor = Cursors.Default;
        }

        private void btnScaraDisconnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            bool success = manager.DisconnectScara();
            if (success)
            {
                pnlRobotView.Controls.Remove(manager.Cobra600.SimulationContainerControl);
                pnlRobotView.Controls.Clear();
            }
            Cursor = Cursors.Default;

        }

        private void btnEmulateScara_Click(object sender, EventArgs e)
        {
            manager.ToggleScaraEmulationMode();
        }

        private void btnFlexibowlConnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            bool success = manager.ConnectFlexibowl();
            // TODO: useful to do something here?
            Cursor = Cursors.Default;
        }

        private void btnFlexibowlDisconnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            bool success = manager.DisconnectFlexibowl();
            // TODO: usefult to do something here?
            Cursor = Cursors.Default;
        }

        private void btnMqttConnect_Click(object sender, EventArgs e)
        {
            // async call, no need for handling cursor
            _ = manager.ConnectMQTTClient(_mqttClient, _mqttBroker);
        }

        private void btnMqttDisconnect_Click(object sender, EventArgs e)
        {
            // async call, no need for handling cursor
            _ = manager.DisconnectMqttClient(_mqttClient);
        }

        private void btnCameraTesting_Click(object sender, EventArgs e)
        {
            manager.ToggleCameraTestingMode();
        }

        private void btnCamerasConnect_Click(object sender, EventArgs e)
        {
            manager.ConnectCameras();
        }

        private void btnCamerasDisconnect_Click(object sender, EventArgs e)
        {
            manager.DisconnectCameras();
        }

        private void btnDtConnect_Click(object sender, EventArgs e)
        {
            manager.ConnectDigitalTwin();
        }

        private void btnDtDisconnect_Click(object sender, EventArgs e)
        {
            manager.DisconnectDigitalTwin();
        }

        // -----------------------------------------------------------------------------------
        // ------------------------------- CAMERA STREAM -------------------------------------
        // -----------------------------------------------------------------------------------

        private void cmbCameras_SelectedIndexChanged(object sender, EventArgs e)
        {
            manager.UpdateStreamingTopic(_camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex).mqttTopic);
        }

        private void OnLogEvent(optisort_mgr.LogEntry logEntry)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => OnLogEvent(logEntry)));
            }
            else
            {
                // Update the ListBox with the new log entry
                lstLog.Items.Add(logEntry);

                // Ensure the last row is visible
                lstLog.TopIndex = lstLog.Items.Count - 1;
            }
        }


        public void LstLog_DrawItem(object sender, DrawItemEventArgs e)
        {
            if (e.Index < 0) return;

            // Retrieve the LogEntry object
            var logEntry = lstLog.Items[e.Index] as optisort_mgr.LogEntry;
            if (logEntry == null) return;

            // Set the text color based on IsError or isSuccess
            Brush textBrush = logEntry.IsError ? Brushes.Red : logEntry.IsSuccess ? Brushes.Green : Brushes.Black;

            // Draw the background and text
            e.DrawBackground();
            e.Graphics.DrawString(logEntry.Message, e.Font, textBrush, e.Bounds);
            e.DrawFocusRectangle();
        }


        private void ShowMessage(string message, string title, MessageBoxIcon icon)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action(() => ShowMessage(message, title, icon)));
                return;
            }

            MessageBox.Show(message, title, MessageBoxButtons.OK, icon);
        }

        
    }
}
