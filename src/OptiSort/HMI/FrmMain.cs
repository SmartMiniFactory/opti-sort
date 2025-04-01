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

        private class Cameras
        {
            public int ID { get; set; }
            public string Text { get; set; }
            public string mqttTopic { get; set; }
        }
        List<Cameras> _camerasList = new List<Cameras> { }; // TODO: review definition

        internal optisort_mgr manager;



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

            btnRun.Enabled = true;
            btnRun.BackgroundImage = Properties.Resources.playPassivated_2x2_pptx;
            btnStop.Enabled = false;
            btnStop.BackgroundImage = Properties.Resources.stopEnabled_2x2_pptx;

            if (!ucProcessView.AutomaticProcess)
            {
                manager.NonBlockingMessageBox("Click the start button to activate automatic process", "Interlock!", MessageBoxIcon.Hand);
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

            ucManualControl ucManualControl = new ucManualControl(manager);
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

            ucConfiguration ucConfiguration = new ucConfiguration(manager);
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



        // -----------------------------------------------------------------------------------
        // ---------------------------------- STATUS BAR -------------------------------------
        // -----------------------------------------------------------------------------------



        private void RefreshStatusBar(object sender, PropertyChangedEventArgs e)
        {
            // Check which property changed and trigger corresponding logic

            // connection buttons should be enabled when status is false, disabled when status is true
            btnScaraConnect.Enabled = !manager.StatusScara;
            btnEmulateScara.Enabled = !manager.StatusScara;
            btnFlexibowlConnect.Enabled = !manager.StatusFlexibowl;
            btnMqttConnect.Enabled = !manager.StatusMqttClient;
            btnCamerasConnect.Enabled = !manager.StatusCameraManager;
            btnCameraTesting.Enabled = !manager.StatusCameraManager;

            // disconnection buttons should be enabled when status is true, disabled when status is false
            btnScaraDisconnect.Enabled = manager.StatusScara;
            btnFlexibowlDisconnect.Enabled = manager.StatusFlexibowl;
            btnMqttDisconnect.Enabled = manager.StatusMqttClient;
            btnCamerasDisconnect.Enabled = manager.StatusCameraManager;

            if (e.PropertyName == nameof(manager.StatusScara))
            {
                if (manager.StatusScara == true)
                {
                    lblScaraStatusValue.Text = "Online";
                    lblScaraStatusValue.ForeColor = Color.Green;
                    btnScaraConnect.BackgroundImage = Properties.Resources.connectedDisabled_2x2_pptx;
                    btnScaraDisconnect.BackgroundImage = Properties.Resources.disconnectedEnabled_2x2_pptx;
                    if (manager.StatusScaraEmulation)
                        btnEmulateScara.BackgroundImage = Properties.Resources.emulationDisabled_2x2_pptx;
                    else
                        btnEmulateScara.BackgroundImage = Properties.Resources.robotDisabled_2x2_pptx;

                }
                else
                {
                    lblScaraStatusValue.Text = "Offline";
                    lblScaraStatusValue.ForeColor = Color.Red;
                    btnScaraConnect.BackgroundImage = Properties.Resources.connectedEnabled_2x2_pptx;
                    btnScaraDisconnect.BackgroundImage = Properties.Resources.disconnectedDisabled_2x2_pptx;
                    if (manager.StatusScaraEmulation)
                        btnEmulateScara.BackgroundImage = Properties.Resources.emulationEnabled_2x2_pptx;
                    else
                        btnEmulateScara.BackgroundImage = Properties.Resources.robotEnabled_2x2_pptx;
                }
            }

            if (e.PropertyName == nameof(manager.StatusScaraEmulation))
            {
                if (manager.StatusScaraEmulation)
                    btnEmulateScara.BackgroundImage = Properties.Resources.emulationEnabled_2x2_pptx;

                else
                    btnEmulateScara.BackgroundImage = Properties.Resources.robotEnabled_2x2_pptx;
            }



            if (e.PropertyName == nameof(manager.StatusFlexibowl))
            {
                if (manager.StatusFlexibowl == true)
                {
                    lblFlexibowlStatusValue.Text = "Online";
                    lblFlexibowlStatusValue.ForeColor = Color.Green;
                    btnFlexibowlConnect.BackgroundImage = Properties.Resources.connectedDisabled_2x2_pptx;
                    btnFlexibowlDisconnect.BackgroundImage = Properties.Resources.disconnectedEnabled_2x2_pptx;
                }
                else
                {
                    lblFlexibowlStatusValue.Text = "Offline";
                    lblFlexibowlStatusValue.ForeColor = Color.Red;
                    btnFlexibowlConnect.BackgroundImage = Properties.Resources.connectedEnabled_2x2_pptx;
                    btnFlexibowlDisconnect.BackgroundImage = Properties.Resources.disconnectedDisabled_2x2_pptx;
                }
            }

            if (e.PropertyName == nameof(manager.StatusMqttClient))
            {
                if (manager.StatusMqttClient == true)
                {
                    lblMqttStatusValue.Text = "Online";
                    lblMqttStatusValue.ForeColor = Color.Green;
                    btnMqttConnect.BackgroundImage = Properties.Resources.connectedDisabled_2x2_pptx;
                    btnMqttDisconnect.BackgroundImage = Properties.Resources.disconnectedEnabled_2x2_pptx;
                }
                else
                {
                    lblMqttStatusValue.Text = "Offline";
                    lblMqttStatusValue.ForeColor = Color.Red;
                    btnMqttConnect.BackgroundImage = Properties.Resources.connectedEnabled_2x2_pptx;
                    btnMqttDisconnect.BackgroundImage = Properties.Resources.disconnectedDisabled_2x2_pptx;
                }
            }

            if (e.PropertyName == nameof(manager.StatusCameraManager))
            {
                if (manager.StatusCameraManager == true)
                {
                    lblCamerasStatusValue.Text = "Online";
                    lblCamerasStatusValue.ForeColor = Color.Green;
                    btnCamerasConnect.BackgroundImage = Properties.Resources.connectedDisabled_2x2_pptx;
                    btnCamerasDisconnect.BackgroundImage = Properties.Resources.disconnectedEnabled_2x2_pptx;
                    if (manager.StatusCameraTesting)
                        btnCameraTesting.BackgroundImage = Properties.Resources.webcamDisabled_2x2_pptx;
                    else
                        btnCameraTesting.BackgroundImage = Properties.Resources.camerasDisabled_2x2_pptx;
                }
                else
                {
                    lblCamerasStatusValue.Text = "Offline";
                    lblCamerasStatusValue.ForeColor = Color.Red;
                    btnCamerasConnect.BackgroundImage = Properties.Resources.connectedEnabled_2x2_pptx;
                    btnCamerasDisconnect.BackgroundImage = Properties.Resources.disconnectedDisabled_2x2_pptx;
                    if (manager.StatusCameraTesting)
                        btnCameraTesting.BackgroundImage = Properties.Resources.webcamEnabled_2x2_pptx;
                    else
                        btnCameraTesting.BackgroundImage = Properties.Resources.camerasEnabled_2x2_pptx;
                }
            }

            if (e.PropertyName == nameof(manager.StatusCameraTesting))
            {
                if (manager.StatusCameraTesting)
                    btnCameraTesting.BackgroundImage = Properties.Resources.webcamEnabled_2x2_pptx;

                else
                    btnCameraTesting.BackgroundImage = Properties.Resources.camerasEnabled_2x2_pptx;
            }
        }

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
            _ = manager.ConnectMQTTClient();
        }

        private void btnMqttDisconnect_Click(object sender, EventArgs e)
        {
            // async call, no need for handling cursor
            _ = manager.DisconnectMqttClient();
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
