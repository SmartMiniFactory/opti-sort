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

        private optisort_mgr _manager;



        // -----------------------------------------------------------------------------------
        // ---------------------------------- CONSTRUCTOR ------------------------------------
        // -----------------------------------------------------------------------------------

        public frmMain()
        {
            InitializeComponent();

            _manager = new optisort_mgr();

            // Display default user control
            ucManualControl ucManualControl = new ucManualControl(_manager);
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
            _manager.StreamingTopic = _camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex).mqttTopic;

            // init camera view
            ucCameraStream ucCameraStream = new ucCameraStream(_manager);
            ucCameraStream.Dock = DockStyle.Fill;
            pnlCameraStream.Controls.Clear();
            pnlCameraStream.Controls.Add(ucCameraStream);


            // Initialize robot panel view
            ucRobotSimulation ucRobotSimulation = new ucRobotSimulation(_manager);
            ucRobotSimulation.Dock = DockStyle.Fill;
            pnlRobotView.Controls.Clear();
            pnlRobotView.Controls.Add(ucRobotSimulation);

            
            // attach events
            _manager.PropertyChanged += RefreshStatusBar;
            _manager.NewUserControlRequested += AddNewUc;
            

            // log box setup
            _manager.LogEvent += OnLogEvent;
            lstLog.ItemHeight = 15; // adjusting interline between log rows

            _manager.Log("OptiSort ready for operation: please connect systems (Scara robot, flexibowl, MQTT service) to begin", false, false);
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
            //    MessageBox.Show("You should connect all the entities (Scara robot, Flexibowl, MQTT Client) to start the automatic process");
            //    return;
            //}

            CleanPnlCurrentUc();

            ucProcessView ucProcessView = new ucProcessView(_manager);
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
                MessageBox.Show("Click the start button to activate automatic process");
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

            ucManualControl ucManualControl = new ucManualControl(_manager);
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

            ucConfiguration ucConfiguration = new ucConfiguration(_manager);
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
            btnScaraConnect.Enabled = !_manager.StatusScara;
            btnEmulateScara.Enabled = !_manager.StatusScara;
            btnFlexibowlConnect.Enabled = !_manager.StatusFlexibowl;
            btnMqttConnect.Enabled = !_manager.StatusMqttClient;
            // disconnection buttons should be enabled when status is true, disabled when status is false
            btnScaraDisconnect.Enabled = _manager.StatusScara;
            btnFlexibowlDisconnect.Enabled = _manager.StatusFlexibowl;
            btnMqttDisconnect.Enabled = _manager.StatusMqttClient;

            if (e.PropertyName == nameof(_manager.StatusScara))
            {
                if (_manager.StatusScara == true)
                {
                    lblScaraStatusValue.Text = "Online";
                    lblScaraStatusValue.ForeColor = Color.Green;
                    btnScaraConnect.BackgroundImage = Properties.Resources.connectedDisabled_2x2_pptx;
                    btnScaraDisconnect.BackgroundImage = Properties.Resources.disconnectedEnabled_2x2_pptx;
                    if (_manager.StatusScaraEmulation)
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
                    if (_manager.StatusScaraEmulation)
                        btnEmulateScara.BackgroundImage = Properties.Resources.emulationEnabled_2x2_pptx;
                    else
                        btnEmulateScara.BackgroundImage = Properties.Resources.robotEnabled_2x2_pptx;
                }
            }

            if(e.PropertyName == nameof(_manager.StatusScaraEmulation))
            {
                if (_manager.StatusScaraEmulation)
                    btnEmulateScara.BackgroundImage = Properties.Resources.emulationEnabled_2x2_pptx;

                else
                    btnEmulateScara.BackgroundImage = Properties.Resources.robotEnabled_2x2_pptx;
            }



            if (e.PropertyName == nameof(_manager.StatusFlexibowl))
            {
                if (_manager.StatusFlexibowl == true)
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

            if (e.PropertyName == nameof(_manager.StatusMqttClient))
            {
                if (_manager.StatusMqttClient == true)
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
        }

        private void btnScaraConnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            bool success = _manager.ConnectScara();

            if (success)
            {
                // add robot rendering to panel
                pnlRobotView.Controls.Clear();
                pnlRobotView.Controls.Add(_manager.Cobra600.SimulationContainerControl); 
                pnlRobotView.Refresh();
            }

            Cursor = Cursors.Default;
        }

        private void btnScaraDisconnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            bool success = _manager.DisconnectScara();
            if (success)
            {
                pnlRobotView.Controls.Remove(_manager.Cobra600.SimulationContainerControl);
                pnlRobotView.Controls.Clear();
            }
            Cursor = Cursors.Default;

        }

        private void btnEmulateScara_Click(object sender, EventArgs e)
        {
            _manager.ToggleScaraEmulationMode();
        }

        private void btnFlexibowlConnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            bool success = _manager.ConnectFlexibowl();
            // TODO: useful to do something here?
            Cursor = Cursors.Default;
        }

        private void btnFlexibowlDisconnect_Click(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            bool success = _manager.DisconnectFlexibowl();
            // TODO: usefult to do something here?
            Cursor = Cursors.Default;
        }

        private void btnMqttConnect_Click(object sender, EventArgs e)
        {
            // async call, no need for handling cursor
            _manager.ConnectMQTTClient();
        }

        private void btnMqttDisconnect_Click(object sender, EventArgs e)
        {
            // async call, no need for handling cursor
            _manager.DisconnectMqttClient(Properties.Settings.Default.mqtt_client);
        }


        // -----------------------------------------------------------------------------------
        // ------------------------------- CAMERA STREAM -------------------------------------
        // -----------------------------------------------------------------------------------

        private void cmbCameras_SelectedIndexChanged(object sender, EventArgs e)
        {
            _manager.StreamingTopic = _camerasList.FirstOrDefault(camera => camera.ID == cmbCameras.SelectedIndex).mqttTopic; 
        }

        private void OnLogEvent(optisort_mgr.LogEntry logEntry)
        {
            // Update the ListBox with the new log entry
            lstLog.Items.Add(logEntry);

            // Ensure the last row is visible
            lstLog.TopIndex = lstLog.Items.Count - 1;
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


    }
}
