using Ace.Core.Server;
using FlexibowlLibrary;
using OptiSort.Classes;
using System;
using System.ComponentModel;
using System.Runtime.Remoting.Channels;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;


namespace OptiSort.userControls
{
    public partial class ucProcessView : UserControl
    {

        private optisort_mgr _manager;
        private ucScaraTargets ScaraTargets;
        private PerformanceReport _report;
        private Watchdog _watchdog;

        private bool _scaraIsMoving = false;
        private bool _flexibowlIsMoving = false;

        private DateTime _startTime;
        private int _counterDetectedA = 0;
        private int _counterDetectedB = 0;
        private int _counterPicked = 0;
        private int _counterDiscarded = 0;

        internal ucProcessView(optisort_mgr manager)
        {
            InitializeComponent();
            _manager = manager;

            // init scara dgv
            ScaraTargets = new ucScaraTargets(_manager); // using log function
            ScaraTargets.Dock = DockStyle.Fill;
            pnlScara.Controls.Clear();
            pnlScara.Controls.Add(ScaraTargets);

            _manager.PropertyChanged += OnPropertyChanged;

            RefreshControls();
        }


        // ----------------------------------------------------- Events --------------------------------------------------


        private void OnPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(_manager.AutomaticProcess) || e.PropertyName == nameof(_manager.StatusScara) || e.PropertyName == nameof(_manager.StatusMqttClient))
            {
                RefreshControls();
            }

            if (e.PropertyName == nameof(_manager.StatusScara) && !_manager.StatusScara && _manager.AutomaticProcess) // scara status dropped: interrupt process
            {
                InterruptProcess();
            }

            if (e.PropertyName == nameof(_manager.StatusMqttClient) && !_manager.StatusMqttClient && _manager.AutomaticProcess) // mqtt connection dropped: interrupt process
            {
                InterruptProcess();
            }

        }
        

        private void OnMessageReceived(string topic, JsonElement message)
        {
            if (topic == Properties.Settings.Default.mqtt_topic_scaraTarget && !_flexibowlIsMoving) // topic should be correct, plus flexibowl shold be still to consider coordinates as valid
                ScaraTargets.UpdateTargetTable(message);

            else if (topic == "PythonResultOrSomething...")
            {
                _manager.MqttClient.MessageReceived -= OnMessageReceived;
                CompleteProcess(message);
            }
        }


        private void OnObjectDetected()
        {
            Task.Run(() =>
            {
                PickAndPlace();
            });
        }


        private void OnWatchdogElapsed(object sender, EventArgs e)
        {
            Task.Run(() =>
            {
                MoveFlexibowl();
            });
        }

        // ----------------------------------------------------- Controls --------------------------------------------------

        private void RefreshControls()
        {
            // update counters
            lbl_Adetected.Text = _counterDetectedA.ToString();
            lbl_Bdetected.Text = _counterDetectedB.ToString();
            lbl_nrPicked.Text = _counterPicked.ToString();
            lbl_nrDiscarded.Text = _counterDiscarded.ToString();

            // update process time
            lbl_cycleTimer.Text = _manager.AutomaticProcess ? $"{(DateTime.Now - _startTime):mm\\:ss}" : "None";

            // change title colors to highlight active process
            lbl_title_setup.BackColor = _manager.AutomaticProcess ? System.Drawing.Color.MediumSeaGreen : System.Drawing.SystemColors.GradientInactiveCaption;
            lbl_title_control.BackColor = _manager.AutomaticProcess ? System.Drawing.Color.MediumSeaGreen : System.Drawing.SystemColors.GradientInactiveCaption;
            lbl_title_coordinates.BackColor = _manager.AutomaticProcess ? System.Drawing.Color.MediumSeaGreen : System.Drawing.SystemColors.GradientInactiveCaption;

            // update start/stop push buttons
            btn_start.BackgroundImage = _manager.StatusScara & _manager.StatusMqttClient & !_manager.AutomaticProcess ? Properties.Resources.playEnabled_2x2_pptx : Properties.Resources.playDisabled_2x2_pptx;
            btn_stop.BackgroundImage = _manager.StatusScara & _manager.StatusMqttClient & _manager.AutomaticProcess ? Properties.Resources.stopEnabled_2x2_pptx : Properties.Resources.stopDisabled_2x2_pptx;
            btn_start.Enabled = !_manager.AutomaticProcess;
            btn_stop.Enabled = _manager.AutomaticProcess;
        }


        private void btn_start_Click(object sender, System.EventArgs e)
        {
            if (!_manager.StatusScara)
            {
                _manager.NonBlockingMessageBox("Please connect SCARA to start automatic proces", "Interlock!", MessageBoxIcon.Hand);
                return;
            }

            if (!_manager.StatusMqttClient)
            {
                _manager.NonBlockingMessageBox("Please connect MQTT to start automatic proces", "Interlock!", MessageBoxIcon.Hand);
                return;
            }

            if (!_manager.StatusCameraManager)
            {
                _manager.NonBlockingMessageBox("Please connect Camera Manager to start automatic proces", "Interlock!", MessageBoxIcon.Hand);
                return;
            }

            _manager.Cameramanager.SwitchToProcessing(_manager.StreamingTopic.Split('/')[1]); // extact camera name from streaming topic
            _manager.Cameramanager.CamerasWorking += BeginProcess; // subscribe to event to start process when cameras are ready
            btn_start.Enabled = false; // disable button until process is started
        }


        private void btn_stop_Click(object sender, EventArgs e)
        {
            InterruptProcess();
        }


        // refresh controls each second if process is active
        private void tmr_process_Tick(object sender, EventArgs e)
        {
            if (_manager.AutomaticProcess)
                RefreshControls();
        }


        // ----------------------------------------------------- Process --------------------------------------------------


        private void BeginProcess()
        {

            _manager.Cameramanager.CamerasWorking -= BeginProcess;
            _manager.StartAutomaticProcess();

            // subscribe to target coordinates topic
            _manager.SubscribeMqttTopic(Properties.Settings.Default.mqtt_client, Properties.Settings.Default.mqtt_topic_scaraTarget);
            _manager.MqttClient.MessageReceived += OnMessageReceived;

            // subscribe to detected events: triggers pick and place
            ScaraTargets.ObjectDetected += OnObjectDetected;

            // 5 seconds; used to move flexibowl if no objects are detected
            _watchdog = new Watchdog(5000);
            _watchdog.Start();
            _watchdog.Elapsed += OnWatchdogElapsed;

            // start timer count
            lbl_actualSelectedCamera.Text = _manager.StreamingTopic;
            _startTime = DateTime.Now;


            // initiate performance report
            //_report = new PerformanceReport(cameraId: "luxonis_01", initTimeMs: 98);
        }

        private void PickAndPlace()
        {

            if (!_manager.AutomaticProcess)
                return;
                
            try
            {
                if (!_scaraIsMoving && ScaraTargets.Backlog > 0) // prevent simultanous picking (physically impossible)
                {

                    if (_flexibowlIsMoving)
                    {
                        _manager.Log("Coordinate have been received (should have not) while flexibowl is moving! Cannot perform picking action", true, false);
                        return;
                    }

                    _scaraIsMoving = true;
                    if (_manager.Cobra600.GripperSuctionStatus)
                    {
                        _manager.Cobra600.ToggleGripperAction(); // turn off suction
                    }

                    Transform3D _locTarget = ScaraTargets.TargetQueueList[0].Transform; // accessing first element to pick

                    Transform3D safeFlexi = new Transform3D(375.0, 15.0, 385.0, 0.0, 180.0, -130.0);
                    Transform3D safeBoxes = new Transform3D(200.0, -450.0, 360.0, 0.0, 180.0, 50.0);
                    Transform3D BoxA = new Transform3D(160, -450.0, 180.0, 0.0, 180.0, 50.0);
                    Transform3D BoxB = new Transform3D(310, -450.0, 180.0, 0.0, 180.0, 50.0);

                    _manager.Cobra600.toggleRingLight();
                    Thread.Sleep(500);
                    _manager.Cobra600.toggleRingLight();

                    // move at safe flexibowl position
                    led_approachFlexibowl.On = true;
                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, safeFlexi, true);
                    led_approachFlexibowl.On = false;

                    // pick object safely
                    led_pick.On = true;
                    Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _locTarget, 20);
                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, _locTarget, true);
                    _manager.Cobra600.ToggleGripperAction(); // turn on suction 
                    Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _locTarget, 20);
                    led_pick.On = false;

                    // move at safe flexibowl position
                    led_approachShuttle.On = true;
                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, safeFlexi, true);

                    // move at safe boxes position
                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, safeBoxes, true);
                    
                    led_approachShuttle.On = false;
                    led_place.On = true;

                    // Place
                    if (ScaraTargets.TargetQueueList[0].Component.Contains("Component A"))
                    {
                        Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, BoxA, 20);
                        Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, BoxA, true);
                        _manager.Cobra600.ToggleGripperAction(); // turn off suction
                        Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, BoxA, 20);
                        _counterDetectedA++;
                    }
                    else if (ScaraTargets.TargetQueueList[0].Component.Contains("Component B"))
                    {
                        Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, BoxB, 20);
                        Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, BoxB, true);
                        _manager.Cobra600.ToggleGripperAction(); // turn off suction
                        Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, BoxB, 20);
                        _counterDetectedB++;
                    }

                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, safeBoxes, true);
                    led_place.On = false;

                    _scaraIsMoving = false;
                    ResetWatchdog();
                    ScaraTargets.PlacingCompleted();
                }

            }
            catch (Exception ex)
            {
                _manager.NonBlockingMessageBox($"Error performing pick-and-plance operation: {ex}", "Error!", MessageBoxIcon.Error);
            }
        }


        private void MoveFlexibowl()
        {
            if (!_manager.AutomaticProcess)
                return;

            if (!_scaraIsMoving)
            {

                _flexibowlIsMoving = true;
                // Flexibowl.Move.Forward(); 

                // TODO: flexibowl shoud be moved slightly to detect new objects
                led_rotate.On = true;
                Thread.Sleep(2000);
                led_rotate.On = false;

                // TODO: track position of pieces, so that when some get under the glass, flipping is possible
                led_flip.On = true;
                Thread.Sleep(500);
                led_flip.On = false;

                _flexibowlIsMoving = false;

            }
            ResetWatchdog();
        }

        private void CompleteProcess(JsonElement pythonMetrics)
        {
            _manager.Log("Automatic process completed.");

            _report.MergePythonMetrics(pythonMetrics);
            _report.ExportToJsonl("performance_report.jsonl");

            _manager.Log("Performance report saved.");

            sendMessageToMontrac();
        }


        private void InterruptProcess()
        {
            // wait until flexibowl is not moving anymore
            while (_flexibowlIsMoving)
            {
                Thread.Sleep(100);
            }

            // wait until scara is not moving anymore
            while (_scaraIsMoving)
            {
                Thread.Sleep(100);
            }

            _watchdog.Elapsed -= OnWatchdogElapsed;
            _watchdog.Stop();

            _manager.MqttClient.MessageReceived -= OnMessageReceived;
            _manager.UnsubscribeMqttTopic(Properties.Settings.Default.mqtt_client, Properties.Settings.Default.mqtt_topic_scaraTarget);

            ScaraTargets.ObjectDetected -= OnObjectDetected;
            ScaraTargets.DropBacklog();

            _manager.Cameramanager.SwitchToStreaming();

            _manager.StopAutomaticProcess();
        }

        // ----------------------------------------------------- Utils --------------------------------------------------


        private void ResetWatchdog()
        {
            _watchdog.Reset();
        }


        private void CountPieceLoading()
        {
            _report.UpdateWorkpieceLoadingCount();
        }
        

        private void sendMessageToMontrac()
        {
            var message = new
            {
                sender = "SCARA",
                receiver = "IPHYSICS",
                command = 8001,
                payload = new
                {
                    blisterReady = true
                }
            };

            //_ = _mqttClient.PublishMessage(_clientId, _mqttTopic, message);
        }

        
    }
}
