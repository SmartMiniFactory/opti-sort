using Ace.Core.Server;
using Crownwood.DotNetMagic.Docking;
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
        

        private void OnTargetReceived(string topic, JsonElement content, int processID)
        {

            if (topic == Properties.Settings.Default.mqtt_topic_scaraTarget & !_flexibowlIsMoving) // topic should be correct, plus flexibowl shold be still to consider coordinates as valid
                ScaraTargets.UpdateTargetTable(content);

            else if (topic == "PythonResultOrSomething...")
            {
                _manager.MqttMessageReceived -= OnTargetReceived;
                //CompleteProcess(message);
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

            _manager.Cameramanager.SwitchToProcessing(_manager.StreamingTopic.Split('/')[1], nud_thresh.Value, nud_polyOut.Value, nud_polyIn.Value); // extact camera name from streaming topic
            _manager.Cameramanager.CamerasWorking += BeginProcess; // subscribe to event to start process when cameras are ready
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
            if (InvokeRequired)
            {
                Invoke(new Action(BeginProcess));
            }

            _manager.Cameramanager.CamerasWorking -= BeginProcess;
            _manager.StartAutomaticProcess();

            // subscribe to target coordinates topic
            _manager.SubscribeMqttTopic(Properties.Settings.Default.mqtt_client, Properties.Settings.Default.mqtt_topic_scaraTarget);
            _manager.MqttMessageReceived += OnTargetReceived;

            // subscribe to detected events: triggers pick and place
            ScaraTargets.ObjectDetected += OnObjectDetected;

            // 5 seconds; used to move flexibowl if no objects are detected
            _watchdog = new Watchdog(5000);
            _watchdog.Start();
            _watchdog.Elapsed += OnWatchdogElapsed;

            // start timer count
            lbl_actualSelectedCamera.Text = _manager.StreamingTopic;
            _startTime = DateTime.Now;

            // setup flexiwbol
            Flexibowl.Set.Rotation.Speed(50);
            Flexibowl.Set.Rotation.Angle(60);

            _counterDetectedA = 0;
            _counterDetectedB = 0;
            _counterPicked = 0;
            _counterDiscarded = 0;

            RefreshControls();

            // initiate performance report
            //_report = new PerformanceReport(cameraId: "luxonis_01", initTimeMs: 98);
        }

        private void PickAndPlace()
        {
            if (!_manager.AutomaticProcess)
                return;
            
            ResetWatchdog(); // trying to avoid the instant where the robot appears to not be moving but it's about to move soon and the flexibowl could move in this very short time

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

                    // move at safe flexibowl position
                    led_approachFlexibowl.On = true;
                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.SafeFlexi, true);
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
                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.SafeFlexi, true);

                    // move at safe boxes position
                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.SafeBoxes, true);
                    
                    led_approachShuttle.On = false;
                    led_place.On = true;

                    // Place
                    if (ScaraTargets.TargetQueueList[0].Component.Contains("Component A"))
                    {
                        Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.BoxPlaceA, 20);
                        Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.BoxPlaceA, true);
                        _manager.Cobra600.ToggleGripperAction(); // turn off suction
                        Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.BoxPlaceA, 20);
                        _counterDetectedA++;
                    }
                    else if (ScaraTargets.TargetQueueList[0].Component.Contains("Component B"))
                    {
                        Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.BoxPlaceB, 20);
                        Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.BoxPlaceB, true);
                        _manager.Cobra600.ToggleGripperAction(); // turn off suction
                        Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.BoxPlaceB, 20);
                        _counterDetectedB++;
                    }
                    _counterPicked++;

                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, _manager.SafeBoxes, true);
                    led_place.On = false;

                    _scaraIsMoving = false;
                    ResetWatchdog();

                    ScaraTargets.PlacingCompleted(); 

                    if (_counterDetectedA == 3 & _counterDetectedB == 3)
                    {
                        InterruptProcess();
                    }

                }

            }
            catch (Exception ex)
            {
                InterruptProcess();
                _manager.NonBlockingMessageBox($"Error performing pick-and-plance operation: {ex}", "PROCESS INTERRUPTED!", MessageBoxIcon.Warning);
            }
        }


        private void MoveFlexibowl()
        {
            if (!_manager.AutomaticProcess)
                return;

            if (ScaraTargets.Backlog == 0 & !_flexibowlIsMoving)
            {

                _flexibowlIsMoving = true;
                led_rotate.On = true;
                bool moved = Flexibowl.Move.Forward();

                if (moved)
                {
                    led_rotate.On = false;
                    _flexibowlIsMoving = false;
                }
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

            _manager.MqttMessageReceived -= OnTargetReceived;
            _manager.UnsubscribeMqttTopic(Properties.Settings.Default.mqtt_client, Properties.Settings.Default.mqtt_topic_scaraTarget);

            ScaraTargets.ObjectDetected -= OnObjectDetected;
            ScaraTargets.DropBacklog();

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
