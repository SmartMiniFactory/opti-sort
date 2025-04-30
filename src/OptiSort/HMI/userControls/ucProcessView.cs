using Ace.Core.Server;
using FlexibowlLibrary;
using OptiSort.Classes;
using System;
using System.ComponentModel;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;


namespace OptiSort.userControls
{
    public partial class ucProcessView : UserControl
    {

        public bool AutomaticProcess { get; set; }

        private optisort_mgr _manager;
        private ucScaraTargets ScaraTargets;
        private PerformanceReport _report;
        private Watchdog _watchdog;
        private bool _robotIsMoving = false;

        internal ucProcessView(optisort_mgr manager)
        {
            InitializeComponent();
            _manager = manager;

            // init scara dgv
            ScaraTargets = new ucScaraTargets(_manager); // using log function
            ScaraTargets.Dock = DockStyle.Fill;

            pnlScara.Controls.Clear();
            pnlScara.Controls.Add(ScaraTargets);

            _watchdog = new Watchdog(5000); // 5 seconds; used to move flexibowl if no objects are detected

            RefreshControls();

        }

        private void RefreshControls()
        {

        }

        private void OnMessageReceived(string topic, JsonElement message)
        {
            if (topic == Properties.Settings.Default.mqtt_topic_scaraTarget)
                ScaraTargets.UpdateTargetTable(message);

            else if (topic == "PythonResultOrSomething...")
            {
                _manager.MqttClient.MessageReceived -= OnMessageReceived;
                CompleteProcess(message);
            }
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

            _manager.SubscribeMqttTopic(Properties.Settings.Default.mqtt_client, Properties.Settings.Default.mqtt_topic_scaraTarget);
            _manager.MqttClient.MessageReceived += OnMessageReceived;

            _watchdog.Start();
            _watchdog.Elapsed += MoveFlexibowl;

            ScaraTargets.ObjectDetected += OnObjectDetected;

            //_report = new PerformanceReport(cameraId: "luxonis_01", initTimeMs: 98);
            _manager.Log("Automatic process started...");
        }


        private void OnObjectDetected()
        {
            Task.Run(() =>
            {
                PickAndPlace();
            });
        }

        private void PickAndPlace()
        {
            try
            {
                if (!_robotIsMoving && ScaraTargets.Backlog > 0) // prevent simultanous picking (physically impossible)
                {
                    _robotIsMoving = true;
                    if (_manager.Cobra600.GripperSuctionStatus)
                    {
                        _manager.Cobra600.ToggleGripperAction(); // turn off suction
                    }

                    Transform3D _locTarget = ScaraTargets.TargetQueueList[0].Transform; // accessing first element to pick

                    Transform3D safeFlexi = new Transform3D(375.0, 15.0, 385.0, 0.0, 180.0, -130.0);
                    Transform3D safeBoxes = new Transform3D(200.0, -450.0, 360.0, 0.0, 180.0, 50.0);
                    Transform3D BoxA = new Transform3D(160, -450.0, 180.0, 0.0, 180.0, 50.0);
                    Transform3D BoxB = new Transform3D(310, -450.0, 180.0, 0.0, 180.0, 50.0);


                    // move at safe flexibowl position
                    _manager.Cobra600.toggleRingLight();
                    Thread.Sleep(500);
                    _manager.Cobra600.toggleRingLight();
                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, safeFlexi, true);

                    // pick object safely
                    Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _locTarget, 20);
                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, _locTarget, true);
                    _manager.Cobra600.ToggleGripperAction(); // turn on suction 
                    Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, _locTarget, 20);

                    // move at safe flexibowl position
                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, safeFlexi, true);

                    // move at safe boxes position
                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, safeBoxes, true);

                    // Place
                    if (ScaraTargets.TargetQueueList[0].Component.Contains("Component A"))
                    {
                        Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, BoxA, 20);
                        Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, BoxA, true);
                        _manager.Cobra600.ToggleGripperAction(); // turn off suction
                        Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, BoxA, 20);
                    }
                    else if (ScaraTargets.TargetQueueList[0].Component.Contains("Component B"))
                    {
                        Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, BoxB, 20);
                        Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, BoxB, true);
                        _manager.Cobra600.ToggleGripperAction(); // turn off suction
                        Cobra600.Motion.Approach(_manager.Cobra600.Server, _manager.Cobra600.Robot, BoxB, 20);
                    }

                    Cobra600.Motion.CartesianMove(_manager.Cobra600.Server, _manager.Cobra600.Robot, safeBoxes, true);

                    _robotIsMoving = false;
                    ResetWatchdog();
                    ScaraTargets.PlacingCompleted();
                }

            }
            catch (Exception ex)
            {
                _manager.NonBlockingMessageBox($"Error performing pick-and-plance operation: {ex}", "Error!", MessageBoxIcon.Error);
            }
        }

        private void MoveFlexibowl(object sender, EventArgs e)
        {
            if (!_robotIsMoving)
            {
                _manager.Log("Flexibowl moving forward due to unrecognition...");
                // Flexibowl.Move.Forward(); 
            }
            ResetWatchdog();
        }

        private void ResetWatchdog()
        {
            _manager.Log("Watchdog reset.");
            _watchdog.Reset();
        }

        private void CountPieceLoading()
        {
            _report.UpdateWorkpieceLoadingCount();
        }

        private void CompleteProcess(JsonElement pythonMetrics)
        {
            _manager.Log("Automatic process completed.");

            _report.MergePythonMetrics(pythonMetrics);
            _report.ExportToJsonl("performance_report.jsonl");

            _manager.Log("Performance report saved.");

            sendMessageToMontrac();
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
