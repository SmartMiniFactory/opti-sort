using OptiSort.Classes;
using System.Text.Json;
using System.Windows.Forms;


namespace OptiSort.userControls
{
    public partial class ucProcessView : UserControl
    {

        public bool AutomaticProcess { get; set; }

        private optisort_mgr _manager;
        private ucScaraTargets ScaraTargets;
        private PerformanceReport _report;

        internal ucProcessView(optisort_mgr manager)
        {
            InitializeComponent();
            _manager = manager;

            // init scara dgv
            ScaraTargets = new ucScaraTargets(_manager); // using log function
            ScaraTargets.Dock = DockStyle.Fill;

            pnlScara.Controls.Clear();
            pnlScara.Controls.Add(ScaraTargets);

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

            if (!_manager.StatusFlexibowl)
            {
                _manager.NonBlockingMessageBox("Please connect FLEXIBOWL to start automatic proces", "Interlock!", MessageBoxIcon.Hand);
                return;
            }

            if (!_manager.StatusMqttClient)
            {
                _manager.NonBlockingMessageBox("Please connect CAMERA MANAGER to start automatic proces", "Interlock!", MessageBoxIcon.Hand);
                return;
            }

            _manager.SubscribeMqttTopic(Properties.Settings.Default.mqtt_client, Properties.Settings.Default.mqtt_topic_scaraTarget);
            _manager.MqttClient.MessageReceived += OnMessageReceived;

            _report = new PerformanceReport(cameraId: "luxonis_01", initTimeMs: 98);
            _manager.Log("Automatic process started...");
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
        }
    }
}
