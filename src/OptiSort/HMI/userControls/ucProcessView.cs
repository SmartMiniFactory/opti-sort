using OptiSort.Classes;
using System.Text.Json;
using System.Windows.Forms;


namespace OptiSort.userControls
{
    public partial class ucProcessView : UserControl
    {

        public bool AutomaticProcess { get; set; }

        private optisort_mgr _manager;
        private ucScaraTargets _ucScaraTargets;
        private PerformanceReport _report;

        internal ucProcessView(optisort_mgr manager)
        {
            InitializeComponent();
            _manager = manager;

            // init scara dgv
            _ucScaraTargets = new ucScaraTargets(_manager); // using log function
            _ucScaraTargets.Dock = DockStyle.Fill;
            pnlScara.Controls.Clear();
            pnlScara.Controls.Add(_ucScaraTargets);

            // TODO: revise this shit
            _manager.MqttClient.MessageReceived += _ucScaraTargets.OnMessageReceived; // enable MQTT messages to trigger the user control

            RefreshControls();

        }

        private void RefreshControls()
        {
            
        }


        private void btn_start_Click(object sender, System.EventArgs e)
        {
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
