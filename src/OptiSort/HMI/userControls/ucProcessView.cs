using System.Windows.Forms;


namespace OptiSort.userControls
{
    public partial class ucProcessView : UserControl
    {

        public bool AutomaticProcess { get; set; }

        private optisort_mgr _manager;
        private ucScaraTargets _ucScaraTargets;

        internal ucProcessView(optisort_mgr manager)
        {
            InitializeComponent();
            _manager = manager;

            // init scara dgv
            _ucScaraTargets = new ucScaraTargets(_manager); // using log function
            _ucScaraTargets.Dock = DockStyle.Fill;
            pnlScara.Controls.Clear();
            pnlScara.Controls.Add(_ucScaraTargets);
            _manager.MqttClient.MessageReceived += _ucScaraTargets.OnMessageReceived; // enable MQTT messages to trigger the user control

            RefreshControls();

        }

        private void RefreshControls()
        {
            
        }

        private void btn_start_Click(object sender, System.EventArgs e)
        {

        }
    }
}
