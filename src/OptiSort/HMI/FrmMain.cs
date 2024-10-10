using System.Linq;
using System.Windows.Forms;
using CobraLibrary;
using static OptiSort.Program;
using System.Collections.Generic;
using System.Threading.Tasks;
using OptiSort.userControls;

namespace OptiSort
{
    public partial class frmMain : Form
    {

        /// <summary
        /// single MQTT broker where at least two topics are present: 
        /// 1. image streaming (one per camera, incoming); python file
        /// 2. scara's pick location streaming (incoming); same python file that operates with openCV on image
        /// 
        /// UDP protocol for flexibowl's communication (outgoing); flexibowl library
        /// 
        /// TCP connection with scara robot (in/out); cobra library
        /// </summary>
        /// 

        public MQTT _mqttClient;
        public string _mqttClientName = "OptiSort";
        public bool _clientCreated = false;

        public frmMain()
        {
            InitializeComponent();

            InitializeMQTTClient();

            ucProcessView ucProcessView = new ucProcessView(this);
            ucProcessView.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Clear();
            pnlCurrentUc.Controls.Add(ucProcessView);

            btnProcess.Enabled = false;
            btnManual.Enabled = true;
            btnConfig.Enabled = true;
        }

        private async void InitializeMQTTClient()
        {
            _mqttClient = new MQTT();

            Task<bool> client = _mqttClient.CreateClient(_mqttClientName);
            _clientCreated = await client; 

            if (_clientCreated)
                Log($"MQTT client '{_mqttClientName}' created");
            else
                Log($"Failed to create '{_mqttClientName}' MQTT client");
        }


        public void Log(string msg)
        {
            lstLog.Items.Add(msg);
            lstLog.TopIndex = lstLog.Items.Count - 1; // showing last row
        }


        private void CleanPnlCurrentUc()
        {
            Control previousControl = pnlCurrentUc.Controls.Cast<Control>().FirstOrDefault(c => c.Dock == DockStyle.Fill); // get control docked in pnlCurrentUc
            pnlCurrentUc.Controls.Remove(previousControl);
            pnlCurrentUc.Controls.Clear();

            previousControl.Dispose();
            // TODO: detach events
        }

        
        private void btnProcess_Click(object sender, System.EventArgs e)
        {
            CleanPnlCurrentUc();
                        
            ucProcessView ucProcessView = new ucProcessView(this);
            ucProcessView.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Add(ucProcessView);

            btnProcess.Enabled = false;
            btnManual.Enabled = true;
            btnConfig.Enabled = true;

            Log("Switched to automatic process view");
        }

        private void btnManual_Click(object sender, System.EventArgs e)
        {
            CleanPnlCurrentUc();

            ucManualControl ucManualControl = new ucManualControl();
            ucManualControl.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Add(ucManualControl);

            btnProcess.Enabled = true;
            btnManual.Enabled = false;
            btnConfig.Enabled = true;

            Log("Switched to manual control view");
        }

        private void btnConfig_Click(object sender, System.EventArgs e)
        {
            CleanPnlCurrentUc();

            ucConfiguration ucConfiguration = new ucConfiguration();
            ucConfiguration.Dock = DockStyle.Fill;
            pnlCurrentUc.Controls.Add(ucConfiguration);

            btnProcess.Enabled = true;
            btnManual.Enabled = true;
            btnConfig.Enabled = false;

            Log("Switched to configuration view");
        }
    }
}
