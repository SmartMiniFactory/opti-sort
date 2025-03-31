using Ace.UIBuilder.Client.Controls.Tools.WindowsForms;
using ActiproSoftware.SyntaxEditor;
using CobraLibrary;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using static OptiSort.Program;
using static System.Net.Mime.MediaTypeNames;

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
    }
}
