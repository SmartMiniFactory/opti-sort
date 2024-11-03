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

namespace OptiSort.userControls
{
    public partial class ucProcessView : UserControl
    {

        private frmMain _frmMain;
        private ucScaraTargets _ucScaraTargets;

        public ucProcessView(frmMain frmMain)
        {
            InitializeComponent();
            _frmMain = frmMain;

            // init scara dgv
            _ucScaraTargets = new ucScaraTargets(_frmMain); // using log function
            _ucScaraTargets.Dock = DockStyle.Fill;
            pnlScara.Controls.Clear();
            pnlScara.Controls.Add(_ucScaraTargets);
            _frmMain.MqttClient.MessageReceived += _ucScaraTargets.OnMessageReceived; // enable MQTT messages to trigger the user control
            _frmMain.Log("Scara targets dgv attached to MQTT messages");

            // init flexibowl
            ucFlexibowl ucFlexibowl = new ucFlexibowl();
            ucFlexibowl.Dock = DockStyle.Fill;
            pnlFlexibowl.Controls.Clear();
            pnlFlexibowl.Controls.Add(ucFlexibowl);
        }

    }
}
