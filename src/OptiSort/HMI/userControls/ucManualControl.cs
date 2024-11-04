using Ace.Core.Client;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace OptiSort.userControls
{
    public partial class ucManualControl : UserControl
    {

        private frmMain _frmMain;

        public ucManualControl(frmMain frmMain)
        {
            InitializeComponent();
            _frmMain = frmMain;
        }

        private void btnScaraJog_Click(object sender, EventArgs e)
        {
            if (!_frmMain.StatusScara)
            {
                MessageBox.Show("You should connect to a robot first");
                return;
            }

            _frmMain.Cobra600.OpenJobControl(this);
        }
    }
}
