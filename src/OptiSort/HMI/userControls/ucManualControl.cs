using Ace.Core.Client;
using FlexibowlLibrary;
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

        private void btnFlexibowlFw_Click(object sender, EventArgs e)
        {
            if (!_frmMain.StatusFlexibowl)
            {
                MessageBox.Show("You should connect to the Flexibowl first");
                return;
            }

            Flexibowl.Move.Forward();
            _frmMain.Log("Flexibowl: forward command sent", false, false);

        }

        private void btnFlexibowlBw_Click(object sender, EventArgs e)
        {
            if (!_frmMain.StatusFlexibowl)
            {
                MessageBox.Show("You should connect to the Flexibowl first");
                return;
            }

            Flexibowl.Move.Backward();
            _frmMain.Log("Flexibowl: backward command sent", false, false);
        }

        private void btnFlexibowlFlip_Click(object sender, EventArgs e)
        {
            if (!_frmMain.StatusFlexibowl)
            {
                MessageBox.Show("You should connect to the Flexibowl first");
                return;
            }

            Flexibowl.Move.Flip(1); // TODO: 1 or 2?
            _frmMain.Log("Flexibowl: piston 1 activated (flip)", false, false);

        }

        private void btnFlexibowlShake_Click(object sender, EventArgs e)
        {
            if (!_frmMain.StatusFlexibowl)
            {
                MessageBox.Show("You should connect to the Flexibowl first");
                return;
            }

            Flexibowl.Move.Shake();
            _frmMain.Log("Flexibowl: shake command sent", false, false);

        }

        private void btnFaultReset_Click(object sender, EventArgs e)
        {
            Flexibowl.resetFault();
        }

        private void btn_lensCalibration_Click(object sender, EventArgs e)
        {
            ucLensDistortionCalibration ucLensDistortionCalibration = new ucLensDistortionCalibration();
            _frmMain.AddNewUc(ucLensDistortionCalibration);
        }
    }
}
