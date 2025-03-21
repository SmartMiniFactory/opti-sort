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

        private optisort_mgr _manager;

        internal ucManualControl(optisort_mgr manager)
        {
            InitializeComponent();
            _manager = manager;
        }

        private void btnScaraJog_Click(object sender, EventArgs e)
        {
            if (!_manager.StatusScara)
            {
                MessageBox.Show("You should connect to a robot first");
                return;
            }

            _manager.Cobra600.OpenJobControl(this);
        }

        private void btnFlexibowlFw_Click(object sender, EventArgs e)
        {
            if (!_manager.StatusFlexibowl)
            {
                MessageBox.Show("You should connect to the Flexibowl first");
                return;
            }

            Flexibowl.Move.Forward();
            _manager.Log("Flexibowl: forward command sent", false, false);

        }

        private void btnFlexibowlBw_Click(object sender, EventArgs e)
        {
            if (!_manager.StatusFlexibowl)
            {
                MessageBox.Show("You should connect to the Flexibowl first");
                return;
            }

            Flexibowl.Move.Backward();
            _manager.Log("Flexibowl: backward command sent", false, false);
        }

        private void btnFlexibowlFlip_Click(object sender, EventArgs e)
        {
            if (!_manager.StatusFlexibowl)
            {
                MessageBox.Show("You should connect to the Flexibowl first");
                return;
            }

            Flexibowl.Move.Flip(1); // TODO: 1 or 2?
            _manager.Log("Flexibowl: piston 1 activated (flip)", false, false);

        }

        private void btnFlexibowlShake_Click(object sender, EventArgs e)
        {
            if (!_manager.StatusFlexibowl)
            {
                MessageBox.Show("You should connect to the Flexibowl first");
                return;
            }

            Flexibowl.Move.Shake();
            _manager.Log("Flexibowl: shake command sent", false, false);

        }

        private void btnFaultReset_Click(object sender, EventArgs e)
        {
            Flexibowl.resetFault();
        }

        private void btn_lensCalibration_Click(object sender, EventArgs e)
        {
            ucCameraLensCalibration ucLensDistortionCalibration = new ucCameraLensCalibration(_manager);
            _manager.RequestNewUcLoading(ucLensDistortionCalibration);
        }

        private void btn_coordinateCalibration_Click(object sender, EventArgs e)
        {
            ucCoordinateReferenceFrame ucCoordinateReferenceFrame = new ucCoordinateReferenceFrame(_manager);
            _manager.RequestNewUcLoading(ucCoordinateReferenceFrame);
        }

        private void btnLight_Click(object sender, EventArgs e)
        {
            _manager.Cobra600.ToggleDigitalOutput(99);
        }
    }
}
