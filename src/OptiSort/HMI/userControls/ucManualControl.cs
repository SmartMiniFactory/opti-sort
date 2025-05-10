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

            _manager.PropertyChanged += PropertyChanged;

            RefreshButtons();
        }

        private void PropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(_manager.StatusScara) && _manager.StatusScara)
                RefreshButtons();
        }

        private void RefreshButtons()
        {
            if (_manager.StatusScara && !_manager.StatusScaraEmulation)
            {
                if (_manager.Cobra600.RingLightStatus)
                    btnLight.Text = "Turn Ring Light Off";
                else
                    btnLight.Text = "Turn Ring Light On";
            }
            else
                btnLight.Text = "Toggle Ring Light";

            if (_manager.StatusScara && !_manager.StatusScaraEmulation)
            {
                if (_manager.Cobra600.GripperSuctionStatus)
                    btn_gripper.Text = "Turn Suction Off";
                else
                    btn_gripper.Text = "Turn Suction On";
            }
            else
                btn_gripper.Text = "Toggle Gripper Suction";
        }

        private void btnScaraJog_Click(object sender, EventArgs e)
        {
            if (!_manager.StatusScara)
            {
                _manager.NonBlockingMessageBox("You should connect to a robot first", "Interlock!", MessageBoxIcon.Hand);
                return;
            }

            _manager.Cobra600.OpenJobControl(this);
        }

        private void btnFlexibowlFw_Click(object sender, EventArgs e)
        {
            if (!_manager.StatusFlexibowl)
            {
                _manager.NonBlockingMessageBox("You should connect to the Flexibowl first", "Intelock!", MessageBoxIcon.Hand);
                return;
            }

            Flexibowl.Move.Forward();
            _manager.Log("Flexibowl: forward command sent", false, false);

        }

        private void btnFlexibowlBw_Click(object sender, EventArgs e)
        {
            if (!_manager.StatusFlexibowl)
            {
                _manager.NonBlockingMessageBox("You should connect to the Flexibowl first", "Interlock!", MessageBoxIcon.Hand);
                return;
            }

            Flexibowl.Move.Shake();
            _manager.Log("Flexibowl: backward command sent", false, false);
        }

        private void btnFlexibowlFlip_Click(object sender, EventArgs e)
        {
            if (!_manager.StatusFlexibowl)
            {
                _manager.NonBlockingMessageBox("You should connect to the Flexibowl first", "Interlock!", MessageBoxIcon.Hand);
                return;
            }

            Flexibowl.Move.Flip(1); // TODO: 1 or 2?
            _manager.Log("Flexibowl: piston 1 activated (flip)", false, false);

        }

        private void btnFlexibowlShake_Click(object sender, EventArgs e)
        {
            if (!_manager.StatusFlexibowl)
            {
                _manager.NonBlockingMessageBox("You should connect to the Flexibowl first", "Interlock!", MessageBoxIcon.Hand);
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
            if (!_manager.StatusScara)
            {
                _manager.NonBlockingMessageBox("You should connect to the Scara robot first", "Interlock!", MessageBoxIcon.Hand);
                return;
            }
            _manager.Cobra600.toggleRingLight();
            RefreshButtons();
        }

        private void btn_gripper_Click(object sender, EventArgs e)
        {
            if (!_manager.StatusScara)
            {
                _manager.NonBlockingMessageBox("You should connect to the REAL Scara robot first", "Interlock!", MessageBoxIcon.Hand);
                return;
            }
            _manager.Cobra600.ToggleGripperAction();
            RefreshButtons();
        }

        private void btn_fixedCoord_Click(object sender, EventArgs e)
        {
            ucFixedCoordinatesDefinition ucFixedCoordinatesDefinition = new ucFixedCoordinatesDefinition(_manager);
            _manager.RequestNewUcLoading(ucFixedCoordinatesDefinition);
        }
    }
}
