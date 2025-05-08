using Ace.Core.Server;
using Ace.Core.Server.Access;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using static Crownwood.DotNetMagic.Docking.RedockerContent;

namespace OptiSort.userControls
{
    internal partial class ucFixedCoordinatesDefinition : UserControl
    {

        optisort_mgr _manager;
        private bool isModified = false;

        public ucFixedCoordinatesDefinition(optisort_mgr manager)
        {
            _manager = manager;
            InitializeComponent();

            btn_save.Enabled = false;

            LoadValues();
            WireNumericUpDowns();
        }

        private void LoadValues()
        {
            nud_gridPick_x.Value        = (decimal)_manager.GridPick.DX;
            nud_gridPick_y.Value        = (decimal)_manager.GridPick.DY;
            nud_gridPick_z.Value        = (decimal)_manager.GridPick.DZ;
            nud_gridPick_roll.Value     = (decimal)_manager.GridPick.Roll;
            nud_gridPick_pitch.Value    = (decimal)_manager.GridPick.Pitch;
            nud_gridPick_yaw.Value      = (decimal)_manager.GridPick.Yaw;

            nud_gridPlace_x.Value       = (decimal)_manager.GridPlace.DX;
            nud_gridPlace_y.Value       = (decimal)_manager.GridPlace.DY;
            nud_gridPlace_z.Value       = (decimal)_manager.GridPlace.DZ;
            nud_gridPlace_roll.Value    = (decimal)_manager.GridPlace.Roll;
            nud_gridPlace_pitch.Value   = (decimal)_manager.GridPlace.Pitch;
            nud_gridPlace_yaw.Value     = (decimal)_manager.GridPlace.Yaw;

            nud_safeFlexi_x.Value      = (decimal)_manager.SafeFlexi.DX;
            nud_safeFlexi_y.Value      = (decimal)_manager.SafeFlexi.DY;
            nud_safeFlexi_z.Value      = (decimal)_manager.SafeFlexi.DZ;
            nud_safeFlexi_roll.Value   = (decimal)_manager.SafeFlexi.Roll;
            nud_safeFlexi_pitch.Value  = (decimal)_manager.SafeFlexi.Pitch;
            nud_safeFlexi_yaw.Value    = (decimal)_manager.SafeFlexi.Yaw;

            nud_boxPlaceA_x.Value       = (decimal)_manager.BoxPlaceA.DX;
            nud_boxPlaceA_y.Value       = (decimal)_manager.BoxPlaceA.DY;
            nud_boxPlaceA_z.Value       = (decimal)_manager.BoxPlaceA.DZ;
            nud_boxPlaceA_roll.Value    = (decimal)_manager.BoxPlaceA.Roll;
            nud_boxPlaceA_pitch.Value   = (decimal)_manager.BoxPlaceA.Pitch;
            nud_boxPlaceA_yaw.Value     = (decimal)_manager.BoxPlaceA.Yaw;

            nud_boxPlaceB_x.Value       = (decimal)_manager.BoxPlaceB.DX;
            nud_boxPlaceB_y.Value       = (decimal)_manager.BoxPlaceB.DY;
            nud_boxPlaceB_z.Value       = (decimal)_manager.BoxPlaceB.DZ;
            nud_boxPlaceB_roll.Value    = (decimal)_manager.BoxPlaceB.Roll;
            nud_boxPlaceB_pitch.Value   = (decimal)_manager.BoxPlaceB.Pitch;
            nud_boxPlaceB_yaw.Value     = (decimal)_manager.BoxPlaceB.Yaw;

            nud_safeBoxes_x.Value       = (decimal)_manager.SafeBoxes.DX;
            nud_safeBoxes_y.Value       = (decimal)_manager.SafeBoxes.DY;
            nud_safeBoxes_z.Value       = (decimal)_manager.SafeBoxes.DZ;
            nud_safeBoxes_roll.Value    = (decimal)_manager.SafeBoxes.Roll;
            nud_safeBoxes_pitch.Value   = (decimal)_manager.SafeBoxes.Pitch;
            nud_safeBoxes_yaw.Value     = (decimal)_manager.SafeBoxes.Yaw;
        }


        // Pair controls with method that verifies their new values
        private void WireNumericUpDowns()
        {
            foreach (Control ctrl in this.Controls)
            {
                if (ctrl is NumericUpDown nud)
                {
                    nud.ValueChanged += NumericUpDown_ValueChanged;
                }
            }

            // If your NumericUpDowns are inside groupboxes/panels, recurse:
            WireNumericUpDownsRecursive(this);
        }

        private void WireNumericUpDownsRecursive(Control parent)
        {
            foreach (Control ctrl in parent.Controls)
            {
                if (ctrl is NumericUpDown nud)
                {
                    nud.ValueChanged += NumericUpDown_ValueChanged;
                }
                else
                {
                    WireNumericUpDownsRecursive(ctrl);
                }
            }
        }

        // As soon as one gets updated, enable start button and track modification
        private void NumericUpDown_ValueChanged(object sender, EventArgs e)
        {
            if (!isModified)
            {
                isModified = true;
                btn_save.Enabled = true;
            }
        }


        private void btn_restoreDefaults_Click(object sender, EventArgs e)
        {
            
        }

        private void btn_save_Click(object sender, EventArgs e)
        {
            //_manager.UpdateCoordinate("GridPick", BuildTransformFromNuds(nud_gridPick_x, nud_gridPick_y, nud_gridPick_z, nud_gridPick_yaw, nud_gridPick_pitch, nud_gridPick_roll));
            //_manager.UpdateCoordinate("GridPlace", BuildTransformFromNuds(nud_gridPlace_x, nud_gridPlace_y, nud_gridPlace_z, nud_gridPlace_yaw, nud_gridPlace_pitch, nud_gridPlace_roll));
            //_manager.UpdateCoordinate("SafeFlexi", BuildTransformFromNuds(nud_safeFlexi_x, nud_safeFlexi_y, nud_safeFlexi_z, nud_safeFlexi_yaw, nud_safeFlexi_pitch, nud_safeFlexi_roll));
            //_manager.UpdateCoordinate("BoxPlaceA", BuildTransformFromNuds(nud_boxPlaceA_x, nud_boxPlaceA_y, nud_boxPlaceA_z, nud_boxPlaceA_yaw, nud_boxPlaceA_pitch, nud_boxPlaceA_roll));
            //_manager.UpdateCoordinate("BoxPlaceB", BuildTransformFromNuds(nud_boxPlaceB_x, nud_boxPlaceB_y, nud_boxPlaceB_z, nud_boxPlaceB_yaw, nud_boxPlaceB_pitch, nud_boxPlaceB_roll));
            //_manager.UpdateCoordinate("SafeBoxes", BuildTransformFromNuds(nud_safeBoxes_x, nud_safeBoxes_y, nud_safeBoxes_z, nud_safeBoxes_yaw, nud_safeBoxes_pitch, nud_safeBoxes_roll));

            isModified = false;
            btn_save.Enabled = false;
        }

        private Transform3D BuildTransformFromNuds(NumericUpDown x, NumericUpDown y, NumericUpDown z, NumericUpDown yaw, NumericUpDown pitch, NumericUpDown roll)
        {
            return new Transform3D(
                (double)x.Value,
                (double)y.Value,
                (double)z.Value,
                (double)yaw.Value,
                (double)pitch.Value,
                (double)roll.Value
            );
        }



        // ------------------------------------------------

        private void btn_help_coordinates_Click(object sender, EventArgs e)
        {
            Form popupForm = new Form();
            popupForm.Text = "Coordinates Help";
            popupForm.Size = new Size(400, 400); // Adjust size as needed

            PictureBox pictureBox = new PictureBox();
            pictureBox.Dock = DockStyle.Fill;
            pictureBox.SizeMode = PictureBoxSizeMode.Zoom;

            pictureBox.Image = Image.FromFile(Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\Resources\Roll-Pitch-Yaw.jpg")));

            popupForm.Controls.Add(pictureBox);
            popupForm.StartPosition = FormStartPosition.CenterParent;
            popupForm.ShowDialog(); // Modal popup
        }

        private void btn_help_gridPick_Click(object sender, EventArgs e)
        {
            _manager.NonBlockingMessageBox("This is the coordinates of the calibration grid pick position.\n\n" +
                "The calibration grid should be placed somewhere near the flexibowl, stored in a 3D printed support.\n\n" +
                "This coordinate represent the calibration grid's center and MUST BE TAKEN AS PRECISE AS POSSIBLE!!\n" + 
                "Even a few millimeters of unprecision can drive great localization imprecisions from the cameras' detection algorithm.\n\n" +
                "Define the point by placing in contact the gripper with the grid, because from this position the robot will be activating suction.\n\n" + 
                "NOTE: do not move the support much lower than it's original position: COLLISION RISK!", "INFO", MessageBoxIcon.Information);
        }

        private void btn_help_gridPlace_Click(object sender, EventArgs e)
        {
            _manager.NonBlockingMessageBox("This is the coordinates of the calibration grid place position.\n\n" +
                "The robot will be commanded to place the grid in this position and the cameras' will be informed of the same information.\n\n" +
                "This data must be consistent between both robot and cameras to calculate precise roto-translation matrices.", "INFO", MessageBoxIcon.Information);
        }

        private void btn_help_safeFlexi_Click(object sender, EventArgs e)
        {
            _manager.NonBlockingMessageBox("This coordinate represents a safe position where the scara never hits the flexibowl (main obstacle to avoid).\n\n" +
                "This should be defined somewhere above the left-hand-side of the flexibowl's outer perimeter.\n\n" +
                "This position is called each time the robot is supposed to travel between picking and placing positions, so to avoid collisions.", "INFO", MessageBoxIcon.Information);
        }

        private void btn_help_boxPlaceA_Click(object sender, EventArgs e)
        {
            _manager.NonBlockingMessageBox("This coordinate represents the placing position of box A.\n\n" +
                "Scara will relese suction at this precise point, for A components.\n\n", "INFO", MessageBoxIcon.Information);
        }

        private void btn_help_boxPlaceB_Click(object sender, EventArgs e)
        {
            _manager.NonBlockingMessageBox("This coordinate represents the placing position of box B.\n\n" +
                "Scara will relese suction at this precise point, for B components.\n\n", "INFO", MessageBoxIcon.Information);
        }

        private void btn_help_safeBoxes_Click(object sender, EventArgs e)
        {
            _manager.NonBlockingMessageBox("This coordinate represents a safe position called in between placing position and flexibowl reach.\n\n" +
                "You cannot define the Z-value of this coordinate because it reflects the flexibowl's safe position' value. This is to force straight horizontal travel.\n\n", "INFO", MessageBoxIcon.Information);
        }

        

        
    }
}
