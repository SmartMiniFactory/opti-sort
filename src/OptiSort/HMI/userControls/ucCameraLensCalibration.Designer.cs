namespace OptiSort.userControls
{
    partial class ucCameraLensCalibration
    {
        /// <summary> 
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary> 
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Component Designer generated code

        /// <summary> 
        /// Required method for Designer support - do not modify 
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.lblNote = new System.Windows.Forms.Label();
            this.tbl_images = new System.Windows.Forms.TableLayoutPanel();
            this.lbl_baslerCalibTimestamp = new System.Windows.Forms.Label();
            this.lbl_luxonisCalibTimestamp = new System.Windows.Forms.Label();
            this.lbl_camera3 = new System.Windows.Forms.Label();
            this.lbl_camera2 = new System.Windows.Forms.Label();
            this.lbl_camera1 = new System.Windows.Forms.Label();
            this.flp_ids = new System.Windows.Forms.FlowLayoutPanel();
            this.flp_luxonis = new System.Windows.Forms.FlowLayoutPanel();
            this.flp_basler = new System.Windows.Forms.FlowLayoutPanel();
            this.lbl_idsCalibTimestamp = new System.Windows.Forms.Label();
            this.btn_acquire = new System.Windows.Forms.Button();
            this.lbl_shots = new System.Windows.Forms.Label();
            this.btn_calibrate = new System.Windows.Forms.Button();
            this.tbl_controls = new System.Windows.Forms.TableLayoutPanel();
            this.btn_home = new System.Windows.Forms.Button();
            this.btn_clear = new System.Windows.Forms.Button();
            this.tbl_images.SuspendLayout();
            this.tbl_controls.SuspendLayout();
            this.SuspendLayout();
            // 
            // lblNote
            // 
            this.lblNote.AutoSize = true;
            this.tbl_images.SetColumnSpan(this.lblNote, 3);
            this.lblNote.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lblNote.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblNote.Location = new System.Drawing.Point(5, 2);
            this.lblNote.Name = "lblNote";
            this.lblNote.Size = new System.Drawing.Size(1285, 31);
            this.lblNote.TabIndex = 0;
            this.lblNote.Text = "Procedure: position the calibration grid in different orientations and take some " +
    "pictures!";
            this.lblNote.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tbl_images
            // 
            this.tbl_images.CellBorderStyle = System.Windows.Forms.TableLayoutPanelCellBorderStyle.Inset;
            this.tbl_images.ColumnCount = 3;
            this.tbl_images.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 33.33332F));
            this.tbl_images.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 33.33334F));
            this.tbl_images.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 33.33334F));
            this.tbl_images.Controls.Add(this.lbl_baslerCalibTimestamp, 2, 2);
            this.tbl_images.Controls.Add(this.lbl_luxonisCalibTimestamp, 1, 2);
            this.tbl_images.Controls.Add(this.lblNote, 0, 0);
            this.tbl_images.Controls.Add(this.lbl_camera3, 2, 1);
            this.tbl_images.Controls.Add(this.lbl_camera2, 1, 1);
            this.tbl_images.Controls.Add(this.lbl_camera1, 0, 1);
            this.tbl_images.Controls.Add(this.flp_ids, 0, 3);
            this.tbl_images.Controls.Add(this.flp_luxonis, 1, 3);
            this.tbl_images.Controls.Add(this.flp_basler, 2, 3);
            this.tbl_images.Controls.Add(this.lbl_idsCalibTimestamp, 0, 2);
            this.tbl_images.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tbl_images.Location = new System.Drawing.Point(0, 0);
            this.tbl_images.Name = "tbl_images";
            this.tbl_images.RowCount = 4;
            this.tbl_images.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 5F));
            this.tbl_images.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 5F));
            this.tbl_images.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 5F));
            this.tbl_images.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 85F));
            this.tbl_images.Size = new System.Drawing.Size(1295, 631);
            this.tbl_images.TabIndex = 1;
            // 
            // lbl_baslerCalibTimestamp
            // 
            this.lbl_baslerCalibTimestamp.AutoSize = true;
            this.lbl_baslerCalibTimestamp.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_baslerCalibTimestamp.Font = new System.Drawing.Font("Segoe UI Light", 10F, System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_baslerCalibTimestamp.Location = new System.Drawing.Point(866, 68);
            this.lbl_baslerCalibTimestamp.Name = "lbl_baslerCalibTimestamp";
            this.lbl_baslerCalibTimestamp.Size = new System.Drawing.Size(424, 31);
            this.lbl_baslerCalibTimestamp.TabIndex = 8;
            this.lbl_baslerCalibTimestamp.Text = "Last calibration: dd/mm/yyy hh:mm";
            this.lbl_baslerCalibTimestamp.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_luxonisCalibTimestamp
            // 
            this.lbl_luxonisCalibTimestamp.AutoSize = true;
            this.lbl_luxonisCalibTimestamp.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_luxonisCalibTimestamp.Font = new System.Drawing.Font("Segoe UI Light", 10F, System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_luxonisCalibTimestamp.Location = new System.Drawing.Point(435, 68);
            this.lbl_luxonisCalibTimestamp.Name = "lbl_luxonisCalibTimestamp";
            this.lbl_luxonisCalibTimestamp.Size = new System.Drawing.Size(423, 31);
            this.lbl_luxonisCalibTimestamp.TabIndex = 7;
            this.lbl_luxonisCalibTimestamp.Text = "Last calibration: dd/mm/yyy hh:mm";
            this.lbl_luxonisCalibTimestamp.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_camera3
            // 
            this.lbl_camera3.AutoSize = true;
            this.lbl_camera3.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_camera3.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_camera3.Location = new System.Drawing.Point(866, 35);
            this.lbl_camera3.Name = "lbl_camera3";
            this.lbl_camera3.Size = new System.Drawing.Size(424, 31);
            this.lbl_camera3.TabIndex = 2;
            this.lbl_camera3.Text = "BASLER";
            this.lbl_camera3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_camera2
            // 
            this.lbl_camera2.AutoSize = true;
            this.lbl_camera2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_camera2.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_camera2.Location = new System.Drawing.Point(435, 35);
            this.lbl_camera2.Name = "lbl_camera2";
            this.lbl_camera2.Size = new System.Drawing.Size(423, 31);
            this.lbl_camera2.TabIndex = 1;
            this.lbl_camera2.Text = "LUXONIS";
            this.lbl_camera2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_camera1
            // 
            this.lbl_camera1.AutoSize = true;
            this.lbl_camera1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_camera1.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_camera1.Location = new System.Drawing.Point(5, 35);
            this.lbl_camera1.Name = "lbl_camera1";
            this.lbl_camera1.Size = new System.Drawing.Size(422, 31);
            this.lbl_camera1.TabIndex = 0;
            this.lbl_camera1.Text = "IDS";
            this.lbl_camera1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // flp_ids
            // 
            this.flp_ids.AutoScroll = true;
            this.flp_ids.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flp_ids.FlowDirection = System.Windows.Forms.FlowDirection.TopDown;
            this.flp_ids.Location = new System.Drawing.Point(5, 104);
            this.flp_ids.Name = "flp_ids";
            this.flp_ids.Size = new System.Drawing.Size(422, 522);
            this.flp_ids.TabIndex = 3;
            // 
            // flp_luxonis
            // 
            this.flp_luxonis.AutoScroll = true;
            this.flp_luxonis.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flp_luxonis.FlowDirection = System.Windows.Forms.FlowDirection.TopDown;
            this.flp_luxonis.Location = new System.Drawing.Point(435, 104);
            this.flp_luxonis.Name = "flp_luxonis";
            this.flp_luxonis.Size = new System.Drawing.Size(423, 522);
            this.flp_luxonis.TabIndex = 4;
            // 
            // flp_basler
            // 
            this.flp_basler.AutoScroll = true;
            this.flp_basler.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flp_basler.FlowDirection = System.Windows.Forms.FlowDirection.TopDown;
            this.flp_basler.Location = new System.Drawing.Point(866, 104);
            this.flp_basler.Name = "flp_basler";
            this.flp_basler.Size = new System.Drawing.Size(424, 522);
            this.flp_basler.TabIndex = 5;
            // 
            // lbl_idsCalibTimestamp
            // 
            this.lbl_idsCalibTimestamp.AutoSize = true;
            this.lbl_idsCalibTimestamp.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_idsCalibTimestamp.Font = new System.Drawing.Font("Segoe UI Light", 10F, System.Drawing.FontStyle.Italic, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_idsCalibTimestamp.Location = new System.Drawing.Point(5, 68);
            this.lbl_idsCalibTimestamp.Name = "lbl_idsCalibTimestamp";
            this.lbl_idsCalibTimestamp.Size = new System.Drawing.Size(422, 31);
            this.lbl_idsCalibTimestamp.TabIndex = 6;
            this.lbl_idsCalibTimestamp.Text = "Last calibration: dd/mm/yyy hh:mm";
            this.lbl_idsCalibTimestamp.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btn_acquire
            // 
            this.btn_acquire.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_acquire.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_acquire.Location = new System.Drawing.Point(149, 5);
            this.btn_acquire.Margin = new System.Windows.Forms.Padding(20, 5, 20, 5);
            this.btn_acquire.Name = "btn_acquire";
            this.btn_acquire.Size = new System.Drawing.Size(283, 81);
            this.btn_acquire.TabIndex = 6;
            this.btn_acquire.Text = "Take picture";
            this.btn_acquire.UseVisualStyleBackColor = true;
            this.btn_acquire.Click += new System.EventHandler(this.btn_acquire_Click);
            // 
            // lbl_shots
            // 
            this.lbl_shots.AutoSize = true;
            this.lbl_shots.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_shots.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_shots.Location = new System.Drawing.Point(455, 0);
            this.lbl_shots.Name = "lbl_shots";
            this.lbl_shots.Size = new System.Drawing.Size(188, 91);
            this.lbl_shots.TabIndex = 7;
            this.lbl_shots.Text = "0/15";
            this.lbl_shots.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // btn_calibrate
            // 
            this.btn_calibrate.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_calibrate.Enabled = false;
            this.btn_calibrate.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F);
            this.btn_calibrate.Location = new System.Drawing.Point(666, 5);
            this.btn_calibrate.Margin = new System.Windows.Forms.Padding(20, 5, 20, 5);
            this.btn_calibrate.Name = "btn_calibrate";
            this.btn_calibrate.Size = new System.Drawing.Size(283, 81);
            this.btn_calibrate.TabIndex = 8;
            this.btn_calibrate.Text = "Calibrate";
            this.btn_calibrate.UseVisualStyleBackColor = true;
            this.btn_calibrate.Click += new System.EventHandler(this.btn_calibrate_Click);
            // 
            // tbl_controls
            // 
            this.tbl_controls.ColumnCount = 5;
            this.tbl_controls.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tbl_controls.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 25F));
            this.tbl_controls.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 15F));
            this.tbl_controls.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 25F));
            this.tbl_controls.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 25F));
            this.tbl_controls.Controls.Add(this.btn_home, 0, 0);
            this.tbl_controls.Controls.Add(this.btn_clear, 4, 0);
            this.tbl_controls.Controls.Add(this.lbl_shots, 2, 0);
            this.tbl_controls.Controls.Add(this.btn_acquire, 1, 0);
            this.tbl_controls.Controls.Add(this.btn_calibrate, 3, 0);
            this.tbl_controls.Dock = System.Windows.Forms.DockStyle.Bottom;
            this.tbl_controls.Location = new System.Drawing.Point(0, 631);
            this.tbl_controls.Name = "tbl_controls";
            this.tbl_controls.RowCount = 1;
            this.tbl_controls.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tbl_controls.Size = new System.Drawing.Size(1295, 91);
            this.tbl_controls.TabIndex = 2;
            // 
            // btn_home
            // 
            this.btn_home.BackgroundImage = global::OptiSort.Properties.Resources.home_2x2_pptx;
            this.btn_home.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Zoom;
            this.btn_home.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_home.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_home.Location = new System.Drawing.Point(20, 5);
            this.btn_home.Margin = new System.Windows.Forms.Padding(20, 5, 20, 5);
            this.btn_home.Name = "btn_home";
            this.btn_home.Size = new System.Drawing.Size(89, 81);
            this.btn_home.TabIndex = 10;
            this.btn_home.UseVisualStyleBackColor = true;
            this.btn_home.Click += new System.EventHandler(this.btn_home_Click);
            // 
            // btn_clear
            // 
            this.btn_clear.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_clear.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F);
            this.btn_clear.Location = new System.Drawing.Point(989, 5);
            this.btn_clear.Margin = new System.Windows.Forms.Padding(20, 5, 20, 5);
            this.btn_clear.Name = "btn_clear";
            this.btn_clear.Size = new System.Drawing.Size(286, 81);
            this.btn_clear.TabIndex = 9;
            this.btn_clear.Text = "Clear memory";
            this.btn_clear.UseVisualStyleBackColor = true;
            this.btn_clear.Click += new System.EventHandler(this.btn_clear_Click);
            // 
            // ucLensDistortionCalibration
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.tbl_images);
            this.Controls.Add(this.tbl_controls);
            this.Name = "ucLensDistortionCalibration";
            this.Size = new System.Drawing.Size(1295, 722);
            this.tbl_images.ResumeLayout(false);
            this.tbl_images.PerformLayout();
            this.tbl_controls.ResumeLayout(false);
            this.tbl_controls.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion
        private System.Windows.Forms.Label lblNote;
        private System.Windows.Forms.TableLayoutPanel tbl_images;
        private System.Windows.Forms.Label lbl_camera3;
        private System.Windows.Forms.Label lbl_camera2;
        private System.Windows.Forms.Label lbl_camera1;
        private System.Windows.Forms.FlowLayoutPanel flp_ids;
        private System.Windows.Forms.FlowLayoutPanel flp_luxonis;
        private System.Windows.Forms.FlowLayoutPanel flp_basler;
        private System.Windows.Forms.Button btn_acquire;
        private System.Windows.Forms.Label lbl_shots;
        private System.Windows.Forms.Button btn_calibrate;
        private System.Windows.Forms.TableLayoutPanel tbl_controls;
        private System.Windows.Forms.Button btn_clear;
        private System.Windows.Forms.Label lbl_baslerCalibTimestamp;
        private System.Windows.Forms.Label lbl_luxonisCalibTimestamp;
        private System.Windows.Forms.Label lbl_idsCalibTimestamp;
        private System.Windows.Forms.Button btn_home;
    }
}
