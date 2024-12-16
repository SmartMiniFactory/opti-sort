namespace OptiSort.userControls
{
    partial class ucLensDistortionCalibration
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
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.label2 = new System.Windows.Forms.Label();
            this.lbl_camera3 = new System.Windows.Forms.Label();
            this.lbl_camera2 = new System.Windows.Forms.Label();
            this.lbl_camera1 = new System.Windows.Forms.Label();
            this.flp_ids = new System.Windows.Forms.FlowLayoutPanel();
            this.flp_luxonis = new System.Windows.Forms.FlowLayoutPanel();
            this.flp_basler = new System.Windows.Forms.FlowLayoutPanel();
            this.btn_acquire = new System.Windows.Forms.Button();
            this.lbl_shots = new System.Windows.Forms.Label();
            this.tableLayoutPanel1.SuspendLayout();
            this.SuspendLayout();
            // 
            // lblNote
            // 
            this.lblNote.AutoSize = true;
            this.tableLayoutPanel1.SetColumnSpan(this.lblNote, 3);
            this.lblNote.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lblNote.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lblNote.Location = new System.Drawing.Point(3, 0);
            this.lblNote.Name = "lblNote";
            this.lblNote.Size = new System.Drawing.Size(1289, 72);
            this.lblNote.TabIndex = 0;
            this.lblNote.Text = "Procedure: position the calibration grid in different orientations and take some " +
    "pictures!";
            this.lblNote.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.ColumnCount = 3;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 33.33332F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 33.33334F));
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 33.33334F));
            this.tableLayoutPanel1.Controls.Add(this.lblNote, 0, 0);
            this.tableLayoutPanel1.Controls.Add(this.label2, 0, 3);
            this.tableLayoutPanel1.Controls.Add(this.lbl_camera3, 2, 1);
            this.tableLayoutPanel1.Controls.Add(this.lbl_camera2, 1, 1);
            this.tableLayoutPanel1.Controls.Add(this.lbl_camera1, 0, 1);
            this.tableLayoutPanel1.Controls.Add(this.flp_ids, 0, 2);
            this.tableLayoutPanel1.Controls.Add(this.flp_luxonis, 1, 2);
            this.tableLayoutPanel1.Controls.Add(this.flp_basler, 2, 2);
            this.tableLayoutPanel1.Controls.Add(this.btn_acquire, 2, 3);
            this.tableLayoutPanel1.Controls.Add(this.lbl_shots, 1, 3);
            this.tableLayoutPanel1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel1.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 4;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 70F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 10F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(1295, 722);
            this.tableLayoutPanel1.TabIndex = 1;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.Location = new System.Drawing.Point(3, 649);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(425, 73);
            this.label2.TabIndex = 8;
            this.label2.Text = "Note: remember to check all the cameras!";
            this.label2.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // lbl_camera3
            // 
            this.lbl_camera3.AutoSize = true;
            this.lbl_camera3.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_camera3.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_camera3.Location = new System.Drawing.Point(865, 72);
            this.lbl_camera3.Name = "lbl_camera3";
            this.lbl_camera3.Size = new System.Drawing.Size(427, 72);
            this.lbl_camera3.TabIndex = 2;
            this.lbl_camera3.Text = "BASLER";
            this.lbl_camera3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_camera2
            // 
            this.lbl_camera2.AutoSize = true;
            this.lbl_camera2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_camera2.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_camera2.Location = new System.Drawing.Point(434, 72);
            this.lbl_camera2.Name = "lbl_camera2";
            this.lbl_camera2.Size = new System.Drawing.Size(425, 72);
            this.lbl_camera2.TabIndex = 1;
            this.lbl_camera2.Text = "LUXONIS";
            this.lbl_camera2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // lbl_camera1
            // 
            this.lbl_camera1.AutoSize = true;
            this.lbl_camera1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lbl_camera1.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbl_camera1.Location = new System.Drawing.Point(3, 72);
            this.lbl_camera1.Name = "lbl_camera1";
            this.lbl_camera1.Size = new System.Drawing.Size(425, 72);
            this.lbl_camera1.TabIndex = 0;
            this.lbl_camera1.Text = "IDS";
            this.lbl_camera1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // flp_ids
            // 
            this.flp_ids.AutoScroll = true;
            this.flp_ids.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flp_ids.FlowDirection = System.Windows.Forms.FlowDirection.TopDown;
            this.flp_ids.Location = new System.Drawing.Point(3, 147);
            this.flp_ids.Name = "flp_ids";
            this.flp_ids.Size = new System.Drawing.Size(425, 499);
            this.flp_ids.TabIndex = 3;
            // 
            // flp_luxonis
            // 
            this.flp_luxonis.AutoScroll = true;
            this.flp_luxonis.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flp_luxonis.FlowDirection = System.Windows.Forms.FlowDirection.TopDown;
            this.flp_luxonis.Location = new System.Drawing.Point(434, 147);
            this.flp_luxonis.Name = "flp_luxonis";
            this.flp_luxonis.Size = new System.Drawing.Size(425, 499);
            this.flp_luxonis.TabIndex = 4;
            // 
            // flp_basler
            // 
            this.flp_basler.AutoScroll = true;
            this.flp_basler.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flp_basler.FlowDirection = System.Windows.Forms.FlowDirection.TopDown;
            this.flp_basler.Location = new System.Drawing.Point(865, 147);
            this.flp_basler.Name = "flp_basler";
            this.flp_basler.Size = new System.Drawing.Size(427, 499);
            this.flp_basler.TabIndex = 5;
            // 
            // btn_acquire
            // 
            this.btn_acquire.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_acquire.Font = new System.Drawing.Font("Microsoft Sans Serif", 15F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_acquire.Location = new System.Drawing.Point(962, 652);
            this.btn_acquire.Margin = new System.Windows.Forms.Padding(100, 3, 100, 3);
            this.btn_acquire.Name = "btn_acquire";
            this.btn_acquire.Size = new System.Drawing.Size(233, 67);
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
            this.lbl_shots.Location = new System.Drawing.Point(434, 649);
            this.lbl_shots.Name = "lbl_shots";
            this.lbl_shots.Size = new System.Drawing.Size(425, 73);
            this.lbl_shots.TabIndex = 7;
            this.lbl_shots.Text = "0/15";
            this.lbl_shots.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // ucLensDistortionCalibration
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.tableLayoutPanel1);
            this.Name = "ucLensDistortionCalibration";
            this.Size = new System.Drawing.Size(1295, 722);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion
        private System.Windows.Forms.Label lblNote;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.Label lbl_camera3;
        private System.Windows.Forms.Label lbl_camera2;
        private System.Windows.Forms.Label lbl_camera1;
        private System.Windows.Forms.FlowLayoutPanel flp_ids;
        private System.Windows.Forms.FlowLayoutPanel flp_luxonis;
        private System.Windows.Forms.FlowLayoutPanel flp_basler;
        private System.Windows.Forms.Button btn_acquire;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label lbl_shots;
    }
}
