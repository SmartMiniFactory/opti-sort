namespace OptiSort.userControls
{
    partial class ucManualControl
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
            this.btn_detectionAlgorithm = new System.Windows.Forms.Button();
            this.btn_coordinateCalibration = new System.Windows.Forms.Button();
            this.btn_lensCalibration = new System.Windows.Forms.Button();
            this.btnTestDTMQTT = new System.Windows.Forms.Button();
            this.btnLight = new System.Windows.Forms.Button();
            this.btnScaraJog = new System.Windows.Forms.Button();
            this.btnFaultReset = new System.Windows.Forms.Button();
            this.btnFlexibowlShake = new System.Windows.Forms.Button();
            this.btnFlexibowlFlip = new System.Windows.Forms.Button();
            this.btnFlexibowlBw = new System.Windows.Forms.Button();
            this.btnFlexibowlFw = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.tableLayoutPanel2 = new System.Windows.Forms.TableLayoutPanel();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.tableLayoutPanel2.SuspendLayout();
            this.SuspendLayout();
            // 
            // btn_detectionAlgorithm
            // 
            this.btn_detectionAlgorithm.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_detectionAlgorithm.Font = new System.Drawing.Font("Segoe UI", 14F);
            this.btn_detectionAlgorithm.Location = new System.Drawing.Point(902, 710);
            this.btn_detectionAlgorithm.Margin = new System.Windows.Forms.Padding(20);
            this.btn_detectionAlgorithm.Name = "btn_detectionAlgorithm";
            this.btn_detectionAlgorithm.Size = new System.Drawing.Size(403, 91);
            this.btn_detectionAlgorithm.TabIndex = 11;
            this.btn_detectionAlgorithm.Text = "Define detection algorithm";
            this.btn_detectionAlgorithm.UseVisualStyleBackColor = true;
            this.btn_detectionAlgorithm.Click += new System.EventHandler(this.btn_detectionAlgorithm_Click);
            // 
            // btn_coordinateCalibration
            // 
            this.btn_coordinateCalibration.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_coordinateCalibration.Font = new System.Drawing.Font("Segoe UI", 14F);
            this.btn_coordinateCalibration.Location = new System.Drawing.Point(461, 710);
            this.btn_coordinateCalibration.Margin = new System.Windows.Forms.Padding(20);
            this.btn_coordinateCalibration.Name = "btn_coordinateCalibration";
            this.btn_coordinateCalibration.Size = new System.Drawing.Size(401, 91);
            this.btn_coordinateCalibration.TabIndex = 10;
            this.btn_coordinateCalibration.Text = "Coordinate reference frame";
            this.btn_coordinateCalibration.UseVisualStyleBackColor = true;
            this.btn_coordinateCalibration.Click += new System.EventHandler(this.btn_coordinateCalibration_Click);
            // 
            // btn_lensCalibration
            // 
            this.btn_lensCalibration.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btn_lensCalibration.Font = new System.Drawing.Font("Segoe UI", 14F);
            this.btn_lensCalibration.Location = new System.Drawing.Point(20, 710);
            this.btn_lensCalibration.Margin = new System.Windows.Forms.Padding(20);
            this.btn_lensCalibration.Name = "btn_lensCalibration";
            this.btn_lensCalibration.Size = new System.Drawing.Size(401, 91);
            this.btn_lensCalibration.TabIndex = 9;
            this.btn_lensCalibration.Text = "Camera lenses";
            this.btn_lensCalibration.UseVisualStyleBackColor = true;
            this.btn_lensCalibration.Click += new System.EventHandler(this.btn_lensCalibration_Click);
            // 
            // btnTestDTMQTT
            // 
            this.btnTestDTMQTT.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnTestDTMQTT.Font = new System.Drawing.Font("Segoe UI", 14F);
            this.btnTestDTMQTT.Location = new System.Drawing.Point(902, 480);
            this.btnTestDTMQTT.Margin = new System.Windows.Forms.Padding(20);
            this.btnTestDTMQTT.Name = "btnTestDTMQTT";
            this.btnTestDTMQTT.Size = new System.Drawing.Size(403, 90);
            this.btnTestDTMQTT.TabIndex = 13;
            this.btnTestDTMQTT.Text = "Test DT MQTT";
            this.btnTestDTMQTT.UseVisualStyleBackColor = true;
            this.btnTestDTMQTT.Click += new System.EventHandler(this.btnTestDTMQTT_Click);
            // 
            // btnLight
            // 
            this.btnLight.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnLight.Font = new System.Drawing.Font("Segoe UI", 14F);
            this.btnLight.Location = new System.Drawing.Point(461, 480);
            this.btnLight.Margin = new System.Windows.Forms.Padding(20);
            this.btnLight.Name = "btnLight";
            this.btnLight.Size = new System.Drawing.Size(401, 90);
            this.btnLight.TabIndex = 12;
            this.btnLight.Text = "Toggle ring light";
            this.btnLight.UseVisualStyleBackColor = true;
            this.btnLight.Click += new System.EventHandler(this.btnLight_Click);
            // 
            // btnScaraJog
            // 
            this.btnScaraJog.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnScaraJog.Font = new System.Drawing.Font("Segoe UI", 14F);
            this.btnScaraJog.Location = new System.Drawing.Point(20, 480);
            this.btnScaraJog.Margin = new System.Windows.Forms.Padding(20);
            this.btnScaraJog.Name = "btnScaraJog";
            this.btnScaraJog.Size = new System.Drawing.Size(401, 90);
            this.btnScaraJog.TabIndex = 6;
            this.btnScaraJog.Text = "Open Manual Control";
            this.btnScaraJog.UseVisualStyleBackColor = true;
            this.btnScaraJog.Click += new System.EventHandler(this.btnScaraJog_Click);
            // 
            // btnFaultReset
            // 
            this.btnFaultReset.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnFaultReset.Font = new System.Drawing.Font("Segoe UI", 14F);
            this.btnFaultReset.Location = new System.Drawing.Point(902, 120);
            this.btnFaultReset.Margin = new System.Windows.Forms.Padding(20);
            this.btnFaultReset.Name = "btnFaultReset";
            this.tableLayoutPanel2.SetRowSpan(this.btnFaultReset, 2);
            this.btnFaultReset.Size = new System.Drawing.Size(403, 220);
            this.btnFaultReset.TabIndex = 7;
            this.btnFaultReset.Text = "Fault reset";
            this.btnFaultReset.UseVisualStyleBackColor = true;
            this.btnFaultReset.Click += new System.EventHandler(this.btnFaultReset_Click);
            // 
            // btnFlexibowlShake
            // 
            this.btnFlexibowlShake.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnFlexibowlShake.Font = new System.Drawing.Font("Segoe UI", 14F);
            this.btnFlexibowlShake.Location = new System.Drawing.Point(461, 250);
            this.btnFlexibowlShake.Margin = new System.Windows.Forms.Padding(20);
            this.btnFlexibowlShake.Name = "btnFlexibowlShake";
            this.btnFlexibowlShake.Size = new System.Drawing.Size(401, 90);
            this.btnFlexibowlShake.TabIndex = 4;
            this.btnFlexibowlShake.Text = "Shake";
            this.btnFlexibowlShake.UseVisualStyleBackColor = true;
            this.btnFlexibowlShake.Click += new System.EventHandler(this.btnFlexibowlShake_Click);
            // 
            // btnFlexibowlFlip
            // 
            this.btnFlexibowlFlip.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnFlexibowlFlip.Font = new System.Drawing.Font("Segoe UI", 14F);
            this.btnFlexibowlFlip.Location = new System.Drawing.Point(20, 250);
            this.btnFlexibowlFlip.Margin = new System.Windows.Forms.Padding(20);
            this.btnFlexibowlFlip.Name = "btnFlexibowlFlip";
            this.btnFlexibowlFlip.Size = new System.Drawing.Size(401, 90);
            this.btnFlexibowlFlip.TabIndex = 3;
            this.btnFlexibowlFlip.Text = "Filp";
            this.btnFlexibowlFlip.UseVisualStyleBackColor = true;
            this.btnFlexibowlFlip.Click += new System.EventHandler(this.btnFlexibowlFlip_Click);
            // 
            // btnFlexibowlBw
            // 
            this.btnFlexibowlBw.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnFlexibowlBw.Font = new System.Drawing.Font("Segoe UI", 14F);
            this.btnFlexibowlBw.Location = new System.Drawing.Point(461, 120);
            this.btnFlexibowlBw.Margin = new System.Windows.Forms.Padding(20);
            this.btnFlexibowlBw.Name = "btnFlexibowlBw";
            this.btnFlexibowlBw.Size = new System.Drawing.Size(401, 90);
            this.btnFlexibowlBw.TabIndex = 2;
            this.btnFlexibowlBw.Text = "Backward";
            this.btnFlexibowlBw.UseVisualStyleBackColor = true;
            this.btnFlexibowlBw.Click += new System.EventHandler(this.btnFlexibowlBw_Click);
            // 
            // btnFlexibowlFw
            // 
            this.btnFlexibowlFw.Dock = System.Windows.Forms.DockStyle.Fill;
            this.btnFlexibowlFw.Font = new System.Drawing.Font("Segoe UI", 14F);
            this.btnFlexibowlFw.Location = new System.Drawing.Point(20, 120);
            this.btnFlexibowlFw.Margin = new System.Windows.Forms.Padding(20);
            this.btnFlexibowlFw.Name = "btnFlexibowlFw";
            this.btnFlexibowlFw.Size = new System.Drawing.Size(401, 90);
            this.btnFlexibowlFw.TabIndex = 1;
            this.btnFlexibowlFw.Text = "Forward";
            this.btnFlexibowlFw.UseVisualStyleBackColor = true;
            this.btnFlexibowlFw.Click += new System.EventHandler(this.btnFlexibowlFw_Click);
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.BackColor = System.Drawing.SystemColors.GradientInactiveCaption;
            this.tableLayoutPanel2.SetColumnSpan(this.label1, 3);
            this.label1.Dock = System.Windows.Forms.DockStyle.Fill;
            this.label1.Font = new System.Drawing.Font("Segoe UI", 15F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.Location = new System.Drawing.Point(3, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(1319, 100);
            this.label1.TabIndex = 1;
            this.label1.Text = "Flexibowl control";
            this.label1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel2
            // 
            this.tableLayoutPanel2.ColumnCount = 3;
            this.tableLayoutPanel2.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 33.33333F));
            this.tableLayoutPanel2.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 33.33334F));
            this.tableLayoutPanel2.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 33.33334F));
            this.tableLayoutPanel2.Controls.Add(this.label1, 0, 0);
            this.tableLayoutPanel2.Controls.Add(this.btn_detectionAlgorithm, 2, 6);
            this.tableLayoutPanel2.Controls.Add(this.label3, 0, 5);
            this.tableLayoutPanel2.Controls.Add(this.btn_coordinateCalibration, 1, 6);
            this.tableLayoutPanel2.Controls.Add(this.btnTestDTMQTT, 2, 4);
            this.tableLayoutPanel2.Controls.Add(this.btn_lensCalibration, 0, 6);
            this.tableLayoutPanel2.Controls.Add(this.label2, 0, 3);
            this.tableLayoutPanel2.Controls.Add(this.btnLight, 1, 4);
            this.tableLayoutPanel2.Controls.Add(this.btnScaraJog, 0, 4);
            this.tableLayoutPanel2.Controls.Add(this.btnFlexibowlFw, 0, 1);
            this.tableLayoutPanel2.Controls.Add(this.btnFlexibowlBw, 1, 1);
            this.tableLayoutPanel2.Controls.Add(this.btnFlexibowlFlip, 0, 2);
            this.tableLayoutPanel2.Controls.Add(this.btnFlexibowlShake, 1, 2);
            this.tableLayoutPanel2.Controls.Add(this.btnFaultReset, 2, 1);
            this.tableLayoutPanel2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.tableLayoutPanel2.Location = new System.Drawing.Point(0, 0);
            this.tableLayoutPanel2.Name = "tableLayoutPanel2";
            this.tableLayoutPanel2.RowCount = 7;
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 100F));
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 25F));
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 25F));
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 100F));
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 25F));
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Absolute, 100F));
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 25F));
            this.tableLayoutPanel2.Size = new System.Drawing.Size(1325, 821);
            this.tableLayoutPanel2.TabIndex = 2;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.BackColor = System.Drawing.SystemColors.GradientInactiveCaption;
            this.tableLayoutPanel2.SetColumnSpan(this.label3, 3);
            this.label3.Dock = System.Windows.Forms.DockStyle.Fill;
            this.label3.Font = new System.Drawing.Font("Segoe UI", 15F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.Location = new System.Drawing.Point(3, 590);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(1319, 100);
            this.label3.TabIndex = 0;
            this.label3.Text = "Cameras control";
            this.label3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.BackColor = System.Drawing.SystemColors.GradientInactiveCaption;
            this.tableLayoutPanel2.SetColumnSpan(this.label2, 3);
            this.label2.Dock = System.Windows.Forms.DockStyle.Fill;
            this.label2.Font = new System.Drawing.Font("Segoe UI", 15F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.Location = new System.Drawing.Point(3, 360);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(1319, 100);
            this.label2.TabIndex = 0;
            this.label2.Text = "Scara control";
            this.label2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // ucManualControl
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.tableLayoutPanel2);
            this.Name = "ucManualControl";
            this.Size = new System.Drawing.Size(1325, 821);
            this.tableLayoutPanel2.ResumeLayout(false);
            this.tableLayoutPanel2.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion
        private System.Windows.Forms.Button btnScaraJog;
        private System.Windows.Forms.Button btnFlexibowlShake;
        private System.Windows.Forms.Button btnFlexibowlFlip;
        private System.Windows.Forms.Button btnFlexibowlBw;
        private System.Windows.Forms.Button btnFlexibowlFw;
        private System.Windows.Forms.Button btnFaultReset;
        private System.Windows.Forms.Button btn_coordinateCalibration;
        private System.Windows.Forms.Button btn_lensCalibration;
        private System.Windows.Forms.Button btn_detectionAlgorithm;
        private System.Windows.Forms.Button btnLight;
        private System.Windows.Forms.Button btnTestDTMQTT;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
    }
}
